/*
 * Shimmer WebSocket Bridge (Android)
 * Hosts a WS server and manages one Bluetooth SPP session per Shimmer MAC.
 * Applies IMU/EXG config (with board auto-detect), streams samples as JSON,
 * locks EXG mode post-connect, and handles client subscriptions per MAC.
 */


using System.Collections.Concurrent;
using System.Text;
using System.Text.Json;
using System.Net.WebSockets;
using Android.Bluetooth;
using Android.Net.Wifi;
using WatsonWebsocket;
using ShimmerSDK.Android;
using ShimmerAPI;
using System.Text.Json.Serialization;
using ShimmerBridgeScan;
using System.Net;


namespace ShimmerBridgeMangager
{

    // Operating modes for the Shimmer EXG.
    public enum ExgMode { None, ECG, EMG, ExgTest, Respiration }


    /// <summary>
    /// Shimmer sensor configuration (IMU, EXG).
    /// Set the flags to enable/disable individual sensor blocks.
    /// </summary>
    public sealed class ShimmerConfig
    {

        // --- IMU flags ---
        public bool EnableLowNoiseAccelerometer { get; set; }
        public bool EnableWideRangeAccelerometer { get; set; }
        public bool EnableGyroscope { get; set; }
        public bool EnableMagnetometer { get; set; }
        public bool EnablePressureTemperature { get; set; }
        public bool EnableBattery { get; set; }
        public bool EnableExtA6 { get; set; }
        public bool EnableExtA7 { get; set; }
        public bool EnableExtA15 { get; set; }
        public double? SamplingRate { get; set; }


        // --- EXG flags ---
        public bool EnableExg1 { get; set; }   
        public bool EnableExg2 { get; set; }   
        public bool ExgUse16Bit { get; set; }


        [JsonIgnore]    // Avoid numeric enum in JSON
        public ExgMode ExgMode { get; set; } = ExgMode.None;


        /// <summary>
        /// JSON wire representation of <see cref="ExgMode"/>.
        /// </summary>
        /// <value>
        /// Getter: returns <c>"ecg"</c>, <c>"emg"</c>, <c>"test"</c>, or <c>"resp"</c> for the current mode,
        /// or <c>null</c> when no mode is selected.
        /// Setter: parses those strings (case/format insensitive) and updates <see cref="ExgMode"/>; 
        /// blank or unknown values reset it to <see cref="ExgMode.None"/>.
        /// </value>
        [JsonPropertyName("exg_mode")]
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        public string? ExgModeWire
        {
            get => ExgMode switch
            {
                ExgMode.ECG => "ecg",
                ExgMode.EMG => "emg",
                ExgMode.ExgTest => "test",
                ExgMode.Respiration => "resp",
                _ => (string?)null
            };
            set
            {
                if (string.IsNullOrWhiteSpace(value))
                {
                    ExgMode = ExgMode.None;
                    return;
                }
                ExgMode = value.Trim().ToLowerInvariant() switch
                {
                    "ecg" => ExgMode.ECG,
                    "emg" => ExgMode.EMG,
                    "test" or "exgtest" or "exg_test" => ExgMode.ExgTest,
                    "resp" or "respiration" => ExgMode.Respiration,
                    _ => ExgMode.None
                };
            }
        }
    }


    /// <summary>
    /// WebSocket bridge for Shimmer devices: hosts a WatsonWsServer, manages one Bluetooth SPP session per MAC,
    /// and broadcasts samples and configuration updates to subscribed clients (server is authoritative over config).
    /// </summary>
    public sealed class WsBridgeManager : IDisposable
    {

        // Diagnostic message event
        public event Action<string>? Log;

        // WebSocket server instance
        private WatsonWsServer? _ws;

        // Active hardware sessions keyed by MAC address (server-managed; one session per MAC).
        private readonly ConcurrentDictionary<string, SppSession> _macSessions =
            new(StringComparer.OrdinalIgnoreCase);

        // Client subscriptions map: each clientId is associated with the set of MACs it subscribes to.
        private readonly ConcurrentDictionary<Guid, HashSet<string>> _subscriptions = new();


        /// <summary>
        /// Indicates whether the WebSocket server is currently listening.
        /// </summary>
        public bool IsRunning => _ws?.IsListening ?? false;


        /// <summary>
        /// Number of active hardware sessions currently tracked.
        /// </summary>
        public int ActiveSessionCount => _macSessions.Count;


        /// <summary>
        /// Disposes the manager by initiating shutdown of the WebSocket server and all sessions (non-blocking).
        /// </summary>
        public void Dispose() => _ = StopAsync();


        /// <summary>
        /// Returns true if at least one sensor block is enabled.
        /// </summary>
        /// <param name="c">The sensor configuration to inspect.</param>
        /// <returns><c>true</c> if any sensor is enabled; otherwise, <c>false</c>.</returns>
        public static bool AnySensorEnabled(ShimmerConfig c) =>
            c.EnableExg1 || c.EnableExg2 ||
            c.EnableLowNoiseAccelerometer || c.EnableWideRangeAccelerometer ||
            c.EnableGyroscope || c.EnableMagnetometer ||
            c.EnablePressureTemperature || c.EnableBattery ||
            c.EnableExtA6 || c.EnableExtA7 || c.EnableExtA15;


        /// <summary>
        /// Starts the WebSocket server on the device's local Wi-Fi IP and wires up handlers.
        /// On Android, HttpListener is not available: if starting the server throws
        /// HttpListenerException, we gracefully disable the WS bridge (set _ws = null)
        /// and keep the app running without crashing.
        /// </summary>
        /// <param name="activity">Android activity used to resolve the Wi-Fi service and local IP.</param>
        /// <param name="port">TCP port for the WebSocket server (default: 8787).</param>
        /// <returns>Completed task once the server is started or gracefully disabled.</returns>
        public Task StartAsync(Activity activity, int port = 8787)
        {
            // no-op if already started
            if (IsRunning) return Task.CompletedTask;

            string ip = GetLocalIp(activity);
            _ws = new WatsonWsServer(ip, port, false);

            _ws.ClientConnected += (s, e) =>
            {
                Log?.Invoke($"WS client {e.Client.Guid} connected");
                _subscriptions.TryAdd(e.Client.Guid, new HashSet<string>(StringComparer.OrdinalIgnoreCase));
            };

            _ws.ClientDisconnected += (s, e) =>
            {
                Log?.Invoke($"WS client {e.Client.Guid} disconnected");
                _subscriptions.TryRemove(e.Client.Guid, out _);
            };

            _ws.MessageReceived += OnMessage;

            try
            {
                _ws.Start();
                Log?.Invoke($"WS on ws://{ip}:{port}/");
            }
            catch (HttpListenerException ex)
            {
                // Android path: HttpListener not supported — disable WS, keep app alive
                Log?.Invoke($"WS disabled on Android: {ex.Message}");
                try { _ws.Dispose(); } catch { }
                _ws = null;
            }
            catch (Exception ex)
            {
                // Any other startup failure — disable WS to avoid crashes
                Log?.Invoke($"WS start failed: {ex.Message}");
                try { _ws?.Dispose(); } catch { }
                _ws = null;
            }

            return Task.CompletedTask;
        }


        /// <summary>
        /// Stops the WebSocket server and disposes resources; also closes all active device sessions.
        /// </summary>
        /// <returns>A task that completes when sessions are closed and the server is stopped.</returns>
        public async Task StopAsync()
        {

            // close and dispose all active SPP sessions
            await CloseAllAsync();
            if (_ws != null)
            {
                try { _ws.Stop(); } catch { }

                // release server resources
                _ws.Dispose();

                // mark as not running
                _ws = null;
            }
            Log?.Invoke("WS stopped");
        }


        /// <summary>
        /// Opens a Bluetooth SPP session for the given MAC, applies the requested configuration
        /// (with board auto-detection/adjustments), starts streaming if any sensor is enabled,
        /// locks EXG mode, and broadcasts the effective config to subscribers.
        /// </summary>
        /// <param name="mac">Target device Bluetooth MAC address (bonded).</param>
        /// <param name="cfg">Desired sensor configuration; may be adjusted based on detected board.</param>
        /// <returns>A task that completes when the session is opened, configured, and (optionally) started.</returns>
        public async Task OpenConfigureAndStartAsync(string mac, ShimmerConfig cfg)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return;

            // Close any previous session for this MAC
            if (_macSessions.TryRemove(mac, out var old))
            {
                try { old.Dispose(); } catch { }
            }

            var sess = new SppSession(
                mac,
                broadcast: (m, json) => { _ = BroadcastToSubscribers(m, json); },    // Fan-out JSON to subscribed clients
                log: msg => Log?.Invoke(msg)
            );

            // Log requested EXG mode (before applying)
            Log?.Invoke($"[SERVER] requested exg_mode (wire)='{cfg.ExgModeWire ?? "null"}' enum={cfg.ExgMode}");

            await sess.OpenAsync();             // Connect SPP
            await sess.ApplyConfigAsync(cfg);   // Apply config (auto-config IMU/EXG per board)

            var applied = sess.CurrentConfig;
            Log?.Invoke($"[SERVER] applied   exg_mode (wire)='{applied.ExgModeWire ?? "null"}' enum={applied.ExgMode}");

            // start streaming only if something is enabled
            if (AnySensorEnabled(sess.CurrentConfig)) sess.Start();

            // prevent EXG mode changes after connect
            sess.LockMode();
            _macSessions[mac] = sess;    // Track session

            // Initial broadcast of effective configuration (includes exg_mode and enabled blocks)
            await BroadcastToSubscribers(mac, JsonSerializer.Serialize(
            new { type = "config_changed", mac, cfg = sess.CurrentConfig, available = sess.EnabledBlocks() }));
        }


        /// <summary>
        /// Live-reconfigures sensors for an active device session (EXG mode stays locked after connect).
        /// Stops streaming, applies the new flags/sampling rate, then restarts if any sensor is enabled,
        /// and broadcasts the effective configuration to subscribers.
        /// </summary>
        /// <param name="mac">Target device Bluetooth MAC address.</param>
        /// <param name="cfg">Requested configuration; EXG mode will be preserved from the current session.</param>
        /// <returns>A task that completes after the session is reconfigured and notifications are sent.</returns>
        public async Task UpdateConfigAsync(string mac, ShimmerConfig cfg)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return;

            if (_macSessions.TryGetValue(mac, out var sess))
            {
                Log?.Invoke($"[SERVER] update requested exg_mode (wire)='{cfg.ExgModeWire ?? "null"}' enum={cfg.ExgMode} (mode lock={sess.IsModeLocked})");

                // Don't allow touching ExgMode after connect
                cfg.ExgMode = sess.CurrentConfig.ExgMode;

                // Pause streaming to reconfigure safely
                try { sess.Stop(); } catch { }
                await sess.ApplyConfigAsync(cfg);   // apply new flags / SR

                var applied2 = sess.CurrentConfig;
                Log?.Invoke($"[SERVER] update applied  exg_mode (wire)='{applied2.ExgModeWire ?? "null"}' enum={applied2.ExgMode}");

                // resume only if something is enabled
                if (AnySensorEnabled(sess.CurrentConfig)) sess.Start();

                // notify subscribers with the effective config + enabled blocks
                var msg = new { type = "config_changed", mac, cfg = sess.CurrentConfig, available = sess.EnabledBlocks() };
                await BroadcastToSubscribers(mac, JsonSerializer.Serialize(msg));

                Log?.Invoke($"[SERVER] Reconfigured {mac} (mode locked)");
            }
            else
            {
                Log?.Invoke($"[SERVER] UpdateConfig ignored: session not found for {mac}");
            }
        }


        /// <summary>
        /// Disposes and removes all active device sessions, without touching the WebSocket server.
        /// Safe to call multiple times.
        /// </summary>
        /// <returns>A completed task after all sessions are closed and cleared.</returns>
        public Task CloseAllAsync()
        {
            foreach (var s in _macSessions.Values) { try { s.Dispose(); } catch { } }

            // drop session map
            _macSessions.Clear();
            Log?.Invoke("[SERVER] All sessions closed");
            return Task.CompletedTask;
        }


        /// <summary>
        /// WebSocket message handler: dispatches text frames to <see cref="HandleTextAsync"/> and logs errors.
        /// Note: uses <c>async void</c> because it's an event handler (fire-and-forget).
        /// </summary>
        /// <param name="sender">The WebSocket server raising the event.</param>
        /// <param name="e">Message event data (includes client GUID, message type, and payload).</param>
        private async void OnMessage(object? sender, MessageReceivedEventArgs e)
        {
            try
            {

                // handle only text frames
                if (e.MessageType == WebSocketMessageType.Text)
                {
                    var txt = GetString(e.Data);                    // UTF-8 decode
                    await HandleTextAsync(e.Client.Guid, txt);      // route to JSON command handler
                }
            }
            catch (Exception ex)
            {
                Log?.Invoke("OnMessage error: " + ex.Message);
            }
        }


        /// <summary>
        /// Sends a one-shot configuration snapshot for the specified MAC to a single client.
        /// Includes the effective config and the list of enabled blocks.
        /// </summary>
        /// <param name="clientId">Target WebSocket client GUID.</param>
        /// <param name="mac">Device MAC address whose configuration to send.</param>
        /// <returns>
        /// A task that completes after the snapshot is queued for send;
        /// returns a completed task if the MAC is not active.
        /// </returns>
        private Task SendConfigSnapshot(Guid clientId, string mac)
        {
            if (_macSessions.TryGetValue(mac, out var sess))
            {
                var cfg = sess.CurrentConfig;
                return SendJson(clientId, new { type = "config_changed", mac, cfg, available = sess.EnabledBlocks() });
            }

            // No active session for this MAC
            return Task.CompletedTask;
        }


        /// <summary>
        /// Handles a single client JSON command: parses the message type and routes to the appropriate action,
        /// replying with typed ACKs and broadcasting updates when needed.
        /// </summary>
        /// <param name="clientId">The GUID of the WebSocket client sending the command.</param>
        /// <param name="json">Raw JSON command payload.</param>
        /// <returns>A task that completes after the command is processed and responses are queued.</returns>
        private async Task HandleTextAsync(Guid clientId, string json)
        {

            // parse JSON
            using var doc = JsonDocument.Parse(json);
            var root = doc.RootElement;
            var type = root.TryGetProperty("type", out var t) ? t.GetString() : null;

            Log?.Invoke($"WS IN [{clientId}] type={type} raw={json}");

            switch (type)
            {
                case "hello":
                    {
                        await SendJson(clientId, new { type = "hello_ack", ok = true, proto = "shimmer.v1" });
                        break;
                    }

                case "list_devices":
                    {

                        // Enumerate already bonded BT devices that look like Shimmer
                        var items = BluetoothAdapter.DefaultAdapter?.BondedDevices?
                            .Select(d => new { name = d?.Name ?? "?", mac = d?.Address ?? "" })
                            .Where(d => LooksLikeShimmer(d.name, d.mac))
                            .ToArray() ?? Array.Empty<object>();
                        await SendJson(clientId, new { type = "devices", items });
                        break;
                    }

                case "list_active":
                    {

                        // Report active sessions (MACs) managed by the server
                        var items = _macSessions.Keys.OrderBy(m => m).ToArray();
                        await SendJson(clientId, new { type = "active_devices", macs = items });
                        break;
                    }

                case "set_exg_mode":
                    {

                        // EXG mode is server-managed / locked post-connect
                        await SendJson(clientId, new { type = "set_exg_mode_ack", ok = false, error = "server_managed" });
                        break;
                    }

                case "get_config":
                    {
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        if (mac.Length == 0)
                        {
                            await SendJson(clientId, new { type = "config", ok = false, error = "no_mac" });
                            break;
                        }
                        if (TryGetConfig(mac, out var cfg))
                            await SendJson(clientId, new { type = "config", ok = true, mac, cfg });
                        else
                            await SendJson(clientId, new { type = "config", ok = false, mac, error = "not_active" });
                        break;
                    }

                case "set_sampling_rate":
                    {

                        // Validate args
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        double sr = (root.TryGetProperty("sr", out var psr) && psr.ValueKind == JsonValueKind.Number)
                                    ? psr.GetDouble() : double.NaN;

                        if (mac.Length == 0 || double.IsNaN(sr) || sr <= 0)
                        {
                            await SendJson(clientId, new { type = "set_sampling_rate_ack", ok = false, error = "bad_args" });
                            break;
                        }

                        if (_macSessions.TryGetValue(mac, out var sess))
                        {
                            double applied = await sess.SetSamplingRateAsync(sr);

                            // Direct ACK to caller with requested vs applied rate
                            await SendJson(clientId, new { type = "set_sampling_rate_ack", ok = true, mac, requested = sr, applied });

                            // Notify subscribers with updated config
                            var cfg = sess.CurrentConfig;
                            await BroadcastToSubscribers(mac, JsonSerializer.Serialize(new { type = "config_changed", mac, cfg }));
                        }
                        else
                        {
                            await SendJson(clientId, new { type = "set_sampling_rate_ack", ok = false, error = "not_active" });
                        }
                        break;
                    }

                case "open":
                    {

                        // Subscribe to a device MAC and send initial snapshot (+ retry)
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";

                        if (string.IsNullOrEmpty(mac))
                        {
                            await SendJson(clientId, new { type = "open_ack", ok = false, error = "no_mac" });
                            break;
                        }

                        if (_macSessions.TryGetValue(mac, out var sess))
                        {
                            Subscribe(clientId, mac);
                            await SendJson(clientId, new { type = "open_ack", ok = true, mac, mode = "subscribed" });
                            await SendConfigSnapshot(clientId, mac);    // Resend to mitigate timing/race on client

                            _ = Task.Run(async () => {
                                try
                                {
                                    await Task.Delay(250);
                                    await SendJson(clientId, new { type = "open_ack", ok = true, mac, mode = "subscribed" });
                                    await SendConfigSnapshot(clientId, mac);
                                }
                                catch (Exception ex) { Log?.Invoke($"open retry send error: {ex.Message}"); }
                            });
                        }
                        else
                        {
                            await SendJson(clientId, new { type = "open_ack", ok = false, error = "not_active" });
                        }
                        break;
                    }

                case "unsubscribe":
                    {
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        Unsubscribe(clientId, mac);     // Remove MAC from the client's subscription set
                        await SendJson(clientId, new { type = "unsubscribe_ack", ok = true, mac });
                        break;
                    }

                case "set_config":

                    // Configuration is server-managed; clients cannot push arbitrary configs
                    await SendJson(clientId, new { type = "config_ack", ok = false, error = "server_managed" });
                    break;

                case "start":
                    {

                        // Server-managed start; treat as subscribe + snapshot
                        string smac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        if (smac.Length > 0 && _macSessions.ContainsKey(smac))
                        {
                            Subscribe(clientId, smac);
                            await SendJson(clientId, new { type = "open_ack", ok = true, mac = smac, mode = "subscribed" });
                            await SendConfigSnapshot(clientId, smac);

                            _ = Task.Run(async () =>
                            {
                                try
                                {
                                    await Task.Delay(250);
                                    await SendJson(clientId, new { type = "open_ack", ok = true, mac = smac, mode = "subscribed" });
                                    await SendConfigSnapshot(clientId, smac);
                                }
                                catch (Exception ex)
                                {
                                    Log?.Invoke($"start retry send error: {ex.Message}");
                                }
                            });
                        }

                        await SendJson(clientId, new { type = "start_ack", ok = true, note = "server_managed" });
                        break;
                    }

                case "stop":

                    // Stopping is server-managed; client gets an ACK but no action is taken here
                    await SendJson(clientId, new { type = "stop_ack", ok = false, error = "server_managed" });
                    break;

                case "close":

                    // client requests to tear down its WS-side subscription state
                    _subscriptions.TryRemove(clientId, out _);
                    await SendJson(clientId, new { type = "close_ack", ok = true });
                    break;

                default:

                    // Unknown command
                    await SendJson(clientId, new { type = "error", error = "unknown_type" });
                    break;
            }
        }


        /// <summary>
        /// Attempts to read the current configuration for the specified MAC from the active session map.
        /// </summary>
        /// <param name="mac">Device Bluetooth MAC address.</param>
        /// <param name="cfg">On success, receives the effective <see cref="ShimmerConfig"/>; otherwise a new default instance.</param>
        /// <returns><c>true</c> if a session exists for the MAC; otherwise, <c>false</c>.</returns>
        private bool TryGetConfig(string mac, out ShimmerConfig cfg)
        {
            cfg = new ShimmerConfig();
            if (_macSessions.TryGetValue(mac, out var sess))
            {
                cfg = sess.CurrentConfig;
                return true;
            }
            return false;
        }


        /// <summary>
        /// Subscribes a client to updates for a given device MAC (enables receiving samples/config events).
        /// </summary>
        /// <param name="clientId">WebSocket client GUID.</param>
        /// <param name="mac">Device MAC to subscribe to.</param>
        private void Subscribe(Guid clientId, string mac)
        {
            var set = _subscriptions.GetOrAdd(clientId, _ => new HashSet<string>(StringComparer.OrdinalIgnoreCase));
            lock (set) set.Add(mac);     // Thread-safe add
            Log?.Invoke($"WS [{clientId}] subscribed {mac}");
        }


        /// <summary>
        /// Unsubscribes a client from a given device MAC (stops updates for that device).
        /// </summary>
        /// <param name="clientId">WebSocket client GUID.</param>
        /// <param name="mac">Device MAC to unsubscribe from.</param>
        private void Unsubscribe(Guid clientId, string mac)
        {
            if (_subscriptions.TryGetValue(clientId, out var set))
            {
                lock (set) set.Remove(mac);     // Thread-safe remove
            }
            Log?.Invoke($"WS [{clientId}] unsubscribed {mac}");
        }


        /// <summary>
        /// Broadcasts a JSON payload to all clients subscribed to the specified device MAC.
        /// </summary>
        /// <param name="mac">Device MAC whose subscribers should receive the message.</param>
        /// <param name="json">Pre-serialized JSON payload to send.</param>
        /// <returns>A task that completes when all sends are awaited.</returns>
        private async Task BroadcastToSubscribers(string mac, string json)
        {
            if (_ws == null) return;

            var tasks = new List<Task>();
            foreach (var kv in _subscriptions)
            {
                var clientId = kv.Key;
                var set = kv.Value;
                bool send;

                // Send only to clients subscribed to this MAC
                lock (set) send = set.Contains(mac);    
                if (send) tasks.Add(SafeSend(clientId, json));  // Enqueue send
            }

            try { await Task.WhenAll(tasks); }                  // Await all sends
            catch (Exception ex)
            {
                Log?.Invoke($"Broadcast error: {ex.Message}");
            }
        }


        /// <summary>
        /// Serializes an object to JSON and sends it to a single client.
        /// No-op if the WebSocket server is not running.
        /// </summary>
        /// <param name="id">Target WebSocket client GUID.</param>
        /// <param name="obj">Object to serialize and send as JSON.</param>
        /// <returns>A task representing the send operation, or a completed task if server is not running.</returns>
        private Task SendJson(Guid id, object obj)
        {

            // Server not running
            if (_ws == null) return Task.CompletedTask;
            try
            {
                string msg = JsonSerializer.Serialize(obj);
                return SafeSend(id, msg);
            }
            catch (Exception ex)
            {
                Log?.Invoke("WS send error: " + ex.Message);
                return Task.CompletedTask;
            }
        }


        /// <summary>
        /// Manages a single Bluetooth SPP session to a Shimmer device:
        /// connects/disconnects, applies configuration, resolves channel indices,
        /// streams samples via the broadcast callback,
        /// supports sampling-rate changes, and locks EXG mode after connect.
        /// </summary>
        private sealed class SppSession : IDisposable
        {

            // Target device MAC address
            readonly string _mac;

            // Fan-out callback: (mac, jsonPayload)
            readonly Action<string, string> _broadcast;

            // Session-level diagnostic logger

            readonly Action<string> _log;

            // Shimmer Android BT core (SPP)
            ShimmerLogAndStreamAndroidBluetoothV2? _core;

            // Data/event handler registration
            EventHandler? _handler;

            // Save last applied configuration
            ShimmerConfig _currentCfg = new ShimmerConfig();

            // Prevents EXG mode changes after connect
            bool _modeLocked = false;

            // Absolute timestamp of first packet in the session; used to compute per-sample relative time
            double? _tsBase = null;

            // Resolved indices into ObjectCluster (lazy-filled)
            int iTs = -1;   // Timestamp 

            // EXG
            int iExg1 = -1, iExg2 = -1;

            // IMU
            int iLnaX = -1, iLnaY = -1, iLnaZ = -1;   // Low-Noise Accelerometer
            int iWraX = -1, iWraY = -1, iWraZ = -1;   // Wide-Range Accelerometer
            int iGx = -1, iGy = -1, iGz = -1;         // Gyroscope
            int iMx = -1, iMy = -1, iMz = -1;         // Magnetometer
            int iTemp = -1, iPress = -1, iVbatt = -1; // BMP180 Temp/Press, Battery
            int iA6 = -1, iA7 = -1, iA15 = -1;        // Ext ADC

            // Last-known scalar readings
            double? _lastVbatt = null;
            double? _lastA6 = null, _lastA7 = null, _lastA15 = null;


            /// <summary>
            /// Permanently locks the current EXG mode for this session.
            /// </summary>
            public void LockMode() => _modeLocked = true;


            /// <summary>
            /// Gets a value indicating whether the EXG mode is locked for this session.
            /// </summary>
            /// <value><c>true</c> if the EXG mode is locked; otherwise, <c>false</c>.</value>
            public bool IsModeLocked => _modeLocked;


            /// <summary>
            /// Gets a snapshot copy of the session’s effective configuration.
            /// The returned instance is detached (read-only) so external callers
            /// cannot mutate the internal state of the ongoing session.
            /// </summary>
            /// <return>A new <see cref="ShimmerConfig"/> reflecting the last applied settings.</return>
            public ShimmerConfig CurrentConfig => new ShimmerConfig
            {
                EnableLowNoiseAccelerometer = _currentCfg.EnableLowNoiseAccelerometer,
                EnableWideRangeAccelerometer = _currentCfg.EnableWideRangeAccelerometer,
                EnableGyroscope = _currentCfg.EnableGyroscope,
                EnableMagnetometer = _currentCfg.EnableMagnetometer,
                EnablePressureTemperature = _currentCfg.EnablePressureTemperature,
                EnableBattery = _currentCfg.EnableBattery,
                EnableExtA6 = _currentCfg.EnableExtA6,
                EnableExtA7 = _currentCfg.EnableExtA7,
                EnableExtA15 = _currentCfg.EnableExtA15,

                SamplingRate = _currentCfg.SamplingRate,

                EnableExg1 = _currentCfg.EnableExg1,
                EnableExg2 = _currentCfg.EnableExg2,
                ExgUse16Bit = _currentCfg.ExgUse16Bit,
                ExgMode = _currentCfg.ExgMode
            };


            /// <summary>
            /// Builds a symbolic list of enabled data blocks based on the current configuration
            /// (e.g., <c>"exg"</c>, <c>"lna"</c>, <c>"wra"</c>, <c>"gyro"</c>, <c>"mag"</c>, <c>"temp"</c>, <c>"press"</c>, <c>"vbatt"</c>, <c>"ext"</c>).
            /// Intended for the client UI to decide which panels/streams to show.
            /// </summary>
            /// <returns>A read-only list of short block keys representing enabled sensors.</returns>
            public IReadOnlyList<string> EnabledBlocks()
            {
                var list = new List<string>();
                if (_currentCfg.EnableExg1 || _currentCfg.EnableExg2) list.Add("exg");
                if (_currentCfg.EnableLowNoiseAccelerometer) list.Add("lna");
                if (_currentCfg.EnableWideRangeAccelerometer) list.Add("wra");
                if (_currentCfg.EnableGyroscope) list.Add("gyro");
                if (_currentCfg.EnableMagnetometer) list.Add("mag");
                if (_currentCfg.EnablePressureTemperature) { list.Add("temp"); list.Add("press"); }
                if (_currentCfg.EnableBattery) list.Add("vbatt");
                if (_currentCfg.EnableExtA6) list.Add("ext6");
                if (_currentCfg.EnableExtA7) list.Add("ext7");
                if (_currentCfg.EnableExtA15) list.Add("ext15");
                return list;
            }


            /// <summary>
            /// Resets all cached channel indices to <c>-1</c> so they will be re-resolved on the next data packet.
            /// </summary>
            void ResetIndices()
            {
                iTs = -1;
                iExg1 = iExg2 = -1;
                iLnaX = iLnaY = iLnaZ = -1;
                iWraX = iWraY = iWraZ = -1;
                iGx = iGy = iGz = -1;
                iMx = iMy = iMz = -1;
                iTemp = iPress = iVbatt = -1;
                iA6 = iA7 = iA15 = -1;
            }


            /// <summary>
            /// Lazily resolves any missing channel indices in the given <see cref="ObjectCluster"/>.
            /// so downstream reads can use fast index-based access instead of repeated lookups.
            /// Only fills indices that are still <c>-1</c>; existing indices are left untouched.
            /// </summary>
            /// <param name="oc">The current Shimmer <see cref="ObjectCluster"/> packet to probe for indices.</param>
            void RefreshMissingIndices(ObjectCluster oc)
            {
                if (iTs == -1)
                    iTs = SafeIdx(oc, ShimmerConfiguration.SignalNames.SYSTEM_TIMESTAMP, "CAL");

                // EXG
                const string EXG_FMT = "CAL";

                if (_currentCfg.EnableExg1 && iExg1 == -1)
                    iExg1 = TryIdx(oc,
                        ("EXG_CH1", EXG_FMT),
                        (Shimmer3Configuration.SignalNames.EXG1_CH1, EXG_FMT),
                        ("ECG_CH1", EXG_FMT), ("ECG CH1", EXG_FMT), ("EMG_CH1", EXG_FMT), ("EMG CH1", EXG_FMT),
                        ("ECG RA-LL", EXG_FMT), ("ECG LL-RA", EXG_FMT)
                    );

                if (_currentCfg.EnableExg2 && iExg2 == -1)
                    iExg2 = TryIdx(oc,
                        ("EXG_CH2", EXG_FMT),
                        (Shimmer3Configuration.SignalNames.EXG2_CH1, EXG_FMT),
                        ("ECG_CH2", EXG_FMT), ("ECG CH2", EXG_FMT), ("EMG_CH2", EXG_FMT), ("EMG CH2", EXG_FMT),
                        ("ECG LA-RA", EXG_FMT)
                    );

                // IMU
                if (_currentCfg.EnableLowNoiseAccelerometer)
                {
                    if (iLnaX == -1) iLnaX = TryIdx(oc, ("Low Noise Accelerometer X", "CAL"), ("Accelerometer X", "CAL"), ("LN_ACC_X", "CAL"), ("Low Noise Accelerometer X", "RAW"));
                    if (iLnaY == -1) iLnaY = TryIdx(oc, ("Low Noise Accelerometer Y", "CAL"), ("Accelerometer Y", "CAL"), ("LN_ACC_Y", "CAL"), ("Low Noise Accelerometer Y", "RAW"));
                    if (iLnaZ == -1) iLnaZ = TryIdx(oc, ("Low Noise Accelerometer Z", "CAL"), ("Accelerometer Z", "CAL"), ("LN_ACC_Z", "CAL"), ("Low Noise Accelerometer Z", "RAW"));
                }
                if (_currentCfg.EnableWideRangeAccelerometer)
                {
                    if (iWraX == -1) iWraX = TryIdx(oc, ("Wide Range Accelerometer X", "CAL"), ("WR Accel X", "CAL"), ("WR_ACC_X", "CAL"), ("Wide Range Accelerometer X", "RAW"));
                    if (iWraY == -1) iWraY = TryIdx(oc, ("Wide Range Accelerometer Y", "CAL"), ("WR Accel Y", "CAL"), ("WR_ACC_Y", "CAL"), ("Wide Range Accelerometer Y", "RAW"));
                    if (iWraZ == -1) iWraZ = TryIdx(oc, ("Wide Range Accelerometer Z", "CAL"), ("WR Accel Z", "CAL"), ("WR_ACC_Z", "CAL"), ("Wide Range Accelerometer Z", "RAW"));
                }
                if (_currentCfg.EnableGyroscope)
                {
                    if (iGx == -1) iGx = TryIdx(oc, ("Gyroscope X", "CAL"), ("Gyro X", "CAL"), ("GYRO_X", "CAL"), ("Gyroscope X", "RAW"));
                    if (iGy == -1) iGy = TryIdx(oc, ("Gyroscope Y", "CAL"), ("Gyro Y", "CAL"), ("GYRO_Y", "CAL"), ("Gyroscope Y", "RAW"));
                    if (iGz == -1) iGz = TryIdx(oc, ("Gyroscope Z", "CAL"), ("Gyro Z", "CAL"), ("GYRO_Z", "CAL"), ("Gyroscope Z", "RAW"));
                }
                if (_currentCfg.EnableMagnetometer)
                {
                    if (iMx == -1) iMx = TryIdx(oc, ("Magnetometer X", "CAL"), ("Mag X", "CAL"), ("MAG_X", "CAL"), ("Magnetometer X", "RAW"));
                    if (iMy == -1) iMy = TryIdx(oc, ("Magnetometer Y", "CAL"), ("Mag Y", "CAL"), ("MAG_Y", "CAL"), ("Magnetometer Y", "RAW"));
                    if (iMz == -1) iMz = TryIdx(oc, ("Magnetometer Z", "CAL"), ("Mag Z", "CAL"), ("MAG_Z", "CAL"), ("Magnetometer Z", "RAW"));
                }
                if (_currentCfg.EnablePressureTemperature)
                {
                    if (iTemp == -1) iTemp = TryIdx(oc, ("Temperature_BMP180", "CAL"), ("BMP180 Temperature", "CAL"), ("Temperature", "CAL"));
                    if (iPress == -1) iPress = TryIdx(oc, ("Pressure_BMP180", "CAL"), ("BMP180 Pressure", "CAL"), ("Pressure", "CAL"));
                }
                if (_currentCfg.EnableBattery && iVbatt == -1)
                    iVbatt = TryIdx(oc,
                        ("Battery Voltage", "CAL"), ("Battery", "CAL"),
                        ("VSense Batt", "CAL"), ("VSense_Batt", "CAL"), ("VSenseBatt", "CAL"),
                        ("VSenseReg", "CAL"), ("VBatt", "CAL"),
                        ("VSense Batt", "RAW"), ("VSense_Batt", "RAW"), ("VSenseBatt", "RAW"),
                        ("VSenseReg", "RAW"), ("VBatt", "RAW")
                    );
                if (_currentCfg.EnableExtA6 && iA6 == -1)
                    iA6 = TryIdx(oc,
                        ("External ADC A6", "CAL"), ("Ext ADC A6", "CAL"), ("Ext A6", "CAL"), ("ADC A6", "CAL"), ("Analog A6", "CAL"), ("A6", "CAL"),
                        ("External ADC A6", "RAW"), ("Ext ADC A6", "RAW"), ("Ext A6", "RAW"), ("ADC A6", "RAW"), ("Analog A6", "RAW"), ("A6", "RAW")
                    );

                if (_currentCfg.EnableExtA7 && iA7 == -1)
                    iA7 = TryIdx(oc,
                        ("External ADC A7", "CAL"), ("Ext ADC A7", "CAL"), ("Ext A7", "CAL"), ("ADC A7", "CAL"), ("Analog A7", "CAL"), ("A7", "CAL"),
                        ("External ADC A7", "RAW"), ("Ext ADC A7", "RAW"), ("Ext A7", "RAW"), ("ADC A7", "RAW"), ("Analog A7", "RAW"), ("A7", "RAW")
                    );

                if (_currentCfg.EnableExtA15 && iA15 == -1)
                    iA15 = TryIdx(oc,
                        ("External ADC A15", "CAL"), ("Ext ADC A15", "CAL"), ("Ext A15", "CAL"), ("ADC A15", "CAL"), ("Analog A15", "CAL"), ("A15", "CAL"),
                        ("External ADC A15", "RAW"), ("Ext ADC A15", "RAW"), ("Ext A15", "RAW"), ("ADC A15", "RAW"), ("Analog A15", "RAW"), ("A15", "RAW")
                    );
            }


            /// <summary>
            /// Sets the device sampling rate (in Hz). If streaming is active, it temporarily stops,
            /// applies the new rate, resets cached indices and timestamp base, updates the current
            /// configuration, and restarts streaming if at least one sensor is enabled.
            /// </summary>
            /// <param name="newHz">Requested sampling rate in Hertz.</param>
            /// <returns>The applied sampling rate (rounded to an integer, in Hertz).</returns>
            /// <exception cref="InvalidOperationException">Thrown if the session is not open.</exception>
            public async Task<double> SetSamplingRateAsync(double newHz)
            {
                if (_core == null) throw new InvalidOperationException("Not open");
                bool wasStreaming = _handler != null;

                if (wasStreaming)
                {
                    try { Stop(); } catch { }
                    await Task.Delay(100);
                }

                int sr = (int)Math.Round(newHz);
                _core.WriteSamplingRate(sr);
                await Task.Delay(250);

                ResetIndices();
                _tsBase = null;

                _currentCfg.SamplingRate = sr;

                if (wasStreaming && WsBridgeManager.AnySensorEnabled(_currentCfg))
                {
                    Start();
                }

                _log($"[CFG] sampling rate set to {sr} Hz");
                return sr;
            }


            /// <summary>
            /// Creates a new Shimmer SPP session bound to the specified device and callbacks.
            /// </summary>
            /// <param name="mac">
            /// Bluetooth MAC address of the target device. The value is normalized (trimmed) and
            /// must refer to a previously bonded Shimmer node.
            /// </param>
            /// <param name="broadcast">
            /// Callback used to publish pre-serialized JSON payloads to WebSocket subscribers.
            /// The delegate is invoked as <c>(mac, json)</c> for each outgoing message.
            /// </param>
            /// <param name="log">
            /// Session-level diagnostic logger invoked with human-readable messages (info/warn/error).
            /// </param>
            public SppSession(string mac, Action<string, string> broadcast, Action<string> log)
            {
                _mac = (mac ?? string.Empty).Trim();
                _broadcast = broadcast;
                _log = log;
            }


            /// <summary>
            /// Opens the SPP connection to the Shimmer device and waits until the adapter reports a connected state.
            /// Ensures streaming is stopped after connect so configuration can be safely applied.
            /// </summary>
            /// <returns>A task that completes when the device is connected.</returns>
            /// <exception cref="InvalidOperationException">Thrown if the connection does not reach a connected state within the timeout.</exception>
            public async Task OpenAsync()
            {

                // init core for target MAC
                _core = new ShimmerLogAndStreamAndroidBluetoothV2("ShimmerBridge", _mac);

                var tcs = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);

                // Observe state changes and resolve when we see a CONNECTED state.
                EventHandler stateHandler = (s, e) =>
                {
                    try
                    {
                        var ev = (CustomEventArgs)e;
                        if (ev.getIndicator() == (int)ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_STATE_CHANGE)
                        {
                            if (IsConnectedState(ev.getObject()))
                                tcs.TrySetResult(true);
                        }
                    }
                    catch {}
                };

                _core.UICallback += stateHandler;   // subscribe to state changes
                _core.Connect();                    // start BT SPP connect

                // Wait for either CONNECTED or timeout (10s).
                var final = await Task.WhenAny(tcs.Task, Task.Delay(10000));
                _core.UICallback -= stateHandler;   // always unhook

                if (final != tcs.Task) throw new InvalidOperationException("SPP connect timeout");

                _log($"[BT] Connected to {_mac}");

                // Ensure streaming is stopped after connect; we will (re)start after config.
                try { _core.StopStreaming(); } catch { }
                await Task.Delay(150);
            }


            /// <summary>
            /// Applies the requested configuration to the connected Shimmer device:
            /// locks EXG mode if already frozen, auto-detects board kind (EXG/IMU) to adjust flags,
            /// writes sampling rate and sensor bitmap, resets cached indices, and stores the
            /// effective configuration for later reporting.
            /// </summary>
            /// <param name="cfg">Desired configuration to apply (may be adjusted based on board detection and lock state).</param>
            /// <returns>A task that completes when the configuration has been written to the device.</returns>
            /// <exception cref="InvalidOperationException">Thrown if the session is not open.</exception>
            public async Task ApplyConfigAsync(ShimmerConfig cfg)
            {

                // Preserve EXG mode if it was locked after connect
                if (_modeLocked)
                {
                    cfg.ExgMode = _currentCfg.ExgMode;
                }

                if (_core == null) throw new InvalidOperationException("Not open");

                // Auto-adjust flags based on detected board (EXG vs IMU); keep UI IMU flags on EXG boards
                try
                {
                    if (ShimmerScanManager.ShimmerBoardDetector.TryDetectBoardKind(_core, out var kind, out var rawId))
                    {

                        if (kind == ShimmerScanManager.ShimmerBoardDetector.BoardKind.EXG)
                        {

                            // Ensure EXG is enabled at 24-bit.
                            cfg.EnableExg1 = true;
                            cfg.EnableExg2 = true;
                            cfg.ExgUse16Bit = false;

                            _log($"[CFG] Board={rawId} → HYBRID (EXG on; IMU via UI)");
                        }

                        else if (kind == ShimmerScanManager.ShimmerBoardDetector.BoardKind.IMU)
                        {

                            // If user enabled no IMU blocks, enable a sensible default IMU set
                            bool anyImu = cfg.EnableLowNoiseAccelerometer || cfg.EnableWideRangeAccelerometer ||
                                          cfg.EnableGyroscope || cfg.EnableMagnetometer ||
                                          cfg.EnablePressureTemperature || cfg.EnableBattery ||
                                          cfg.EnableExtA6 || cfg.EnableExtA7 || cfg.EnableExtA15;

                            if (!anyImu)
                            {
                                cfg.EnableLowNoiseAccelerometer = true;
                                cfg.EnableGyroscope = true;
                                cfg.EnableMagnetometer = true;
                                cfg.EnablePressureTemperature = true;
                                cfg.EnableBattery = true;
                            }

                            // IMU-only boards: force EXG off
                            cfg.EnableExg1 = false;
                            cfg.EnableExg2 = false;

                            _log($"[CFG] Board={rawId} → IMU mode");
                        }
                        else
                        {
                            _log("[CFG] Board detection: Unknown → using requested flags");
                        }
                    }
                    else
                    {
                        _log("[CFG] Board detection failed → using requested flags");
                    }
                }
                catch {}

                // Default SR if not specified
                if (!cfg.SamplingRate.HasValue || cfg.SamplingRate.Value <= 0)
                    cfg.SamplingRate = 51;

                int BuildMask()
                {
                    int mask = 0;

                    // EXG
                    if (cfg.EnableExg1) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG1_24BIT;
                    if (cfg.EnableExg2) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG2_24BIT;

                    // IMU
                    if (cfg.EnableLowNoiseAccelerometer) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_A_ACCEL;
                    if (cfg.EnableWideRangeAccelerometer) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_D_ACCEL;
                    if (cfg.EnableGyroscope) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_MPU9150_GYRO;
                    if (cfg.EnableMagnetometer) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG;
                    if (cfg.EnablePressureTemperature) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_BMP180_PRESSURE;
                    if (cfg.EnableBattery) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_VBATT;
                    if (cfg.EnableExtA6) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXT_A6;            
                    if (cfg.EnableExtA7) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXT_A7;
                    if (cfg.EnableExtA15) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXT_A15;

                    return mask;
                }

                // Apply SR (if provided) before enabling sensors
                if (cfg.SamplingRate.HasValue && cfg.SamplingRate.Value > 0)
                {
                    int sr = (int)Math.Round(cfg.SamplingRate.Value);
                    _core.WriteSamplingRate(sr);
                    await Task.Delay(250);

                    cfg.SamplingRate = sr;
                }

                // Apply sensor bitmap
                _core.WriteSensors(BuildMask());
                await Task.Delay(350);

                try { _core.Inquiry(); } catch { }
                await Task.Delay(300);
                try { _core.ReadCalibrationParameters("All"); } catch { }
                await Task.Delay(220);

                ResetIndices();
                _tsBase = null;

                // Clear cached indices (labels can change with new mask)
                ResetIndices();

                // Snapshot the effective configuration for later queries/broadcasts
                _currentCfg = new ShimmerConfig
                {
                    EnableLowNoiseAccelerometer = cfg.EnableLowNoiseAccelerometer,
                    EnableWideRangeAccelerometer = cfg.EnableWideRangeAccelerometer,
                    EnableGyroscope = cfg.EnableGyroscope,
                    EnableMagnetometer = cfg.EnableMagnetometer,
                    EnablePressureTemperature = cfg.EnablePressureTemperature,
                    EnableBattery = cfg.EnableBattery,
                    EnableExtA6 = cfg.EnableExtA6,
                    EnableExtA7 = cfg.EnableExtA7,
                    EnableExtA15 = cfg.EnableExtA15,
                    SamplingRate = cfg.SamplingRate,
                    EnableExg1 = cfg.EnableExg1,
                    EnableExg2 = cfg.EnableExg2,
                    ExgUse16Bit = cfg.ExgUse16Bit,
                    ExgMode = cfg.ExgMode
                };

                _log($"[CFG] ExgMode applied = {_currentCfg.ExgMode} (wire='{_currentCfg.ExgModeWire ?? "null"}')");
                _log($"[CFG] applied (SR={_currentCfg.SamplingRate:F0}Hz, EXG1={_currentCfg.EnableExg1}, EXG2={_currentCfg.EnableExg2}, IMU: LN={_currentCfg.EnableLowNoiseAccelerometer}, WR={_currentCfg.EnableWideRangeAccelerometer}, GYR={_currentCfg.EnableGyroscope}, MAG={_currentCfg.EnableMagnetometer}, BMP180={_currentCfg.EnablePressureTemperature}, VBATT={_currentCfg.EnableBattery}, EXT={_currentCfg.EnableExtA6 || _currentCfg.EnableExtA7 || _currentCfg.EnableExtA15})");
            }


            /// <summary>
            /// Starts streaming from the connected Shimmer device:
            /// wires a packet handler, resets cached indices/timestamps,
            /// reads EXG/IMU values from each packet, builds a JSON payload,
            /// and broadcasts it to subscribers.
            /// </summary>
            public void Start()
            {

                // Must be connected/opened first
                if (_core == null) throw new InvalidOperationException("Not open");

                // Ensure we don't double-subscribe the handler
                if (_handler != null)
                {
                    try { _core.UICallback -= _handler; } catch { }
                    _handler = null;
                }

                // Fresh parsing state for a new streaming session
                ResetIndices();
                _tsBase = null;

                // Packet handler: invoked for every incoming data packet
                _handler = (s, e) =>
                {
                    try
                    {
                        var ev = (CustomEventArgs)e;
                        if (ev.getIndicator() == (int)ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET)
                        {
                            var oc = ev.getObject() as ObjectCluster;
                            if (oc == null) return;

                            // Resolve indices lazily, accounting for label variations  
                            RefreshMissingIndices(oc);

                            // Compute session-relative timestamp (first packet becomes zero)
                            double? tsAbs = Val(SafeGet(oc, iTs));
                            double tsRel = 0.0;
                            if (tsAbs.HasValue)
                            {
                                if (!_tsBase.HasValue) _tsBase = tsAbs.Value;
                                tsRel = tsAbs.Value - _tsBase.Value;
                            }

                            // ---- Read sensor values ----

                            // EXG
                            double? ex1 = Val(SafeGet(oc, iExg1));
                            double? exg2 = Val(SafeGet(oc, iExg2));

                            // IMU
                            double? lnaX = Val(SafeGet(oc, iLnaX)), lnaY = Val(SafeGet(oc, iLnaY)), lnaZ = Val(SafeGet(oc, iLnaZ));
                            double? wraX = Val(SafeGet(oc, iWraX)), wraY = Val(SafeGet(oc, iWraY)), wraZ = Val(SafeGet(oc, iWraZ));
                            double? gx = Val(SafeGet(oc, iGx)), gy = Val(SafeGet(oc, iGy)), gz = Val(SafeGet(oc, iGz));
                            double? mx = Val(SafeGet(oc, iMx)), my = Val(SafeGet(oc, iMy)), mz = Val(SafeGet(oc, iMz));
                            double? temp = Val(SafeGet(oc, iTemp)), press = Val(SafeGet(oc, iPress)), vbatt = Val(SafeGet(oc, iVbatt));
                            if (vbatt.HasValue) _lastVbatt = vbatt;
                            double? a6 = Val(SafeGet(oc, iA6)), a7 = Val(SafeGet(oc, iA7)), a15 = Val(SafeGet(oc, iA15));
                            if (a6.HasValue) _lastA6 = a6;
                            if (a7.HasValue) _lastA7 = a7;
                            if (a15.HasValue) _lastA15 = a15;


                            // Build outbound JSON payload
                            var map = new Dictionary<string, object?>(StringComparer.OrdinalIgnoreCase)
                            {
                                ["type"] = "sample",
                                ["mac"] = _mac,
                                ["ts"] = tsRel
                            };

                            // Include EXG mode if set
                            if (_currentCfg.ExgMode != ExgMode.None)
                                map["exg_mode"] = _currentCfg.ExgModeWire;

                            bool hasExg = (_currentCfg.EnableExg1 && iExg1 >= 0) || (_currentCfg.EnableExg2 && iExg2 >= 0) || ex1.HasValue || exg2.HasValue;
                            map["Exg1"] = ex1 ?? 0.0;
                            map["Exg2"] = exg2 ?? 0.0;

                            // IMU blocks are added only if the corresponding sensors are enabled
                            if (_currentCfg.EnableLowNoiseAccelerometer || _currentCfg.EnableWideRangeAccelerometer ||
                                _currentCfg.EnableGyroscope || _currentCfg.EnableMagnetometer ||
                                _currentCfg.EnablePressureTemperature || _currentCfg.EnableBattery ||
                                _currentCfg.EnableExtA6 || _currentCfg.EnableExtA7 || _currentCfg.EnableExtA15)
                            {
                                if (_currentCfg.EnableLowNoiseAccelerometer)
                                    map["lna"] = new { x = lnaX ?? 0.0, y = lnaY ?? 0.0, z = lnaZ ?? 0.0 };

                                if (_currentCfg.EnableWideRangeAccelerometer)
                                    map["wra"] = new { x = wraX ?? 0.0, y = wraY ?? 0.0, z = wraZ ?? 0.0 };

                                if (_currentCfg.EnableGyroscope)
                                    map["gyro"] = new { x = gx ?? 0.0, y = gy ?? 0.0, z = gz ?? 0.0 };

                                if (_currentCfg.EnableMagnetometer)
                                    map["mag"] = new { x = mx ?? 0.0, y = my ?? 0.0, z = mz ?? 0.0 };

                                if (_currentCfg.EnablePressureTemperature)
                                {
                                    if (temp.HasValue) map["temp"] = temp.Value;
                                    if (press.HasValue) map["press"] = press.Value;
                                }
                                if (_currentCfg.EnableBattery && _lastVbatt.HasValue)
                                    map["vbatt"] = _lastVbatt.Value;


                                if (_currentCfg.EnableExtA6 || _currentCfg.EnableExtA7 || _currentCfg.EnableExtA15)
                                    map["ext"] = new
                                    {
                                        a6 = _currentCfg.EnableExtA6 ? _lastA6 : (double?)null,
                                        a7 = _currentCfg.EnableExtA7 ? _lastA7 : (double?)null,
                                        a15 = _currentCfg.EnableExtA15 ? _lastA15 : (double?)null
                                    };
                            }

                            // Fan-out to WS subscribers
                            _broadcast(_mac, JsonSerializer.Serialize(map));
                        }
                    }
                    catch {}
                };

                // Subscribe handler and start firmware stream
                _core.UICallback += _handler;
                _core.StartStreaming();
            }


            /// <summary>
            /// Stops the current streaming session: unsubscribes the packet handler
            /// and sends a stop command to the device.
            /// </summary>
            public void Stop()
            {
                try { if (_core != null && _handler != null) _core.UICallback -= _handler; } catch { }
                try { _core?.StopStreaming(); } catch { }
            }


            /// <summary>
            /// Releases resources held by this session: stops streaming (if active),
            /// disconnects the underlying Bluetooth link, and clears the core handle.
            /// </summary>
            public void Dispose()
            {
                Stop();
                try { _core?.Disconnect(); } catch { }
                _core = null;
            }


            /// <summary>
            /// Safely resolves the index of a signal in an <see cref="ObjectCluster"/> by name/format.
            /// Returns -1 if the label is missing or the call throws.
            /// </summary>
            /// <param name="oc">The Shimmer packet container.</param>
            /// <param name="name">Signal label (e.g., "Gyroscope X").</param>
            /// <param name="fmt">Format key (e.g., "CAL" or "RAW").</param>
            /// <returns>The zero-based index if found; otherwise -1.</returns>
            static int SafeIdx(ObjectCluster oc, string name, string fmt)
            {
                try { return oc.GetIndex(name, fmt); } catch { return -1; }
            }


            /// <summary>
            /// Safely retrieves a sensor datum by index from an <see cref="ObjectCluster"/>.
            /// Returns <c>null</c> if the index is invalid or access throws.
            /// </summary>
            /// <param name="oc">The Shimmer packet container.</param>
            /// <param name="idx">Index previously obtained via <see cref="SafeIdx"/>.</param>
            /// <returns>The <see cref="SensorData"/> if available; otherwise <c>null</c>.</returns>
            static SensorData? SafeGet(ObjectCluster oc, int idx)
            {
                try { return idx >= 0 ? oc.GetData(idx) : null; } catch { return null; }
            }


            /// <summary>
            /// Converts a <see cref="SensorData"/> value to <see cref="double"/> when possible.
            /// Returns <c>null</c> on conversion errors or when the input is <c>null</c>.
            /// </summary>
            /// <param name="s">The sensor data to convert.</param>
            /// <returns>The numeric value, or <c>null</c> if unavailable.</returns>
            static double? Val(SensorData? s)
            {
                try { return s == null ? (double?)null : Convert.ToDouble(s.Data); }
                catch { return null; }
            }


            /// <summary>
            /// Determines whether a state object from the Shimmer callback represents a connected state.
            /// Supports multiple shapes:
            ///  - <see cref="int"/> values 2 or 3
            ///  - <see cref="Java.Lang.Integer"/> values 2 or 3
            ///  - Strings that contain "CONNECTED" or "CONNECT" (case-insensitive),
            ///    while explicitly excluding any string that contains "DISCONNECT".
            /// </summary>
            /// <param name="o">The state object provided by the callback.</param>
            /// <returns><c>true</c> if the device is connected; otherwise, <c>false</c>.</returns>
            static bool IsConnectedState(object? o)
            {
                if (o == null) return false;

                try
                {
                    if (o is int i) return i == 2 || i == 3;
                    if (o is Java.Lang.Integer ji) { var v = ji.IntValue(); return v == 2 || v == 3; }

                    var s = o.ToString() ?? string.Empty;

                    if (s.IndexOf("DISCONNECT", StringComparison.OrdinalIgnoreCase) >= 0)
                        return false;

                    if (s.IndexOf("CONNECTED", StringComparison.OrdinalIgnoreCase) >= 0) return true;
                    if (s.IndexOf("CONNECT", StringComparison.OrdinalIgnoreCase) >= 0) return true;

                    return false;
                }
                catch
                {
                    return false;
                }
            }


            /// <summary>
            /// Tries multiple (name, format) label candidates and returns the first matching index.
            /// Useful to handle firmware/label variations across builds.
            /// </summary>
            /// <param name="oc">The Shimmer packet container.</param>
            /// <param name="cands">Candidate tuples of signal name and format (e.g., ("Gyroscope X","CAL")).</param>
            /// <returns>The index of the first match; otherwise -1.</returns>
            static int TryIdx(ObjectCluster oc, params (string name, string fmt)[] cands)
            {
                foreach (var (n, f) in cands)
                {
                    int i = SafeIdx(oc, n, f);
                    if (i >= 0) return i;
                }
                return -1;
            }
        }


        /// <summary>
        /// Decide whether a bonded Bluetooth device looks like a Shimmer node.
        /// Checks common name prefixes and the vendor MAC OUI.
        /// </summary>
        /// <param name="name">Device advertised name.</param>
        /// <param name="mac">Device MAC address.</param>
        /// <returns><c>true</c> if it likely is a Shimmer; otherwise <c>false</c>.</returns>
        static bool LooksLikeShimmer(string? name, string? mac)
        {
            var n = (name ?? "").ToUpperInvariant();
            var m = (mac ?? "").ToUpperInvariant();
            if (n.Contains("SHIMMER")) return true;
            if (n.StartsWith("RNBT") || n.StartsWith("RN42") || n.StartsWith("RN-42")) return true;
            if (m.StartsWith("00:06:66")) return true;
            return false;
        }


        /// <summary>
        /// Sends a JSON text message to a specific WebSocket client, swallowing transport errors
        /// and logging them instead of throwing.
        /// </summary>
        /// <param name="clientId">Target client identifier.</param>
        /// <param name="json">UTF-8 JSON payload to send.</param>
        /// <returns>A task that completes when the send is attempted.</returns>
        private Task SafeSend(Guid clientId, string json)
        {
            if (_ws == null) return Task.CompletedTask;
            try
            {
                return _ws.SendAsync(clientId, json);
            }
            catch (Exception ex)
            {
                Log?.Invoke($"WS send error to {clientId}: {ex.Message}");
                return Task.CompletedTask;
            }
        }


        /// <summary>
        /// Returns the local IPv4 address of the Android Wi-Fi interface in dotted-quad notation.
        /// </summary>
        /// <param name="activity">
        /// Current Android <see cref="Activity"/> to access the Wi-Fi service and query the current connection info.
        /// </param>
        /// <returns>The local IPv4 address (e.g., "192.168.1.23"), or "0.0.0.0" if unavailable.</returns>
        static string GetLocalIp(Activity? activity)
        {
            var appCtx = activity?.ApplicationContext;
            var wm = appCtx?.GetSystemService(Activity.WifiService) as WifiManager;
            var ci = wm?.ConnectionInfo;
            if (ci == null) return "0.0.0.0";

            int ip = ci.IpAddress;
            return ((ip) & 0xFF) + "." + ((ip >> 8) & 0xFF) + "." + ((ip >> 16) & 0xFF) + "." + ((ip >> 24) & 0xFF);
        }


        /// <summary>
        /// Decodes a UTF-8 string from a given byte segment without extra copying.
        /// </summary>
        /// <param name="seg">The byte segment containing UTF-8 text.</param>
        /// <returns>The decoded string.</returns>
        static string GetString(ArraySegment<byte> seg) =>
            Encoding.UTF8.GetString(seg.Array!, seg.Offset, seg.Count);
    }
}
