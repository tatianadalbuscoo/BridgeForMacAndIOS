using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Net.WebSockets;
using Android.Bluetooth;
using Android.Net.Wifi;
using WatsonWebsocket;
using ShimmerSDK; // for ShimmerScanManager
using ShimmerSDK.Android;
using ShimmerAPI;
using Activity = Android.App.Activity;
using System.Text.Json.Serialization;
using ShimmerBridgeScan;

namespace Com.Example.ShimmerBridge
{
    // Modalità EXG (se vuoi distinguere UI lato client)
    public enum ExgMode { None, ECG, EMG, ExgTest, Respiration }

    // Config sensori (SR opzionale)
    public sealed class ShimmerConfig
    {
        // IMU flags
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

        // --- EXG ---
        public bool EnableExg1 { get; set; }   // EXG1 CH1/CH2
        public bool EnableExg2 { get; set; }   // EXG2 CH1/CH2
        public bool ExgUse16Bit { get; set; }  // false -> 24-bit (default)

        [JsonIgnore] // <-- evita serializzazione numerica del campo enum
        public ExgMode ExgMode { get; set; } = ExgMode.None;

        // wire name <-> enum (nessun default)
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

    public sealed class WsBridgeManager : IDisposable
    {
        public event Action<string>? Log;

        private WatsonWsServer? _ws;

        // sessioni "hardware" attive, 1 per MAC (server-managed)
        private readonly ConcurrentDictionary<string, SppSession> _macSessions =
            new(StringComparer.OrdinalIgnoreCase);

        // sottoscrizioni clientId -> set di MAC
        private readonly ConcurrentDictionary<Guid, HashSet<string>> _subscriptions = new();

        public bool IsRunning => _ws?.IsListening ?? false;
        public int ActiveSessionCount => _macSessions.Count;

        // Stream se almeno un sensore è attivo (EXG o IMU)
        public static bool AnySensorEnabled(ShimmerConfig c) =>
            c.EnableExg1 || c.EnableExg2 ||
            c.EnableLowNoiseAccelerometer || c.EnableWideRangeAccelerometer ||
            c.EnableGyroscope || c.EnableMagnetometer ||
            c.EnablePressureTemperature || c.EnableBattery ||
            c.EnableExtA6 || c.EnableExtA7 || c.EnableExtA15;

        public Task StartAsync(Activity activity, int port = 8787)
        {
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

            _ws.Start();
            Log?.Invoke($"WS on ws://{ip}:{port}/");
            return Task.CompletedTask;
        }

        public async Task StopAsync()
        {
            await CloseAllAsync();
            if (_ws != null)
            {
                try { _ws.Stop(); } catch { }
                _ws.Dispose();
                _ws = null;
            }
            Log?.Invoke("WS stopped");
        }

        public void Dispose() => _ = StopAsync();

        // ====== SERVER UI: open + config + start su un MAC (decide il server) ======
        public async Task OpenConfigureAndStartAsync(string mac, ShimmerConfig cfg)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return;

            // Chiudi eventuale precedente
            if (_macSessions.TryRemove(mac, out var old))
            {
                try { old.Dispose(); } catch { }
            }

            var sess = new SppSession(
                mac,
                broadcast: (m, json) => BroadcastToSubscribers(m, json),
                log: msg => Log?.Invoke(msg)
            );

            // logga PRIMA di applicare (scelta utente)
            Log?.Invoke($"[SERVER] requested exg_mode (wire)='{cfg.ExgModeWire ?? "null"}' enum={cfg.ExgMode}");

            await sess.OpenAsync();
            await sess.ApplyConfigAsync(cfg);   // auto-config IMU/EXG secondo board

            // logga DOPO l'applicazione (effettiva)
            var applied = sess.CurrentConfig;
            Log?.Invoke($"[SERVER] applied   exg_mode (wire)='{applied.ExgModeWire ?? "null"}' enum={applied.ExgMode}");

            if (AnySensorEnabled(sess.CurrentConfig)) sess.Start();

            sess.LockMode();
            _macSessions[mac] = sess;

            // broadcast iniziale della config effettiva (include exg_mode)
            await BroadcastToSubscribers(mac, JsonSerializer.Serialize(
            new { type = "config_changed", mac, cfg = sess.CurrentConfig, available = sess.EnabledBlocks() }));
        }

        // === Update live della configurazione sensori per un MAC attivo
        public async Task UpdateConfigAsync(string mac, ShimmerConfig cfg)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return;

            if (_macSessions.TryGetValue(mac, out var sess))
            {
                Log?.Invoke($"[SERVER] update requested exg_mode (wire)='{cfg.ExgModeWire ?? "null"}' enum={cfg.ExgMode} (mode lock={sess.IsModeLocked})");

                // non permettere di toccare ExgMode dopo il connect
                cfg.ExgMode = sess.CurrentConfig.ExgMode;

                try { sess.Stop(); } catch { }
                await sess.ApplyConfigAsync(cfg);

                var applied2 = sess.CurrentConfig;
                Log?.Invoke($"[SERVER] update applied  exg_mode (wire)='{applied2.ExgModeWire ?? "null"}' enum={applied2.ExgMode}");

                if (AnySensorEnabled(sess.CurrentConfig)) sess.Start();

                // manda la config effettivamente applicata (include exg_mode)
                var msg = new { type = "config_changed", mac, cfg = sess.CurrentConfig, available = sess.EnabledBlocks() };
                await BroadcastToSubscribers(mac, JsonSerializer.Serialize(msg));

                Log?.Invoke($"[SERVER] Reconfigured {mac} (mode locked)");
            }
            else
            {
                Log?.Invoke($"[SERVER] UpdateConfig ignored: session not found for {mac}");
            }
        }

        public async Task<bool> SetExgModeAsync(string mac, string mode)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return false;

            if (_macSessions.TryGetValue(mac, out var sess))
            {
                var m = (mode ?? "").Trim().ToLowerInvariant();
                ExgMode em = m switch
                {
                    "ecg" => ExgMode.ECG,
                    "emg" => ExgMode.EMG,
                    "test" or "exgtest" or "exg_test" => ExgMode.ExgTest,
                    "resp" or "respiration" => ExgMode.Respiration,
                    _ => sess.CurrentConfig.ExgMode
                };

                bool ok = await sess.SetExgModeAsync(em); // viene ignorato se la sessione è "mode locked"
                if (ok)
                {
                    var cfg = sess.CurrentConfig;
                    await BroadcastToSubscribers(mac, JsonSerializer.Serialize(
                    new { type = "config_changed", mac, cfg, available = sess.EnabledBlocks() }));
                }
                return ok;
            }
            return false;
        }

        // === chiusura di una singola sessione
        public Task CloseAsync(string mac)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return Task.CompletedTask;

            if (_macSessions.TryRemove(mac, out var old))
            {
                try { old.Dispose(); } catch { }
                var msg = new { type = "closed", mac };
                return BroadcastToSubscribers(mac, JsonSerializer.Serialize(msg));
            }
            return Task.CompletedTask;
        }

        public Task StopAllStreamingAsync()
        {
            foreach (var s in _macSessions.Values) { try { s.Stop(); } catch { } }
            Log?.Invoke("[SERVER] All streams stopped");
            return Task.CompletedTask;
        }

        public Task CloseAllAsync()
        {
            foreach (var s in _macSessions.Values) { try { s.Dispose(); } catch { } }
            _macSessions.Clear();
            Log?.Invoke("[SERVER] All sessions closed");
            return Task.CompletedTask;
        }

        // ====== WS handler ======
        private async void OnMessage(object? sender, MessageReceivedEventArgs e)
        {
            try
            {
                if (e.MessageType == WebSocketMessageType.Text)
                {
                    var txt = GetString(e.Data);
                    await HandleTextAsync(e.Client.Guid, txt);
                }
            }
            catch (Exception ex)
            {
                Log?.Invoke("OnMessage error: " + ex.Message);
            }
        }

        private Task SendConfigSnapshot(Guid clientId, string mac)
        {
            if (_macSessions.TryGetValue(mac, out var sess))
            {
                var cfg = sess.CurrentConfig;
                return SendJson(clientId, new { type = "config_changed", mac, cfg, available = sess.EnabledBlocks() });
            }
            return Task.CompletedTask;
        }

        private async Task HandleTextAsync(Guid clientId, string json)
        {
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
                        var items = BluetoothAdapter.DefaultAdapter?.BondedDevices?
                            .Select(d => new { name = d?.Name ?? "?", mac = d?.Address ?? "" })
                            .Where(d => LooksLikeShimmer(d.name, d.mac))
                            .ToArray() ?? Array.Empty<object>();
                        await SendJson(clientId, new { type = "devices", items });
                        break;
                    }

                case "list_active":
                    {
                        var items = _macSessions.Keys.OrderBy(m => m).ToArray();
                        await SendJson(clientId, new { type = "active_devices", macs = items });
                        break;
                    }

                case "set_exg_mode":
                    {
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

                            // ACK puntuale al chiamante
                            await SendJson(clientId, new { type = "set_sampling_rate_ack", ok = true, mac, requested = sr, applied });

                            // broadcast per i subscriber (config aggiornata)
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
                            await SendConfigSnapshot(clientId, mac);

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
                        Unsubscribe(clientId, mac);
                        await SendJson(clientId, new { type = "unsubscribe_ack", ok = true, mac });
                        break;
                    }

                case "set_config":
                    await SendJson(clientId, new { type = "config_ack", ok = false, error = "server_managed" });
                    break;

                case "start":
                    {
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
                    await SendJson(clientId, new { type = "stop_ack", ok = false, error = "server_managed" });
                    break;

                case "close":
                    _subscriptions.TryRemove(clientId, out _);
                    await SendJson(clientId, new { type = "close_ack", ok = true });
                    break;

                default:
                    await SendJson(clientId, new { type = "error", error = "unknown_type" });
                    break;
            }
        }

        // ====== helper: leggere config corrente
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

        // ====== sottoscrizioni e broadcast ======
        private void Subscribe(Guid clientId, string mac)
        {
            var set = _subscriptions.GetOrAdd(clientId, _ => new HashSet<string>(StringComparer.OrdinalIgnoreCase));
            lock (set) set.Add(mac);
            Log?.Invoke($"WS [{clientId}] subscribed {mac}");
        }

        private void Unsubscribe(Guid clientId, string mac)
        {
            if (_subscriptions.TryGetValue(clientId, out var set))
            {
                lock (set) set.Remove(mac);
            }
            Log?.Invoke($"WS [{clientId}] unsubscribed {mac}");
        }

        private async Task BroadcastToSubscribers(string mac, string json)
        {
            if (_ws == null) return;

            var tasks = new List<Task>();
            foreach (var kv in _subscriptions)
            {
                var clientId = kv.Key;
                var set = kv.Value;
                bool send;
                lock (set) send = set.Contains(mac);
                if (send) tasks.Add(SafeSend(clientId, json));
            }

            try { await Task.WhenAll(tasks); }
            catch (Exception ex)
            {
                Log?.Invoke($"Broadcast error: {ex.Message}");
            }
        }

        private Task SendJson(Guid id, object obj)
        {
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

        // ====== sessione HW ======
        private sealed class SppSession : IDisposable
        {
            readonly string _mac;
            readonly Action<string, string> _broadcast;
            readonly Action<string> _log;

            ShimmerLogAndStreamAndroidBluetoothV2? _core;
            EventHandler? _handler;

            // indici usati
            int iTs = -1;

            // EXG
            //int iExg1Ch1 = -1, iExg1Ch2 = -1, iExg2Ch1 = -1, iExg2Ch2 = -1;
            int iExg1 = -1, iExg2 = -1;

            // IMU
            int iLnaX = -1, iLnaY = -1, iLnaZ = -1;   // Low-Noise Accelerometer
            int iWraX = -1, iWraY = -1, iWraZ = -1;   // Wide-Range Accelerometer
            int iGx = -1, iGy = -1, iGz = -1;         // Gyroscope
            int iMx = -1, iMy = -1, iMz = -1;         // Magnetometer
            int iTemp = -1, iPress = -1, iVbatt = -1; // BMP180 Temp/Press, Battery
            int iA6 = -1, iA7 = -1, iA15 = -1;        // Ext ADC

            double? _lastVbatt = null;
            double? _lastA6 = null, _lastA7 = null, _lastA15 = null;

            // memorizza ultima config applicata
            ShimmerConfig _currentCfg = new ShimmerConfig();
            public ShimmerConfig CurrentConfig => new ShimmerConfig
            {
                // IMU flags
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

                        // Elenco simbolico dei blocchi che il client può mostrare
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
                if (_currentCfg.EnableExtA6 || _currentCfg.EnableExtA7 || _currentCfg.EnableExtA15) list.Add("ext");
                return list;
            }

    // dentro SppSession
    bool _modeLocked = false;
            public void LockMode() => _modeLocked = true;
            public bool IsModeLocked => _modeLocked;

            public Task<bool> SetExgModeAsync(ExgMode mode)
            {
                if (_modeLocked && mode != _currentCfg.ExgMode)
                {
                    _log("[CFG] exg_mode change ignored (locked after connect)");
                    return Task.FromResult(false);
                }
                _currentCfg.ExgMode = mode;
                return Task.FromResult(true);
            }

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

            void RefreshMissingIndices(ObjectCluster oc)
            {
                if (iTs == -1)
                    iTs = SafeIdx(oc, ShimmerConfiguration.SignalNames.SYSTEM_TIMESTAMP, "CAL");

                // === EXG: aggiungi tutti gli alias comuni ===
                if (_currentCfg.EnableExg1 && iExg1 == -1)
                    iExg1 = TryIdx(oc,
                        // underscore + varianti
                        ("EXG_CH1", "CAL"), ("EXG1_CH1", "CAL"), ("EXG1 CH1", "CAL"), ("EXG CH1", "CAL"),
                        // alias ECG/EMG
                        ("ECG_CH1", "CAL"), ("EMG_CH1", "CAL"),
                        ("ECG LA-RA", "CAL"), ("ECG RA-LL", "CAL"),
                        // raw fallback
                        ("EXG_CH1", "RAW"), ("EXG1_CH1", "RAW"), ("EXG1 CH1", "RAW")
                    );

                if (_currentCfg.EnableExg2 && iExg2 == -1)
                    iExg2 = TryIdx(oc,
                        ("EXG_CH2", "CAL"), ("EXG2_CH1", "CAL"), ("EXG2 CH1", "CAL"), ("EXG CH2", "CAL"),
                        ("ECG_CH2", "CAL"), ("EMG_CH2", "CAL"),
                        ("ECG LA-RA", "CAL"), ("ECG RA-LA", "CAL"),
                        // opzionale: alcune build mappano la respiration qui
                        ("RESP", "CAL"), ("Respiration", "CAL"),
                        ("EXG_CH2", "RAW"), ("EXG2_CH1", "RAW"), ("EXG2 CH1", "RAW")
                    );


                // === IMU (FUORI dall’if EXG!) ===
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

            // base tempo per ts relativo
            double? _tsBase = null;

            public SppSession(string mac, Action<string, string> broadcast, Action<string> log)
            {
                _mac = (mac ?? string.Empty).Trim();
                _broadcast = broadcast;
                _log = log;
            }

            public async Task OpenAsync()
            {
                _core = new ShimmerLogAndStreamAndroidBluetoothV2("ShimmerBridge", _mac);

                var tcs = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
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
                    catch { }
                };

                _core.UICallback += stateHandler;
                _core.Connect();

                var final = await Task.WhenAny(tcs.Task, Task.Delay(10000));
                _core.UICallback -= stateHandler;
                if (final != tcs.Task) throw new InvalidOperationException("SPP connect timeout");
                _log($"[BT] Connected to {_mac}");
                try { _core.StopStreaming(); } catch { }
                await Task.Delay(150);
            }

            public async Task ApplyConfigAsync(ShimmerConfig cfg)
            {
                // se la modalità è già stata “congelata” non permettere override
                if (_modeLocked)
                {
                    cfg.ExgMode = _currentCfg.ExgMode;
                }

                if (_core == null) throw new InvalidOperationException("Not open");

                // === AUTOCONFIG IN BASE ALLA BOARD RILEVATA (IMU vs EXG) ===
                try
                {
                    if (ShimmerScanManager.ShimmerBoardDetector.TryDetectBoardKind(_core, out var kind, out var rawId))
                    {
                        // DOPO: su EXG rispetta i flag scelti dall’utente per gli IMU.
                        // Manteniamo solo l’EXG acceso, per garantire ECG/EMG/Resp/Test.
                        if (kind == ShimmerScanManager.ShimmerBoardDetector.BoardKind.EXG)
                        {
                            // EXG acceso (canali principali), profondità 24-bit di default
                            cfg.EnableExg1 = true;
                            cfg.EnableExg2 = true;
                            cfg.ExgUse16Bit = false;

                            // ⛔ NON toccare i flag IMU: usa quelli della UI (cfg.* così come arrivano)
                            _log($"[CFG] Board={rawId} → HYBRID (EXG on; IMU via UI)");
                        }

                        else if (kind == ShimmerScanManager.ShimmerBoardDetector.BoardKind.IMU)
                        {
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

                            // Spegni EXG su board IMU-only
                            cfg.EnableExg1 = false;
                            cfg.EnableExg2 = false;

                            _log($"[CFG] Board={rawId} → IMU mode");
                        }
                        else
                        {
                            _log("[CFG] Board detection: Unknown → uso flags richiesti");
                        }
                    }
                    else
                    {
                        _log("[CFG] Board detection failed → uso flags richiesti");
                    }
                }
                catch { /* non bloccare la config se detection fallisce */ }

                // SR default: 512Hz per EXG, 100Hz per IMU
                if (!cfg.SamplingRate.HasValue || cfg.SamplingRate.Value <= 0)
                    cfg.SamplingRate = 100;



                int BuildMask()
                {
                    int mask = 0;

                    // --- EXG ---
                    if (cfg.EnableExg1) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG1_24BIT; // 0x10
                    if (cfg.EnableExg2) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG2_24BIT; // 0x08

                    // --- IMU ---
                    if (cfg.EnableLowNoiseAccelerometer) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_A_ACCEL;           // 0x80
                    if (cfg.EnableWideRangeAccelerometer) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_D_ACCEL;           // 0x1000
                    if (cfg.EnableGyroscope) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_MPU9150_GYRO;      // 0x040
                    if (cfg.EnableMagnetometer) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG;    // 0x20
                    if (cfg.EnablePressureTemperature) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_BMP180_PRESSURE;   // 0x40000
                    if (cfg.EnableBattery) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_VBATT;             // 0x2000

                    // --- EXT ADC ---
                    if (cfg.EnableExtA6) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXT_A6;            // 0x01
                    if (cfg.EnableExtA7) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXT_A7;            // 0x02
                    if (cfg.EnableExtA15) mask |= (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXT_A15;           // 0x0800

                    return mask;
                }

                if (cfg.SamplingRate.HasValue && cfg.SamplingRate.Value > 0)
                {
                    int sr = (int)Math.Round(cfg.SamplingRate.Value);
                    _core.WriteSamplingRate(sr);
                    await Task.Delay(250);
                }

                _core.WriteSensors(BuildMask());
                await Task.Delay(350);

                ResetIndices();

                // salva copia dell'ultima config applicata
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

            public void Start()
            {
                if (_core == null) throw new InvalidOperationException("Not open");

                if (_handler != null)
                {
                    try { _core.UICallback -= _handler; } catch { }
                    _handler = null;
                }

                ResetIndices();
                _tsBase = null;

                _handler = (s, e) =>
                {
                    try
                    {
                        var ev = (CustomEventArgs)e;
                        if (ev.getIndicator() == (int)ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET)
                        {
                            var oc = ev.getObject() as ObjectCluster;
                            if (oc == null) return;

                            RefreshMissingIndices(oc);

                            // timestamp relativo alla sessione
                            double? tsAbs = Val(SafeGet(oc, iTs));
                            double tsRel = 0.0;
                            if (tsAbs.HasValue)
                            {
                                if (!_tsBase.HasValue) _tsBase = tsAbs.Value;
                                tsRel = tsAbs.Value - _tsBase.Value; // stesse unità di tsAbs
                            }

                            // ====== LETTURA VALORI ======
                            // EXG
                            double? exg1 = Val(SafeGet(oc, iExg1));
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


                            // ====== COSTRUZIONE PAYLOAD ======
                            var map = new Dictionary<string, object?>(StringComparer.OrdinalIgnoreCase)
                            {
                                ["type"] = "sample",
                                ["mac"] = _mac,
                                ["ts"] = tsRel
                            };

                            // includi la modalità corrente nel sample (se impostata)
                            if (_currentCfg.ExgMode != ExgMode.None)
                                map["exg_mode"] = _currentCfg.ExgModeWire;

                            bool hasExg = (_currentCfg.EnableExg1 && iExg1 >= 0) || (_currentCfg.EnableExg2 && iExg2 >= 0) || exg1.HasValue || exg2.HasValue;


                                var ch1 = exg1 ?? 0.0;
                                var ch2 = exg2 ?? 0.0;


                                // Nomi attesi dalla UI
                                map["ExgCh1"] = ch1;
                                map["ExgCh2"] = ch2;

                                // Alias moderni (facoltativi; non disturbano la DataPage)
                                map["exg1"] = ch1;
                                map["exg2"] = ch2;



                            // --- IMU (annidato) ---
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

                            _broadcast(_mac, JsonSerializer.Serialize(map));
                        }
                    }
                    catch { /* ignore */ }
                };

                _core.UICallback += _handler;
                _core.StartStreaming();
            }

            public void Stop()
            {
                try { if (_core != null && _handler != null) _core.UICallback -= _handler; } catch { }
                try { _core?.StopStreaming(); } catch { }
            }

            public void Dispose()
            {
                Stop();
                try { _core?.Disconnect(); } catch { }
                _core = null;
            }

            static int SafeIdx(ObjectCluster oc, string name, string fmt)
            {
                try { return oc.GetIndex(name, fmt); } catch { return -1; }
            }
            static SensorData? SafeGet(ObjectCluster oc, int idx)
            {
                try { return idx >= 0 ? oc.GetData(idx) : null; } catch { return null; }
            }
            static double? Val(SensorData? s)
            {
                try { return s == null ? (double?)null : Convert.ToDouble(s.Data); }
                catch { return null; }
            }
            static bool IsConnectedState(object? o)
            {
                if (o == null) return false;
                try
                {
                    if (o is int i) return i == 2 || i == 3;
                    if (o is Java.Lang.Integer ji) return ji.IntValue() == 2 || ji.IntValue() == 3;
                    var s = o.ToString() ?? "";
                    return s.IndexOf("CONNECTED", StringComparison.OrdinalIgnoreCase) >= 0;
                }
                catch { return false; }
            }

            // --- helper EXG/IMU: prova più alias dei label ---
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

        // helper
        static bool LooksLikeShimmer(string? name, string? mac)
        {
            var n = (name ?? "").ToUpperInvariant();
            var m = (mac ?? "").ToUpperInvariant();
            if (n.Contains("SHIMMER")) return true;
            if (n.StartsWith("RNBT") || n.StartsWith("RN42") || n.StartsWith("RN-42")) return true;
            if (m.StartsWith("00:06:66")) return true;
            return false;
        }

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

        static string GetLocalIp(Activity activity)
        {
            var wm = (WifiManager?)activity.ApplicationContext.GetSystemService(Activity.WifiService);
            if (wm?.ConnectionInfo is null) return "0.0.0.0";
            int ip = wm.ConnectionInfo.IpAddress;
            return ((ip) & 0xFF) + "." + ((ip >> 8) & 0xFF) + "." + ((ip >> 16) & 0xFF) + "." + ((ip >> 24) & 0xFF);
        }

        static string GetString(ArraySegment<byte> seg) =>
            Encoding.UTF8.GetString(seg.Array!, seg.Offset, seg.Count);
    }
}
