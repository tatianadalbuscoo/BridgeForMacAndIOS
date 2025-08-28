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
using XR2Learn_ShimmerAPI.IMU.Android;
using ShimmerAPI;
using Activity = Android.App.Activity;

namespace Com.Example.ShimmerBridge
{
    // Config sensori (SR opzionale)
    public sealed class ShimmerConfig
    {
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
    }

    public sealed class WsBridgeManager : IDisposable
    {
        public event Action<string>? Log;

        private WatsonWsServer? _ws;

        // NEW: sessioni "hardware" attive, 1 per MAC (server-managed)
        private readonly ConcurrentDictionary<string, SppSession> _macSessions =
            new(StringComparer.OrdinalIgnoreCase);

        // NEW: sottoscrizioni clientId -> set di MAC
        private readonly ConcurrentDictionary<Guid, HashSet<string>> _subscriptions = new();

        public bool IsRunning => _ws?.IsListening ?? false;
        public int ActiveSessionCount => _macSessions.Count;

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

            await sess.OpenAsync();
            await sess.ApplyConfigAsync(cfg);
            sess.Start();

            _macSessions[mac] = sess;
            Log?.Invoke($"[SERVER] Session started for {mac}");
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

        private async Task HandleTextAsync(Guid clientId, string json)
        {
            using var doc = JsonDocument.Parse(json);
            var root = doc.RootElement;
            var type = root.TryGetProperty("type", out var t) ? t.GetString() : null;

            Log?.Invoke($"WS IN [{clientId}] type={type} raw={json}");

            switch (type)
            {
                case "hello":
                    await SendJson(clientId, new { type = "hello_ack", ok = true, proto = "shimmer.v1" });
                    break;

                // Elenco dei paired (a scopo informativo)
                case "list_devices":
                    {
                        var items = BluetoothAdapter.DefaultAdapter?.BondedDevices?
                            .Select(d => new { name = d?.Name ?? "?", mac = d?.Address ?? "" })
                            .Where(d => LooksLikeShimmer(d.name, d.mac))
                            .ToArray() ?? Array.Empty<object>();
                        await SendJson(clientId, new { type = "devices", items });
                        break;
                    }

                // NEW: elenco dei MAC attivi sul server (sessioni hardware già aperte)
                case "list_active":
                    {
                        var items = _macSessions.Keys.OrderBy(m => m).ToArray();
                        await SendJson(clientId, new { type = "active_devices", macs = items });
                        break;
                    }

                // **IMPORTANT**: in "server-managed mode" l'open vale come SUBSCRIBE
                // Se esiste già la sessione hardware per quel MAC, iscrivo questo client.
                // Non avvio nulla lato HW da qui.
                case "open":
                    {
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        if (mac.Length == 0)
                        {
                            await SendJson(clientId, new { type = "open_ack", ok = false, error = "no_mac" });
                            break;
                        }

                        if (_macSessions.ContainsKey(mac))
                        {
                            Subscribe(clientId, mac);
                            await SendJson(clientId, new { type = "open_ack", ok = true, mac, mode = "subscribed" });

                            // ritrasmetto l'ack un paio di volte: evita race su iOS
                            _ = Task.Delay(250).ContinueWith(_ =>
                                _ws?.SendAsync(clientId, JsonSerializer.Serialize(new { type = "open_ack", ok = true, mac, mode = "subscribed" })));
                            _ = Task.Delay(800).ContinueWith(_ =>
                                _ws?.SendAsync(clientId, JsonSerializer.Serialize(new { type = "open_ack", ok = true, mac, mode = "subscribed" })));
                        }
                        else
                        {
                            await SendJson(clientId, new { type = "open_ack", ok = false, error = "not_active" });
                        }
                        break;
                    }

                // Opzionale: explicit unsubscribe
                case "unsubscribe":
                    {
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        Unsubscribe(clientId, mac);
                        await SendJson(clientId, new { type = "unsubscribe_ack", ok = true, mac });
                        break;
                    }

                // In server-managed mode, da client iOS queste chiamate NON guidano l'HW.
                // Rispondiamo con ok=false (il tuo client prosegue comunque con i default del bridge).
                case "set_config":
                    await SendJson(clientId, new { type = "config_ack", ok = false, error = "server_managed" });
                    break;
                case "start":
                    {
                        // In server-managed mode lo start HW non lo guida il client.
                        // Rispondiamo comunque ok:true per non far scadere il client iOS.
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

        private Task BroadcastToSubscribers(string mac, string json)
        {
            if (_ws == null) return Task.CompletedTask;

            var tasks = new List<Task>();
            foreach (var kv in _subscriptions)
            {
                var clientId = kv.Key;
                var set = kv.Value;
                bool send = false;
                lock (set) send = set.Contains(mac);
                if (send)
                {
                    tasks.Add(_ws.SendAsync(clientId, json));
                }
            }
            return Task.WhenAll(tasks);
        }

        private Task SendJson(Guid id, object obj)
        {
            if (_ws == null) return Task.CompletedTask;
            try
            {
                string msg = JsonSerializer.Serialize(obj);
                return _ws.SendAsync(id, msg);
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

            bool _firstMap = true;
            bool _configured = false;

            // indici CAL nel primo pacchetto
            int iTs = -1,
                iLnaX = -1, iLnaY = -1, iLnaZ = -1,
                iWraX = -1, iWraY = -1, iWraZ = -1,
                iGx = -1, iGy = -1, iGz = -1,
                iMx = -1, iMy = -1, iMz = -1,
                iTemp = -1, iPress = -1, iVbatt = -1,
                iA6 = -1, iA7 = -1, iA15 = -1;

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
            }

            public async Task ApplyConfigAsync(ShimmerConfig cfg)
            {
                if (_core == null) throw new InvalidOperationException("Not open");

                int BuildMask()
                {
                    int mask = 0;
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

                if (cfg.SamplingRate.HasValue && cfg.SamplingRate.Value > 0)
                {
                    int sr = (int)Math.Round(cfg.SamplingRate.Value);
                    _core.WriteSamplingRate(sr);
                    await Task.Delay(150);
                }

                _core.WriteSensors(BuildMask());
                await Task.Delay(180);

                _firstMap = true;
                _configured = true;
                _log($"[CFG] applied");
            }

            public void Start()
            {
                if (_core == null) throw new InvalidOperationException("Not open");

                if (!_configured)
                {
                    try { _core.WriteSensors((int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_A_ACCEL); }
                    catch { }
                }

                if (_handler != null)
                {
                    try { _core.UICallback -= _handler; } catch { }
                    _handler = null;
                }

                _handler = (s, e) =>
                {
                    try
                    {
                        var ev = (CustomEventArgs)e;
                        if (ev.getIndicator() == (int)ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET)
                        {
                            var oc = ev.getObject() as ObjectCluster;
                            if (oc == null) return;

                            if (_firstMap)
                            {
                                iTs = SafeIdx(oc, ShimmerConfiguration.SignalNames.SYSTEM_TIMESTAMP, "CAL");
                                iLnaX = SafeIdx(oc, Shimmer3Configuration.SignalNames.LOW_NOISE_ACCELEROMETER_X, "CAL");
                                iLnaY = SafeIdx(oc, Shimmer3Configuration.SignalNames.LOW_NOISE_ACCELEROMETER_Y, "CAL");
                                iLnaZ = SafeIdx(oc, Shimmer3Configuration.SignalNames.LOW_NOISE_ACCELEROMETER_Z, "CAL");
                                iWraX = SafeIdx(oc, Shimmer3Configuration.SignalNames.WIDE_RANGE_ACCELEROMETER_X, "CAL");
                                iWraY = SafeIdx(oc, Shimmer3Configuration.SignalNames.WIDE_RANGE_ACCELEROMETER_Y, "CAL");
                                iWraZ = SafeIdx(oc, Shimmer3Configuration.SignalNames.WIDE_RANGE_ACCELEROMETER_Z, "CAL");
                                iGx = SafeIdx(oc, Shimmer3Configuration.SignalNames.GYROSCOPE_X, "CAL");
                                iGy = SafeIdx(oc, Shimmer3Configuration.SignalNames.GYROSCOPE_Y, "CAL");
                                iGz = SafeIdx(oc, Shimmer3Configuration.SignalNames.GYROSCOPE_Z, "CAL");
                                iMx = SafeIdx(oc, Shimmer3Configuration.SignalNames.MAGNETOMETER_X, "CAL");
                                iMy = SafeIdx(oc, Shimmer3Configuration.SignalNames.MAGNETOMETER_Y, "CAL");
                                iMz = SafeIdx(oc, Shimmer3Configuration.SignalNames.MAGNETOMETER_Z, "CAL");
                                iTemp = SafeIdx(oc, Shimmer3Configuration.SignalNames.TEMPERATURE, "CAL");
                                iPress = SafeIdx(oc, Shimmer3Configuration.SignalNames.PRESSURE, "CAL");
                                iVbatt = SafeIdx(oc, Shimmer3Configuration.SignalNames.V_SENSE_BATT, "CAL");
                                iA6 = SafeIdx(oc, Shimmer3Configuration.SignalNames.EXTERNAL_ADC_A6, "CAL");
                                iA7 = SafeIdx(oc, Shimmer3Configuration.SignalNames.EXTERNAL_ADC_A7, "CAL");
                                iA15 = SafeIdx(oc, Shimmer3Configuration.SignalNames.EXTERNAL_ADC_A15, "CAL");
                                _firstMap = false;
                            }

                            double? ts = Val(SafeGet(oc, iTs));
                            double? lnaX = Val(SafeGet(oc, iLnaX));
                            double? lnaY = Val(SafeGet(oc, iLnaY));
                            double? lnaZ = Val(SafeGet(oc, iLnaZ));
                            double? wraX = Val(SafeGet(oc, iWraX));
                            double? wraY = Val(SafeGet(oc, iWraY));
                            double? wraZ = Val(SafeGet(oc, iWraZ));
                            double? gx = Val(SafeGet(oc, iGx));
                            double? gy = Val(SafeGet(oc, iGy));
                            double? gz = Val(SafeGet(oc, iGz));
                            double? mx = Val(SafeGet(oc, iMx));
                            double? my = Val(SafeGet(oc, iMy));
                            double? mz = Val(SafeGet(oc, iMz));
                            double? temp = Val(SafeGet(oc, iTemp));
                            double? press = Val(SafeGet(oc, iPress));
                            double? vbatt = Val(SafeGet(oc, iVbatt));
                            double? a6 = Val(SafeGet(oc, iA6));
                            double? a7 = Val(SafeGet(oc, iA7));
                            double? a15 = Val(SafeGet(oc, iA15));

                            var payload = new
                            {
                                type = "sample",
                                mac = _mac, // NEW: chiave!
                                ts = ts,
                                lna = new { x = lnaX, y = lnaY, z = lnaZ },
                                wra = new { x = wraX, y = wraY, z = wraZ },
                                gyro = new { x = gx, y = gy, z = gz },
                                mag = new { x = mx, y = my, z = mz },
                                temp,
                                press,
                                vbatt,
                                ext = new { a6, a7, a15 }
                            };

                            _broadcast(_mac, JsonSerializer.Serialize(payload));
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
