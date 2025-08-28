using System;
using System.Collections.Concurrent;
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
using Activity = Android.App.Activity; // evita ambiguità con System.Diagnostics.Activity

namespace Com.Example.ShimmerBridge
{
    // Config sensori (SR opzionale: lasciamo ~51.2Hz hw se null)
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

        // sessioni WS (key = Guid client)
        private readonly ConcurrentDictionary<Guid, SppSession> _wsSessions = new();

        // sessioni UI dirette (key = MAC)
        private readonly ConcurrentDictionary<string, SppSession> _uiSessions =
            new(StringComparer.OrdinalIgnoreCase);

        public bool IsRunning => _ws?.IsListening ?? false;
        public int ActiveSessionCount => _wsSessions.Count + _uiSessions.Count;
        public bool HasActiveSessions => ActiveSessionCount > 0;

        public Task StartAsync(Activity activity, int port = 8787)
        {
            if (IsRunning) return Task.CompletedTask;

            string ip = GetLocalIp(activity);
            _ws = new WatsonWsServer(ip, port, false);

            _ws.ClientConnected += (s, e) =>
            {
                Log?.Invoke($"WS client {e.Client.Guid} connected");
            };

            _ws.ClientDisconnected += (s, e) =>
            {
                Log?.Invoke($"WS client {e.Client.Guid} disconnected");
                if (_wsSessions.TryRemove(e.Client.Guid, out var sess))
                {
                    try { sess.Dispose(); } catch { }
                }
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

        // ====== UI path: open + config + start su un MAC ======
        public async Task OpenConfigureAndStartAsync(string mac, ShimmerConfig cfg)
        {
            if (_uiSessions.TryGetValue(mac, out var old))
            {
                try { old.Stop(); old.Dispose(); } catch { }
                _uiSessions.TryRemove(mac, out _);
            }

            var sess = new SppSession(
                mac,
                sendText: _ => Task.CompletedTask, // in UI mode non serve inoltrare via WS
                msg => Log?.Invoke(msg)
            );

            await sess.OpenAsync();
            await sess.ApplyConfigAsync(cfg);
            sess.Start();

            _uiSessions[mac] = sess;
            Log?.Invoke($"UI session started for {mac}");
        }

        public Task StopAllStreamingAsync()
        {
            foreach (var s in _wsSessions.Values) { try { s.Stop(); } catch { } }
            foreach (var s in _uiSessions.Values) { try { s.Stop(); } catch { } }
            Log?.Invoke("All streams stopped");
            return Task.CompletedTask;
        }

        public Task CloseAllAsync()
        {
            foreach (var s in _wsSessions.Values) { try { s.Dispose(); } catch { } }
            _wsSessions.Clear();

            foreach (var s in _uiSessions.Values) { try { s.Dispose(); } catch { } }
            _uiSessions.Clear();

            Log?.Invoke("All sessions closed");
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
                // (Se volessi accettare binari dal client → comandi SPP, puoi gestirli qui)
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

            Log?.Invoke($"WS IN type={type} raw={json}");

            switch (type)
            {
                case "hello":
                    await SendJson(clientId, new { type = "hello_ack", ok = true, proto = "shimmer.v1" });
                    break;

                case "list_devices":
                    {
                        var items = BluetoothAdapter.DefaultAdapter?.BondedDevices?
                            .Select(d => new { name = d?.Name ?? "?", mac = d?.Address ?? "" })
                            .Where(d => LooksLikeShimmer(d.name, d.mac))
                            .ToArray() ?? Array.Empty<object>();
                        await SendJson(clientId, new { type = "devices", items });
                        break;
                    }
                case "open":
                    {
                        var mac = root.GetProperty("mac").GetString() ?? "";
                        Log?.Invoke($"WS IN open for {mac} (client {clientId})");

                        if (_wsSessions.TryGetValue(clientId, out var old)) { try { old.Dispose(); } catch { } _wsSessions.TryRemove(clientId, out _); }

                        var sess = new SppSession(
                            mac,
                            sendText: s => _ws?.SendAsync(clientId, s) ?? Task.CompletedTask,
                            log: msg => Log?.Invoke(msg)
                        );
                        _wsSessions[clientId] = sess;

                        // Avvio connessione SPP in background (non blocca l’ACK)
                        _ = sess.OpenAsync(ackEarly: true);

                        // ACK immediato
                        Log?.Invoke($"WS OUT open_ack for {mac} (client {clientId})");
                        await SendJson(clientId, new { type = "open_ack", ok = true, mac });

                        // Safety resend dopo 400ms (se per caso il primo frame andasse perso)
                        _ = Task.Delay(400).ContinueWith(_ =>
                            _ws?.SendAsync(clientId, JsonSerializer.Serialize(new { type = "open_ack", ok = true, mac })));

                        break;
                    }



                case "set_config":
                    {
                        if (!_wsSessions.TryGetValue(clientId, out var s))
                        {
                            await SendJson(clientId, new { type = "config_ack", ok = false, error = "no_session" });
                            break;
                        }

                        var cfg = JsonSerializer.Deserialize<ShimmerConfig>(json);
                        if (cfg == null)
                        {
                            await SendJson(clientId, new { type = "config_ack", ok = false, error = "bad_config" });
                            break;
                        }

                        try
                        {
                            await s.ApplyConfigAsync(cfg);
                            await SendJson(clientId, new { type = "config_ack", ok = true });
                        }
                        catch (Exception ex)
                        {
                            await SendJson(clientId, new { type = "config_ack", ok = false, error = ex.Message });
                        }
                        break;
                    }

                case "start":
                    if (_wsSessions.TryGetValue(clientId, out var s1))
                    {
                        try { s1.Start(); await SendJson(clientId, new { type = "start_ack", ok = true }); }
                        catch (Exception ex) { await SendJson(clientId, new { type = "start_ack", ok = false, error = ex.Message }); }
                    }
                    else { await SendJson(clientId, new { type = "error", error = "no_session" }); }
                    break;

                case "stop":
                    if (_wsSessions.TryGetValue(clientId, out var s2))
                    {
                        try { s2.Stop(); await SendJson(clientId, new { type = "stop_ack", ok = true }); }
                        catch (Exception ex) { await SendJson(clientId, new { type = "stop_ack", ok = false, error = ex.Message }); }
                    }
                    else { await SendJson(clientId, new { type = "error", error = "no_session" }); }
                    break;

                case "close":
                    if (_wsSessions.TryRemove(clientId, out var s3)) { try { s3.Dispose(); } catch { } }
                    await SendJson(clientId, new { type = "close_ack", ok = true });
                    break;

                default:
                    await SendJson(clientId, new { type = "error", error = "unknown_type" });
                    break;
            }
        }

        private Task SendJson(Guid id, object obj)
        {
            if (_ws == null) { Log?.Invoke("WS send skipped: server null"); return Task.CompletedTask; }
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


        // ====== Sessione con ShimmerAPI (via wrapper) ======
        private sealed class SppSession : IDisposable
        {
            readonly string _mac;
            readonly Func<string, Task> _sendTextToWs;
            readonly Action<string> _log;

            ShimmerLogAndStreamAndroidBluetoothV2? _core;
            EventHandler? _handler;

            bool _firstMap = true;
            bool _configured = false;
            ShimmerConfig? _lastCfg;

            // indici CAL nel primo pacchetto
            int iTs = -1,
                iLnaX = -1, iLnaY = -1, iLnaZ = -1,
                iWraX = -1, iWraY = -1, iWraZ = -1,
                iGx = -1, iGy = -1, iGz = -1,
                iMx = -1, iMy = -1, iMz = -1,
                iTemp = -1, iPress = -1, iVbatt = -1,
                iA6 = -1, iA7 = -1, iA15 = -1;

            public SppSession(string mac, Func<string, Task> sendText, Action<string> log)
            {
                _mac = (mac ?? string.Empty).Trim();
                _sendTextToWs = sendText;
                _log = log;
            }

            public async Task OpenAsync(bool ackEarly = false)
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
                    catch { /* ignore */ }
                };

                _core.UICallback += stateHandler;
                _core.Connect();

                if (ackEarly)
                {
                    // Non bloccare l’ACK: pulizia del handler in background
                    _ = Task.Run(async () =>
                    {
                        await Task.WhenAny(tcs.Task, Task.Delay(10000));
                        try { _core!.UICallback -= stateHandler; } catch { }
                        if (tcs.Task.IsCompleted && tcs.Task.Result) _log($"[BT] Connected to {_mac}");
                        else _log($"[BT] Connect grace elapsed (proceeding): {_mac}");
                    });
                    _log($"[BT] Connecting to {_mac}...");
                    return;
                }

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

                Exception? last = null;
                for (int attempt = 1; attempt <= 3; attempt++)
                {
                    try
                    {
                        if (cfg.SamplingRate.HasValue && cfg.SamplingRate.Value > 0)
                        {
                            int sr = (int)Math.Round(cfg.SamplingRate.Value);
                            _core.WriteSamplingRate(sr);
                            await Task.Delay(150); // piccolo margine
                        }

                        _core.WriteSensors(BuildMask());
                        await Task.Delay(180);

                        // Rimapparò gli indici al primo pacchetto
                        _firstMap = true;
                        _configured = true;
                        _lastCfg = cfg;
                        _log($"[CFG] applied (attempt {attempt})");
                        return;
                    }
                    catch (Exception ex)
                    {
                        last = ex;
                        string msg = ex.Message ?? string.Empty;
                        bool transient = msg.IndexOf("Output stream", StringComparison.OrdinalIgnoreCase) >= 0
                                         || msg.IndexOf("stream", StringComparison.OrdinalIgnoreCase) >= 0
                                         || msg.IndexOf("state", StringComparison.OrdinalIgnoreCase) >= 0;

                        _log($"[CFG] apply failed (attempt {attempt}): {msg}");

                        if (!transient || attempt == 3)
                            throw;

                        await Task.Delay(350 * attempt); // backoff
                    }
                }

                // dovrebbe essere già uscito sopra
                throw last ?? new InvalidOperationException("Unknown config error");
            }

            public void Start()
            {
                if (_core == null) throw new InvalidOperationException("Not open");

                // fallback minimo: se non configurato, accendi almeno LNA accel
                if (!_configured)
                {
                    try
                    {
                        _core.WriteSensors((int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_A_ACCEL);
                        _log("[CFG] fallback: LNA accel on");
                    }
                    catch { /* ignore */ }
                }

                // handler UICallback
                // (evito di registrarlo due volte)
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
                        int ind = ev.getIndicator();

                        if (ind == (int)ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET)
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

                            // estrai in double?
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
                                ts = ts,
                                lna = new { x = lnaX, y = lnaY, z = lnaZ },
                                wra = new { x = wraX, y = wraY, z = wraZ },
                                gyro = new { x = gx, y = gy, z = gz },
                                mag = new { x = mx, y = my, z = mz },
                                temp = temp,
                                press = press,
                                vbatt = vbatt,
                                ext = new { a6 = a6, a7 = a7, a15 = a15 }
                            };

                            var json = JsonSerializer.Serialize(payload);
                            _ = _sendTextToWs(json);
                        }
                    }
                    catch { /* no-op */ }
                };

                _core.UICallback += _handler;
                _core.StartStreaming();
            }

            public void Stop()
            {
                try
                {
                    if (_core != null && _handler != null) _core.UICallback -= _handler;
                }
                catch { }
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

                // molti binding passano un int o una stringa ("CONNECTED")
                try
                {
                    if (o is int i)
                        return i == 2 /* spesso CONNECTED = 2 */ || i == 3 /* some APIs */;
                    if (o is Java.Lang.Integer ji)
                        return ji.IntValue() == 2 || ji.IntValue() == 3;

                    var s = o.ToString() ?? "";
                    return s.IndexOf("CONNECTED", StringComparison.OrdinalIgnoreCase) >= 0;
                }
                catch
                {
                    return false;
                }
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
