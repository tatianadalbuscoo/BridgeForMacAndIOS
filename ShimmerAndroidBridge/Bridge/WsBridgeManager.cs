using System;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Net.WebSockets;
using Android.App;
using Android.Bluetooth;
using Android.Net.Wifi;
using WatsonWebsocket;
using XR2Learn_ShimmerAPI.IMU.Android;
using ShimmerAPI;

namespace Com.Example.ShimmerBridge
{
    // Sensors-only config (no sample rate)
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
    }

    public sealed class WsBridgeManager : IDisposable
    {
        public event Action<string>? Log;

        private WatsonWsServer? _ws;

        // WS clients (key = client Guid)
        private readonly ConcurrentDictionary<Guid, SppSession> _wsSessions = new();

        // UI sessions (key = MAC)
        private readonly ConcurrentDictionary<string, SppSession> _uiSessions =
            new(StringComparer.OrdinalIgnoreCase);

        public bool IsRunning => _ws?.IsListening ?? false;
        public int ActiveSessionCount => _wsSessions.Count + _uiSessions.Count;
        public bool HasActiveSessions => ActiveSessionCount > 0;

        // Start WS on the phone (optional for your UI flow)
        public Task StartAsync(Activity activity, int port = 8787)
        {
            if (IsRunning) return Task.CompletedTask;

            string ip = GetLocalIp(activity);
            _ws = new WatsonWsServer(ip, port, false);

            _ws.ClientConnected += (s, e) => Log?.Invoke($"WS client {e.Client.Guid} connected");
            _ws.ClientDisconnected += (s, e) =>
            {
                Log?.Invoke($"WS client {e.Client.Guid} disconnected");
                if (_wsSessions.TryRemove(e.Client.Guid, out var sess)) sess.Dispose();
            };
            _ws.MessageReceived += OnMessage;

            _ws.Start();
            Log?.Invoke($"WS on ws://{ip}:{port}/stream");
            return Task.CompletedTask;
        }

        public async Task StopAsync()
        {
            await CloseAllAsync();  // close all sessions (UI + WS)
            if (_ws != null) { try { _ws.Stop(); } catch { } _ws.Dispose(); _ws = null; }
            Log?.Invoke("WS stopped");
        }

        public void Dispose() => _ = StopAsync();

        // ====== UI path: open + config + start for one MAC ======
        public async Task OpenConfigureAndStartAsync(string mac, ShimmerConfig cfg)
        {
            // Replace existing session for this MAC
            if (_uiSessions.TryGetValue(mac, out var old))
            {
                try { old.Stop(); old.Dispose(); } catch { }
                _uiSessions.TryRemove(mac, out _);
            }

            var sess = new SppSession(mac, _ => Task.CompletedTask, msg => Log?.Invoke(msg));
            sess.Open();
            await sess.ApplyConfigAsync(cfg);
            sess.Start();

            _uiSessions[mac] = sess;
            Log?.Invoke($"UI session started for {mac}");
        }

        // ====== Stop streaming on ALL sessions (UI + WS), but keep connections ======
        public Task StopAllStreamingAsync()
        {
            foreach (var s in _wsSessions.Values) { try { s.Stop(); } catch { } }
            foreach (var s in _uiSessions.Values) { try { s.Stop(); } catch { } }
            Log?.Invoke("All streams stopped");
            return Task.CompletedTask;
        }

        // ====== Close & clear ALL sessions ======
        public Task CloseAllAsync()
        {
            foreach (var s in _wsSessions.Values) { try { s.Dispose(); } catch { } }
            _wsSessions.Clear();

            foreach (var s in _uiSessions.Values) { try { s.Dispose(); } catch { } }
            _uiSessions.Clear();

            Log?.Invoke("All sessions closed");
            return Task.CompletedTask;
        }

        // ====== WebSocket handler (optional protocol) ======
        private async void OnMessage(object? sender, MessageReceivedEventArgs e)
        {
            try
            {
                if (e.MessageType == WebSocketMessageType.Text)
                {
                    var txt = GetString(e.Data);
                    await HandleTextAsync(e.Client.Guid, txt);
                }
                else if (e.MessageType == WebSocketMessageType.Binary)
                {
                    if (_wsSessions.TryGetValue(e.Client.Guid, out var sess))
                    {
                        var bytes = ToArray(e.Data);
                        await sess.WriteToSppAsync(bytes);
                    }
                }
            }
            catch (Exception ex) { Log?.Invoke("OnMessage error: " + ex.Message); }
        }

        private async Task HandleTextAsync(Guid clientId, string json)
        {
            using var doc = JsonDocument.Parse(json);
            var type = doc.RootElement.TryGetProperty("type", out var t) ? t.GetString() : null;

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
                        var mac = doc.RootElement.GetProperty("mac").GetString() ?? "";
                        if (_wsSessions.TryGetValue(clientId, out var old)) { old.Dispose(); _wsSessions.TryRemove(clientId, out _); }
                        var sess = new SppSession(mac, bytes => _ws!.SendAsync(clientId, bytes), msg => Log?.Invoke(msg));
                        try { sess.Open(); _wsSessions[clientId] = sess; await SendJson(clientId, new { type = "open_ack", ok = true, mac }); }
                        catch (Exception ex) { await SendJson(clientId, new { type = "open_ack", ok = false, error = ex.Message }); }
                        break;
                    }

                case "set_config":
                    {
                        if (!_wsSessions.TryGetValue(clientId, out var s)) { await SendJson(clientId, new { type = "config_ack", ok = false, error = "no_session" }); break; }
                        var cfg = JsonSerializer.Deserialize<ShimmerConfig>(json);
                        if (cfg == null) { await SendJson(clientId, new { type = "config_ack", ok = false, error = "bad_config" }); break; }
                        await s.ApplyConfigAsync(cfg);
                        await SendJson(clientId, new { type = "config_ack", ok = true });
                        break;
                    }

                case "start":
                    if (_wsSessions.TryGetValue(clientId, out var s1)) { s1.Start(); await SendJson(clientId, new { type = "start_ack", ok = true }); }
                    else { await SendJson(clientId, new { type = "error", error = "no_session" }); }
                    break;

                case "stop":
                    if (_wsSessions.TryGetValue(clientId, out var s2)) { s2.Stop(); await SendJson(clientId, new { type = "stop_ack", ok = true }); }
                    else { await SendJson(clientId, new { type = "error", error = "no_session" }); }
                    break;

                case "close":
                    if (_wsSessions.TryRemove(clientId, out var s3)) s3.Dispose();
                    await SendJson(clientId, new { type = "close_ack", ok = true });
                    break;

                default:
                    await SendJson(clientId, new { type = "error", error = "unknown_type" });
                    break;
            }
        }

        private Task SendJson(Guid id, object obj)
        {
            var data = Encoding.UTF8.GetBytes(JsonSerializer.Serialize(obj));
            return _ws!.SendAsync(id, data);
        }

        // ====== SPP + ShimmerAPI session ======
        private sealed class SppSession : IDisposable
        {
            readonly AndroidBluetoothConnection _conn;
            readonly Func<byte[], Task> _sendBinaryToWs;
            readonly Action<string> _log;

            ShimmerBluetoothTransport? _transport;
            CancellationTokenSource? _cts;
            Task? _rx;

            public SppSession(string mac, Func<byte[], Task> sendBinaryToWs, Action<string> log)
            {
                _conn = new AndroidBluetoothConnection(mac);
                _sendBinaryToWs = sendBinaryToWs;
                _log = log;
            }

            public void Open()
            {
                _conn.Open(); // RFCOMM SPP
                _transport = new ShimmerBluetoothTransport("Shimmer3", _conn);
            }

            // Sensors only; keep factory sampling rate (~51.2 Hz)
            public async Task ApplyConfigAsync(ShimmerConfig cfg)
            {
                if (_transport == null) throw new InvalidOperationException("Not open");

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

                _transport.WriteSensors(mask);
                await Task.Delay(50);
            }

            public void Start()
            {
                if (_transport == null) throw new InvalidOperationException("Not open");
                _transport.StartStreaming();

                _cts = new CancellationTokenSource();
                var token = _cts.Token;
                _rx = Task.Run(async () =>
                {
                    var buf = new byte[2048];
                    int n = 0;
                    try
                    {
                        while (!token.IsCancellationRequested)
                        {
                            int b = _conn.ReadByte(); // blocking
                            buf[n++] = (byte)b;
                            if (n >= buf.Length)
                            { await _sendBinaryToWs(buf.ToArray()); n = 0; }
                        }
                    }
                    catch { /* stop */ }
                    if (n > 0) await _sendBinaryToWs(buf.Take(n).ToArray());
                }, token);
            }

            public void Stop()
            {
                try { _transport?.StopStreaming(); } catch { }
                try { _cts?.Cancel(); } catch { }
            }

            public Task WriteToSppAsync(byte[] data)
            { _conn.WriteBytes(data, 0, data.Length); return Task.CompletedTask; }

            public void Dispose()
            {
                try { _cts?.Cancel(); } catch { }
                try { _rx?.Wait(200); } catch { }
                try { _transport?.StopStreaming(); } catch { }
                try { _conn.Close(); } catch { }
            }
        }

        // helpers
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

        static byte[] ToArray(ArraySegment<byte> seg)
        {
            var arr = new byte[seg.Count];
            Buffer.BlockCopy(seg.Array!, seg.Offset, arr, 0, seg.Count);
            return arr;
        }
    }
}
