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

        // --- EXG (NUOVO) ---
        public bool EnableExg1 { get; set; }   // EXG1 CH1/CH2
        public bool EnableExg2 { get; set; }   // EXG2 CH1/CH2
        public bool ExgUse16Bit { get; set; }  // false -> 24-bit (default)
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

        // --- helper pubblico: almeno un sensore attivo
        public static bool AnySensorEnabled(ShimmerConfig c) =>
            c.EnableLowNoiseAccelerometer || c.EnableWideRangeAccelerometer || c.EnableGyroscope ||
            c.EnableMagnetometer || c.EnablePressureTemperature || c.EnableBattery ||
            c.EnableExtA6 || c.EnableExtA7 || c.EnableExtA15 ||
            // --- EXG (NUOVO) ---
            c.EnableExg1 || c.EnableExg2;

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
            if (AnySensorEnabled(cfg)) sess.Start(); // parte solo se c'è almeno un sensore

            _macSessions[mac] = sess;

            // broadcast iniziale della config verso i subscriber (se già presenti)
            var initMsg = new { type = "config_changed", mac, cfg };
            await BroadcastToSubscribers(mac, JsonSerializer.Serialize(initMsg));

            Log?.Invoke($"[SERVER] Session started for {mac}");
        }

        // === Update live della configurazione sensori per un MAC attivo
        public async Task UpdateConfigAsync(string mac, ShimmerConfig cfg)
        {
            mac = (mac ?? "").Trim();
            if (mac.Length == 0) return;

            if (_macSessions.TryGetValue(mac, out var sess))
            {
                try { sess.Stop(); } catch { }
                await sess.ApplyConfigAsync(cfg);

                if (AnySensorEnabled(cfg))
                {
                    sess.Start(); // riavvia solo se ci sono sensori attivi
                }

                var msg = new { type = "config_changed", mac, cfg };
                await BroadcastToSubscribers(mac, JsonSerializer.Serialize(msg));

                Log?.Invoke($"[SERVER] Reconfigured {mac}");
            }
            else
            {
                Log?.Invoke($"[SERVER] UpdateConfig ignored: session not found for {mac}");
            }
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

        // dentro WsBridgeManager
        private Task SendConfigSnapshot(Guid clientId, string mac)
        {
            if (_macSessions.TryGetValue(mac, out var sess))
            {
                var cfg = sess.CurrentConfig;
                return SendJson(clientId, new { type = "config_changed", mac, cfg });
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

                // restituisce la config corrente per un MAC
                case "get_config":
                    {
                        string mac = root.TryGetProperty("mac", out var pm) ? (pm.GetString() ?? "").Trim() : "";
                        if (mac.Length == 0)
                        {
                            await SendJson(clientId, new { type = "config", ok = false, error = "no_mac" });
                            break;
                        }
                        if (TryGetConfig(mac, out var cfg))
                        {
                            await SendJson(clientId, new { type = "config", ok = true, mac, cfg });
                        }
                        else
                        {
                            await SendJson(clientId, new { type = "config", ok = false, mac, error = "not_active" });
                        }
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

                            // broadcast per chi è sottoscritto a quel MAC (invia anche la config con SR aggiornato)
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

                        if (_macSessions.ContainsKey(mac))
                        {
                            // 1) iscrizione
                            Subscribe(clientId, mac);

                            // 2) ACK immediato (await)
                            await SendJson(clientId, new { type = "open_ack", ok = true, mac, mode = "subscribed" });

                            // 3) snapshot config immediato (await)
                            await SendConfigSnapshot(clientId, mac);

                            // 4) retry "best effort" senza ContinueWith/SendAsync diretti
                            _ = Task.Run(async () =>
                            {
                                try
                                {
                                    await Task.Delay(250);
                                    await SendJson(clientId, new { type = "open_ack", ok = true, mac, mode = "subscribed" });
                                    await SendConfigSnapshot(clientId, mac);
                                }
                                catch (Exception ex)
                                {
                                    Log?.Invoke($"open retry send error: {ex.Message}");
                                }
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
                if (send)
                {
                    tasks.Add(SafeSend(clientId, json));
                }
            }

            try { await Task.WhenAll(tasks); }
            catch (Exception ex)
            {
                // non far esplodere il chiamante se 1 client fallisce
                Log?.Invoke($"Broadcast error: {ex.Message}");
            }
        }

        private Task SendJson(Guid id, object obj)
        {
            if (_ws == null) return Task.CompletedTask;
            try
            {
                string msg = JsonSerializer.Serialize(obj);
                return SafeSend(id, msg); // <— invece di _ws.SendAsync
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
                iA6 = -1, iA7 = -1, iA15 = -1,
                // --- EXG (NUOVO) ---
                iExg1Ch1 = -1, iExg1Ch2 = -1, iExg2Ch1 = -1, iExg2Ch2 = -1;

            // memorizza ultima config applicata
            ShimmerConfig _currentCfg = new ShimmerConfig();
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
                // --- EXG (NUOVO) ---
                EnableExg1 = _currentCfg.EnableExg1,
                EnableExg2 = _currentCfg.EnableExg2,
                ExgUse16Bit = _currentCfg.ExgUse16Bit
            };

            // === AGGIUNGERE in SppSession (ad es. subito dopo i campi iTs, iLnaX... ===
            void ResetIndices()
            {
                iTs = -1;
                iLnaX = iLnaY = iLnaZ = -1;
                iWraX = iWraY = iWraZ = -1;
                iGx = iGy = iGz = -1;
                iMx = iMy = iMz = -1;
                iTemp = iPress = iVbatt = -1;
                iA6 = iA7 = iA15 = -1;
                // --- EXG (NUOVO) ---
                iExg1Ch1 = iExg1Ch2 = iExg2Ch1 = iExg2Ch2 = -1;
            }

            void RefreshMissingIndices(ObjectCluster oc)
            {
                if (iTs == -1) iTs = SafeIdx(oc, ShimmerConfiguration.SignalNames.SYSTEM_TIMESTAMP, "CAL");

                if (_currentCfg.EnableLowNoiseAccelerometer)
                {
                    if (iLnaX == -1) iLnaX = SafeIdx(oc, Shimmer3Configuration.SignalNames.LOW_NOISE_ACCELEROMETER_X, "CAL");
                    if (iLnaY == -1) iLnaY = SafeIdx(oc, Shimmer3Configuration.SignalNames.LOW_NOISE_ACCELEROMETER_Y, "CAL");
                    if (iLnaZ == -1) iLnaZ = SafeIdx(oc, Shimmer3Configuration.SignalNames.LOW_NOISE_ACCELEROMETER_Z, "CAL");
                }
                if (_currentCfg.EnableWideRangeAccelerometer)
                {
                    if (iWraX == -1) iWraX = SafeIdx(oc, Shimmer3Configuration.SignalNames.WIDE_RANGE_ACCELEROMETER_X, "CAL");
                    if (iWraY == -1) iWraY = SafeIdx(oc, Shimmer3Configuration.SignalNames.WIDE_RANGE_ACCELEROMETER_Y, "CAL");
                    if (iWraZ == -1) iWraZ = SafeIdx(oc, Shimmer3Configuration.SignalNames.WIDE_RANGE_ACCELEROMETER_Z, "CAL");
                }
                if (_currentCfg.EnableGyroscope)
                {
                    if (iGx == -1) iGx = SafeIdx(oc, Shimmer3Configuration.SignalNames.GYROSCOPE_X, "CAL");
                    if (iGy == -1) iGy = SafeIdx(oc, Shimmer3Configuration.SignalNames.GYROSCOPE_Y, "CAL");
                    if (iGz == -1) iGz = SafeIdx(oc, Shimmer3Configuration.SignalNames.GYROSCOPE_Z, "CAL");
                }
                if (_currentCfg.EnableMagnetometer)
                {
                    if (iMx == -1) iMx = SafeIdx(oc, Shimmer3Configuration.SignalNames.MAGNETOMETER_X, "CAL");
                    if (iMy == -1) iMy = SafeIdx(oc, Shimmer3Configuration.SignalNames.MAGNETOMETER_Y, "CAL");
                    if (iMz == -1) iMz = SafeIdx(oc, Shimmer3Configuration.SignalNames.MAGNETOMETER_Z, "CAL");
                }
                if (_currentCfg.EnablePressureTemperature)
                {
                    if (iTemp == -1) iTemp = SafeIdx(oc, Shimmer3Configuration.SignalNames.TEMPERATURE, "CAL");
                    if (iPress == -1) iPress = SafeIdx(oc, Shimmer3Configuration.SignalNames.PRESSURE, "CAL");
                }
                if (_currentCfg.EnableBattery)
                {
                    if (iVbatt == -1) iVbatt = SafeIdx(oc, Shimmer3Configuration.SignalNames.V_SENSE_BATT, "CAL");
                }
                if (_currentCfg.EnableExtA6 && iA6 == -1) iA6 = SafeIdx(oc, Shimmer3Configuration.SignalNames.EXTERNAL_ADC_A6, "CAL");
                if (_currentCfg.EnableExtA7 && iA7 == -1) iA7 = SafeIdx(oc, Shimmer3Configuration.SignalNames.EXTERNAL_ADC_A7, "CAL");
                if (_currentCfg.EnableExtA15 && iA15 == -1) iA15 = SafeIdx(oc, Shimmer3Configuration.SignalNames.EXTERNAL_ADC_A15, "CAL");

                // --- EXG (NUOVO): prova più alias per compatibilità ---
                if (_currentCfg.EnableExg1)
                {
                    if (iExg1Ch1 == -1) iExg1Ch1 = TryIdx(oc,
                        ("EXG1 CH1", "CAL"), ("EXG1_CH1", "CAL"), ("EXG CH1", "CAL"),
                        ("ECG LA-RA", "CAL"), ("EMG CH1", "CAL"),
                        ("EXG1 CH1", "RAW"));
                    if (iExg1Ch2 == -1) iExg1Ch2 = TryIdx(oc,
                        ("EXG1 CH2", "CAL"), ("EXG1_CH2", "CAL"), ("EXG CH2", "CAL"),
                        ("ECG LL-LA", "CAL"), ("EMG CH2", "CAL"),
                        ("EXG1 CH2", "RAW"));
                }
                if (_currentCfg.EnableExg2)
                {
                    if (iExg2Ch1 == -1) iExg2Ch1 = TryIdx(oc,
                        ("EXG2 CH1", "CAL"), ("EXG2_CH1", "CAL"), ("EXG CH3", "CAL"),
                        ("EXG2 CH1", "RAW"));
                    if (iExg2Ch2 == -1) iExg2Ch2 = TryIdx(oc,
                        ("EXG2 CH2", "CAL"), ("EXG2_CH2", "CAL"), ("EXG CH4", "CAL"),
                        ("EXG2 CH2", "RAW"));
                }
            }


            public async Task<double> SetSamplingRateAsync(double newHz)
            {
                if (_core == null) throw new InvalidOperationException("Not open");
                bool wasStreaming = _handler != null; // se stiamo già streammando

                if (wasStreaming)
                {
                    try { Stop(); } catch { /* no-op */ }
                    await Task.Delay(100);   // era 50
                }

                int sr = (int)Math.Round(newHz);
                _core.WriteSamplingRate(sr);
                await Task.Delay(250);       // era 150

                // Reset mappa e base temporale per sessione "pulita"
                ResetIndices();
                _tsBase = null;


                // 4) Allinea la config corrente
                _currentCfg.SamplingRate = sr;

                // 5) Riavvia solo se ci sono sensori attivi
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
                if (_core == null) throw new InvalidOperationException("Not open");

                // --- AGGIUNTA: auto-abilita EXG1+EXG2 se la board è EXG e l'utente non li ha già abilitati ---
                try
                {
                    if (!cfg.EnableExg1 && !cfg.EnableExg2)
                    {
                        if (ShimmerScanManager.ShimmerBoardDetector.TryDetectBoardKind(
                                _core, out var kind, out var rawId) &&
                            kind == ShimmerScanManager.ShimmerBoardDetector.BoardKind.EXG)
                        {
                            cfg.EnableExg1 = true;
                            cfg.EnableExg2 = true;
                            cfg.ExgUse16Bit = false; // 24-bit default (come ShimmerApp)
                            _log($"[CFG] EXG board detected ({rawId}), enabling EXG1+EXG2 @24-bit");
                        }
                    }
                }
                catch { /* non bloccare la config se detection fallisce */ }

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

                    // --- EXG (NUOVO) ---
                    if (cfg.EnableExg1)
                        mask |= cfg.ExgUse16Bit
                            ? (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG1_16BIT
                            : (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG1_24BIT;
                    if (cfg.EnableExg2)
                        mask |= cfg.ExgUse16Bit
                            ? (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG2_16BIT
                            : (int)ShimmerBluetooth.SensorBitmapShimmer3.SENSOR_EXG2_24BIT;

                    return mask;
                }

                if (cfg.SamplingRate.HasValue && cfg.SamplingRate.Value > 0)
                {
                    int sr = (int)Math.Round(cfg.SamplingRate.Value);
                    _core.WriteSamplingRate(sr);
                    await Task.Delay(250);   // era 150
                }

                _core.WriteSensors(BuildMask());
                await Task.Delay(350);       // era 180

                ResetIndices();              // reset mappa completa
                _configured = true;

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
                    // --- EXG (NUOVO) ---
                    EnableExg1 = cfg.EnableExg1,
                    EnableExg2 = cfg.EnableExg2,
                    ExgUse16Bit = cfg.ExgUse16Bit
                };

                _log($"[CFG] applied");
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
                            double? tsRel = null;
                            if (tsAbs.HasValue)
                            {
                                if (!_tsBase.HasValue) _tsBase = tsAbs.Value;
                                tsRel = tsAbs.Value - _tsBase.Value; // in stesse unità di tsAbs
                            }

                            // estrazione valori (possono essere null se il sensore non è nel pacchetto)
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

                            // --- EXG (NUOVO) ---
                            double? exg1ch1 = Val(SafeGet(oc, iExg1Ch1));
                            double? exg1ch2 = Val(SafeGet(oc, iExg1Ch2));
                            double? exg2ch1 = Val(SafeGet(oc, iExg2Ch1));
                            double? exg2ch2 = Val(SafeGet(oc, iExg2Ch2));

                            // payload dinamico: includo SOLO i sensori abilitati nella config corrente
                            var map = new Dictionary<string, object?>(StringComparer.OrdinalIgnoreCase)
                            {
                                ["type"] = "sample",
                                ["mac"] = _mac,
                                ["ts"] = tsRel
                            };

                            // helper per costruire oggetti non-null
                            static object Vec3(double? x, double? y, double? z) =>
                                new { x = x ?? 0.0, y = y ?? 0.0, z = z ?? 0.0 };
                            static object Ext3(double? a6, double? a7, double? a15) =>
                                new { a6 = a6 ?? 0.0, a7 = a7 ?? 0.0, a15 = a15 ?? 0.0 };

                            if (_currentCfg.EnableLowNoiseAccelerometer)
                                map["lna"] = Vec3(lnaX, lnaY, lnaZ);
                            if (_currentCfg.EnableWideRangeAccelerometer)
                                map["wra"] = Vec3(wraX, wraY, wraZ);
                            if (_currentCfg.EnableGyroscope)
                                map["gyro"] = Vec3(gx, gy, gz);
                            if (_currentCfg.EnableMagnetometer)
                                map["mag"] = Vec3(mx, my, mz);
                            if (_currentCfg.EnablePressureTemperature)
                            {
                                map["temp"] = temp ?? 0.0;
                                map["press"] = press ?? 0.0;
                            }
                            if (_currentCfg.EnableBattery)
                                map["vbatt"] = vbatt ?? 0.0;
                            if (_currentCfg.EnableExtA6 || _currentCfg.EnableExtA7 || _currentCfg.EnableExtA15)
                                map["ext"] = Ext3(
                                    _currentCfg.EnableExtA6 ? a6 : 0.0,
                                    _currentCfg.EnableExtA7 ? a7 : 0.0,
                                    _currentCfg.EnableExtA15 ? a15 : 0.0
                                );

                            // --- EXG nel JSON: blocchi strutturati + compatibilità con VM MAUI (ExgCh1/ExgCh2) ---
                            // Pubblica solo se arrivano davvero i canali EXG
                            bool hasExg1 = exg1ch1.HasValue || exg1ch2.HasValue;
                            bool hasExg2 = exg2ch1.HasValue || exg2ch2.HasValue;

                            if (hasExg1)
                            {
                                map["exg1"] = new { ch1 = exg1ch1 ?? 0.0, ch2 = exg1ch2 ?? 0.0 };
                                map["ExgCh1"] = exg1ch1 ?? 0.0;  // compat con VM
                                map["ExgCh2"] = exg1ch2 ?? 0.0;
                            }
                            else if (!hasExg1 && hasExg2)
                            {
                                // se solo EXG2 dà dati, esponi anche i flat per compat
                                map["exg2"] = new { ch1 = exg2ch1 ?? 0.0, ch2 = exg2ch2 ?? 0.0 };
                                map["ExgCh1"] = exg2ch1 ?? 0.0;
                                map["ExgCh2"] = exg2ch2 ?? 0.0;
                            }
                            else if (hasExg2)
                            {
                                map["exg2"] = new { ch1 = exg2ch1 ?? 0.0, ch2 = exg2ch2 ?? 0.0 };
                            }
                            // (se nessuno ha valori, non aggiungiamo proprio le chiavi EXG)

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

            // --- helper EXG (NUOVO): prova più alias dei label ---
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

