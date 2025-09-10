using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Reflection;
using Android.App;
using Android.Bluetooth;
using Android.Content;
using XR2Learn_ShimmerAPI.IMU.Android; // ShimmerLogAndStreamAndroidBluetoothV2

namespace Com.Example.ShimmerBridge
{
    /// <summary>
    /// Scan manager (Bluetooth Classic) per Shimmer3.
    /// - Scansiona via Classic BT
    /// - Classifica EXG/IMU leggendo l’Expansion Board con micro-connessione SPP (reflection)
    /// - DeviceOff = accoppiati non visti nello scan corrente
    /// </summary>
    public sealed class ShimmerScanManager
    {
        public enum DeviceType { Unknown = 0, IMU = 1, EXG = 2, DeviceOff = 3 }

        public sealed class Entry
        {
            public string Name { get; set; } = "";
            public string Mac { get; set; } = "";
            public int? Rssi { get; set; }
            public bool IsPaired { get; set; }
            public DeviceType Type { get; set; } = DeviceType.Unknown;
        }

        public sealed class Result
        {
            public List<Entry> Visible { get; } = new();
            public List<Entry> Off { get; } = new();
        }

        // Cache classificazione (MAC -> Tipo) solo quando detection reale ha successo
        private static readonly ConcurrentDictionary<string, DeviceType> _typeCache =
            new(StringComparer.OrdinalIgnoreCase);

        readonly Activity _activity;
        readonly BluetoothAdapter? _adapter;

        public ShimmerScanManager(Activity activity)
        {
            _activity = activity;
            _adapter = BluetoothAdapter.DefaultAdapter;
        }

        /// <summary>
        /// Avvia discovery BT classico per la durata indicata, poi classifica EXG/IMU con micro-connessioni SPP.
        /// </summary>
        public async Task<Result> ScanAsync(TimeSpan duration, CancellationToken ct = default)
        {
            var result = new Result();

            if (_adapter == null || !_adapter.IsEnabled)
                return result;

            // Paired Shimmer (per calcolare DeviceOff)
            var bonded = (_adapter.BondedDevices ?? new HashSet<BluetoothDevice>())
                .Where(d => LooksLikeShimmer(d?.Name, d?.Address))
                .ToDictionary(d => d.Address, d => d);

            var discovered = new ConcurrentDictionary<string, Entry>(StringComparer.OrdinalIgnoreCase);
            var tcsFinished = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);

            using var filter = new IntentFilter();
            filter.AddAction(BluetoothDevice.ActionFound);
            filter.AddAction(BluetoothAdapter.ActionDiscoveryFinished);

            var receiver = new DiscoveryReceiver(
                onFound: (dev, rssi) =>
                {
                    if (dev?.Name == null) return;
                    if (!LooksLikeShimmer(dev.Name, dev.Address)) return;

                    // prova cache: se abbiamo già classificato questo MAC in passato, riusa
                    var cached = _typeCache.TryGetValue(dev.Address, out var t) ? t : DeviceType.Unknown;

                    discovered[dev.Address] = new Entry
                    {
                        Name = dev.Name,
                        Mac = dev.Address,
                        Rssi = rssi,
                        IsPaired = bonded.ContainsKey(dev.Address),
                        Type = cached
                    };
                },
                onFinished: () => tcsFinished.TrySetResult()
            );

            _activity.RegisterReceiver(receiver, filter);

            try
            {
                if (_adapter.IsDiscovering) _adapter.CancelDiscovery();
                _adapter.StartDiscovery();

                using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                cts.CancelAfter(duration);
                await Task.WhenAny(tcsFinished.Task, Task.Delay(duration, cts.Token));
            }
            finally
            {
                try { if (_adapter.IsDiscovering) _adapter.CancelDiscovery(); } catch { }
                try { _activity.UnregisterReceiver(receiver); } catch { }
            }

            // Dispositivi visibili
            var visibles = discovered.Values.OrderBy(x => x.Name).ToList();

            // === Classificazione reale EXG/IMU con micro-connessione SPP (sequenziale per stabilità RN-42) ===
            foreach (var e in visibles)
            {
                if (e.Type != DeviceType.Unknown) continue; // già in cache

                try
                {
                    var t = await ShimmerBoardDetector.GetExpansionBoardKindAndroidAsync(e.Name, e.Mac);
                    // t.Item1 = ok, t.Item2 = kind, t.Item3 = rawId
                    if (t.Item1)
                    {
                        var mapped = (t.Item2 == ShimmerBoardDetector.BoardKind.EXG)
                            ? DeviceType.EXG
                            : DeviceType.IMU;

                        e.Type = mapped;
                        _typeCache[e.Mac] = mapped; // memorizza solo se detection ok
                    }
                    // se !ok resta Unknown (non forziamo IMU)
                }
                catch
                {
                    // lascia Unknown; non cache-iamo i fallimenti
                }
            }

            // Popola risultato
            result.Visible.AddRange(visibles);

            // DeviceOff = paired non visti nello scan
            foreach (var kv in bonded)
            {
                if (discovered.ContainsKey(kv.Key)) continue;
                result.Off.Add(new Entry
                {
                    Name = kv.Value.Name ?? $"Shimmer3-{kv.Key}",
                    Mac = kv.Key,
                    IsPaired = true,
                    Type = DeviceType.DeviceOff
                });
            }

            return result;
        }

        public static bool LooksLikeShimmer(string? name, string? mac)
        {
            string n = (name ?? "").ToUpperInvariant();
            string m = (mac ?? "").ToUpperInvariant();
            if (n.Contains("SHIMMER") || n.StartsWith("SHIMMER3")) return true;
            if (n.StartsWith("RNBT") || n.StartsWith("RN42") || n.StartsWith("RN-42")) return true;
            if (m.StartsWith("00:06:66")) return true; // Roving RN-42 vendor
            return false;
        }

        // ==== Receiver discovery ====
        sealed class DiscoveryReceiver : BroadcastReceiver
        {
            readonly Action<BluetoothDevice?, int?> _onFound;
            readonly Action _onFinished;

            public DiscoveryReceiver(Action<BluetoothDevice?, int?> onFound, Action onFinished)
            {
                _onFound = onFound;
                _onFinished = onFinished;
            }

            public override void OnReceive(Context context, Intent intent)
            {
                var action = intent.Action;
                if (action == BluetoothDevice.ActionFound)
                {
                    var dev = (BluetoothDevice?)intent.GetParcelableExtra(BluetoothDevice.ExtraDevice);
                    int? rssi = null;
                    if (intent.HasExtra(BluetoothDevice.ExtraRssi))
                        rssi = intent.GetShortExtra(BluetoothDevice.ExtraRssi, short.MinValue);
                    _onFound(dev, rssi);
                }
                else if (action == BluetoothAdapter.ActionDiscoveryFinished)
                {
                    _onFinished();
                }
            }
        }

        // ==========================================================
        //  LOGICA INCORPORATA: detection Expansion Board (Android)
        // ==========================================================
        internal static class ShimmerBoardDetector
        {
            public enum BoardKind { Unknown = 0, IMU = 1, EXG = 2 }

            /// <summary>
            /// Connette → detect → disconnette. Restituisce (ok, kind, rawString).
            /// </summary>
            public static async Task<Tuple<bool, BoardKind, string>> GetExpansionBoardKindAndroidAsync(
                string deviceName, string mac)
            {
                ShimmerLogAndStreamAndroidBluetoothV2? shim = null;
                try
                {
                    if (!BluetoothAdapter.CheckBluetoothAddress(mac))
                        return Tuple.Create(false, BoardKind.Unknown, "Invalid MAC");

                    shim = new ShimmerLogAndStreamAndroidBluetoothV2(deviceName, mac);
                    shim.Connect();

                    // attendo connessione (max ~6s)
                    var t0 = DateTime.UtcNow;
                    while (!shim.IsConnected() && (DateTime.UtcNow - t0).TotalMilliseconds < 6000)
                        await Task.Delay(50);

                    if (!shim.IsConnected())
                        return Tuple.Create(false, BoardKind.Unknown, "Connect timeout");

                    // detection reale via reflection "tollerante"
                    if (TryDetectBoardKind(shim, out var kind, out var raw))
                        return Tuple.Create(true, kind, raw);

                    return Tuple.Create(false, BoardKind.Unknown, "Detection failed");
                }
                catch (Exception ex)
                {
                    return Tuple.Create(false, BoardKind.Unknown, ex.Message);
                }
                finally
                {
                    try { shim?.Disconnect(); } catch { }
                }
            }

            /// <summary>
            /// Rileva la board:
            /// - trova un oggetto (shim o child) che espone GetExpansionBoard()
            /// - invia ReadInternalExpPower/ReadExpansionBoard
            /// - polla GetExpansionBoard() (retry a metà timeout)
            /// </summary>
            public static bool TryDetectBoardKind(
                ShimmerLogAndStreamAndroidBluetoothV2 shim,
                out BoardKind kind,
                out string rawId)
            {
                kind = BoardKind.Unknown;
                rawId = "";

                try
                {
                    if (shim == null || !shim.IsConnected())
                        return false;

                    // 1) trova il "target" che ha GetExpansionBoard()
                    var target = FindExpansionTarget(shim, maxDepth: 3);
                    if (target == null)
                        return false;

                    // 2) power + richiesta lettura (se i metodi esistono)
                    InvokeNoArgIfExists(target, "ReadInternalExpPower");     // alcune build richiedono power-on
                    InvokeNoArgIfExists(target, "WriteInternalExpPower");    // fallback alternativo
                    InvokeNoArgIfExists(target, "ReadExpansionBoard");
                    SafeDelay(120);

                    // 3) attende la risposta su GetExpansionBoard(), retry a metà timeout
                    string boardStr;
                    var ok = TryWaitExpansionString(target, out boardStr, timeoutMs: 2600);
                    if (!ok)
                    {
                        // un secondo tentativo
                        InvokeNoArgIfExists(target, "ReadExpansionBoard");
                        ok = TryWaitExpansionString(target, out boardStr, timeoutMs: 1400);
                    }

                    if (!ok || string.IsNullOrWhiteSpace(boardStr))
                    {
                        kind = BoardKind.Unknown;
                        rawId = "";
                        return false;
                    }

                    rawId = boardStr;
                    kind = MapBoardStringToKind(boardStr);
                    return kind != BoardKind.Unknown;
                }
                catch
                {
                    kind = BoardKind.Unknown;
                    rawId = "";
                    return false;
                }
            }

            // ===== helper =====

            private static BoardKind MapBoardStringToKind(string? boardStr)
            {
                if (string.IsNullOrWhiteSpace(boardStr))
                    return BoardKind.Unknown;

                // Se la stringa contiene "EXG" → EXG, altrimenti IMU (come nel tuo scanner)
                return boardStr.IndexOf("EXG", StringComparison.OrdinalIgnoreCase) >= 0
                    ? BoardKind.EXG
                    : BoardKind.IMU;
            }

            // BFS su campi/proprietà per trovare il primo oggetto che espone GetExpansionBoard()
            static object? FindExpansionTarget(object root, int maxDepth)
            {
                if (root == null || maxDepth < 0) return null;

                var visited = new HashSet<object>(new RefEqComparer());
                var q = new Queue<(object obj, int depth)>();

                EnqueueIfNew(root, 0);

                while (q.Count > 0)
                {
                    var (obj, depth) = q.Dequeue();
                    if (obj == null) continue;

                    if (HasMethod(obj, "GetExpansionBoard"))
                        return obj;

                    if (depth >= maxDepth) continue;

                    var t = obj.GetType();
                    var flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

                    foreach (var f in t.GetFields(flags))
                    {
                        object? val = null;
                        try { val = f.GetValue(obj); } catch { }
                        if (val == null) continue;
                        EnqueueIfNew(val, depth + 1);
                    }

                    foreach (var p in t.GetProperties(flags))
                    {
                        if (p.GetIndexParameters().Length != 0) continue;
                        object? val = null;
                        try { val = p.GetValue(obj); } catch { }
                        if (val == null) continue;
                        EnqueueIfNew(val, depth + 1);
                    }
                }

                return null;

                void EnqueueIfNew(object o, int d)
                {
                    if (o == null) return;
                    var tt = o.GetType();
                    if (tt.IsPrimitive || tt.IsEnum || tt == typeof(string)) return;
                    if (visited.Add(o)) q.Enqueue((o, d));
                }
            }

            private static bool HasMethod(object instance, string methodName)
            {
                var t = instance.GetType();
                var m = t.GetMethod(methodName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                return m != null && m.GetParameters().Length == 0;
            }

            /// Polla GetExpansionBoard() finché non ritorna una stringa non vuota; fa retry di ReadExpansionBoard a metà timeout.
            private static bool TryWaitExpansionString(object target, out string boardStr, int timeoutMs)
            {
                boardStr = GetStringNoArgIfExists(target, "GetExpansionBoard") ?? "";
                if (!string.IsNullOrWhiteSpace(boardStr)) return true;

                var waited = 0;
                const int step = 100;
                var retried = false;

                while (waited < timeoutMs)
                {
                    SafeDelay(step);
                    waited += step;

                    boardStr = GetStringNoArgIfExists(target, "GetExpansionBoard") ?? "";
                    if (!string.IsNullOrWhiteSpace(boardStr)) return true;

                    if (!retried && waited >= timeoutMs / 2)
                    {
                        retried = true;
                        InvokeNoArgIfExists(target, "ReadExpansionBoard");
                    }
                }

                boardStr = GetStringNoArgIfExists(target, "GetExpansionBoard") ?? "";
                return !string.IsNullOrWhiteSpace(boardStr);
            }

            private static string? GetStringNoArgIfExists(object instance, string methodName)
            {
                var t = instance.GetType();
                var m = t.GetMethod(methodName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                if (m != null && m.GetParameters().Length == 0)
                {
                    try
                    {
                        var res = m.Invoke(instance, null);
                        return res as string ?? res?.ToString();
                    }
                    catch { }
                }

                // fallback: proprietà omonima (alcune build esportano una property)
                foreach (var pName in new[] { "ExpansionBoard", "ExpansionBoardID", "DaughterCardID" })
                {
                    var p = t.GetProperty(pName, BindingFlags.Public | BindingFlags.Instance);
                    if (p != null)
                    {
                        try
                        {
                            var val = p.GetValue(instance);
                            var s = val as string ?? val?.ToString();
                            if (!string.IsNullOrWhiteSpace(s)) return s;
                        }
                        catch { }
                    }
                }
                return null;
            }

            private static void SafeDelay(int ms)
            {
                try { System.Threading.Thread.Sleep(ms); } catch { }
            }

            /// Invoca via reflection un metodo senza argomenti se esiste (public o non-public).
            private static object? InvokeNoArgIfExists(object instance, string methodName)
            {
                var t = instance.GetType();
                var m = t.GetMethod(methodName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                if (m != null && m.GetParameters().Length == 0)
                {
                    try { return m.Invoke(instance, null); } catch { }
                }
                return null;
            }

            // Comparer reference-based per "visited" nella BFS
            private sealed class RefEqComparer : IEqualityComparer<object>
            {
                public new bool Equals(object? x, object? y) => ReferenceEquals(x, y);
                public int GetHashCode(object obj) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(obj);
            }
        }




    }
}

