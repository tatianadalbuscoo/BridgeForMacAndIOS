/*
 * Android-specific manager that scans for Shimmer devices via Bluetooth Classic.
 * Performs discovery, identifies IMU vs EXG boards through a short SPP handshake,
 * and reports both visible and paired-but-offline devices.
 */


using System.Collections.Concurrent;
using System.Reflection;
using Android.Bluetooth;
using Android.Content;
using ShimmerSDK.Android;


namespace ShimmerBridgeScan
{

    /// <summary>
    /// Handles Bluetooth scanning and classification of Shimmer devices (IMU vs EXG).
    /// </summary>
    public sealed class ShimmerScanManager
    {

        readonly Activity _activity;
        readonly BluetoothAdapter? _adapter;


        // Types of devices that can be discovered
        public enum DeviceType { Unknown = 0, IMU = 1, EXG = 2, DeviceOff = 3 }


        // Cache: remember classification results for MAC
        private static readonly ConcurrentDictionary<string, DeviceType> _typeCache =
            new(StringComparer.OrdinalIgnoreCase);


        /// <summary>
        /// Represents a single device entry with metadata (name, MAC, RSSI, etc.).
        /// </summary>
        public sealed class Entry
        {
            public string Name { get; set; } = "";
            public string Mac { get; set; } = "";
            public int? Rssi { get; set; }
            public bool IsPaired { get; set; }
            public DeviceType Type { get; set; } = DeviceType.Unknown;
        }


        /// <summary>
        /// Container for scan results: visible devices and paired-but-offline devices.
        /// </summary>
        public sealed class Result
        {
            public List<Entry> Visible { get; } = new();
            public List<Entry> Off { get; } = new();
        }


        /// <summary>
        /// Initializes a new instance of the ShimmerScanManager class.
        /// </summary>
        /// <param name="activity">
        /// The Android <see cref="Activity"/> that owns the Bluetooth scan.  
        /// It is required to register and unregister the broadcast receiver used during discovery.
        /// </param>
        public ShimmerScanManager(Activity activity)
        {
            _activity = activity;
            _adapter = BluetoothAdapter.DefaultAdapter;
        }


        /// <summary>
        /// Runs a Bluetooth Classic discovery for the given <paramref name="duration"/>,
        /// collects Shimmer-like devices, and (sequentially) classifies them as IMU/EXG
        /// via a short SPP handshake. Also returns paired-but-not-seen devices as <c>DeviceOff</c>.
        /// </summary>
        /// <param name="duration">Max time to keep discovery active.</param>
        /// <param name="ct">Optional cancellation token to abort early.</param>
        /// <returns>A <see cref="Result"/> with <c>Visible</c> and <c>Off</c> device lists.</returns>
        public async Task<Result> ScanAsync(TimeSpan duration, CancellationToken ct = default)
        {
            var result = new Result();

            // Early exit if BT is unavailable or disabled
            if (_adapter == null || !_adapter.IsEnabled)
                return result;

            // Snapshot of paired Shimmer-like devices (used to compute DeviceOff later)
            var bonded = (_adapter.BondedDevices ?? new HashSet<BluetoothDevice>())
                .Where(d => d != null && d.Address != null && LooksLikeShimmer(d.Name, d.Address))
                .ToDictionary(d => d.Address!, d => d);

            var discovered = new ConcurrentDictionary<string, Entry>(StringComparer.OrdinalIgnoreCase);
            var tcsFinished = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);

            // Listen for "device found" and "discovery finished" broadcasts
            using var filter = new IntentFilter();
            filter.AddAction(BluetoothDevice.ActionFound);
            filter.AddAction(BluetoothAdapter.ActionDiscoveryFinished);

            var receiver = new DiscoveryReceiver(
                onFound: (dev, rssi) =>
                {
                    if (dev?.Name == null) return;

                    var addr = dev.Address;
                    if (string.IsNullOrEmpty(addr)) return;

                    if (!LooksLikeShimmer(dev.Name, addr)) return;

                    // Reuse cached classification if we have it for this MAC
                    var cached = (_typeCache.TryGetValue(addr, out var t)) ? t : DeviceType.Unknown;

                    discovered[addr] = new Entry
                    {
                        Name = dev.Name,
                        Mac = addr,
                        Rssi = rssi,
                        IsPaired = bonded.ContainsKey(addr),
                        Type = cached
                    };
                },
                onFinished: () => tcsFinished.TrySetResult()
            );

            _activity.RegisterReceiver(receiver, filter);

            try
            {

                // Start discovery
                if (_adapter.IsDiscovering) _adapter.CancelDiscovery();
                _adapter.StartDiscovery();

                // Bound discovery by duration and honor external cancellation
                using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                cts.CancelAfter(duration);
                await Task.WhenAny(tcsFinished.Task, Task.Delay(duration, cts.Token));
            }
            finally
            {

                // Stop discovery and unregister receiver
                try { if (_adapter.IsDiscovering) _adapter.CancelDiscovery(); } catch { }
                try { _activity.UnregisterReceiver(receiver); } catch { }
            }

            // Consolidate visible devices
            var visibles = discovered.Values.OrderBy(x => x.Name).ToList();

            // Classify IMU vs EXG via short SPP handshake
            foreach (var e in visibles)
            {
                if (e.Type != DeviceType.Unknown) continue;  // already from cache

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
                        _typeCache[e.Mac] = mapped; // cache only on success
                    }
                }
                catch
                {
                    // Keep Unknown; do not cache failures
                }
            }

            // Build result: visible devices and paired devices not seen during this scan → mark as DeviceOff
            result.Visible.AddRange(visibles);
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


        /// <summary>
        /// Checks if a Bluetooth device looks like a Shimmer board.
        /// </summary>
        /// <param name="name">Device name.</param>
        /// <param name="mac">Device MAC address.</param>
        /// <returns>True if it matches known Shimmer/RN-42 patterns; otherwise false.</returns>
        public static bool LooksLikeShimmer(string? name, string? mac)
        {

            // Normalize to compare safely (null-safe + case-insensitive)
            string n = (name ?? "").ToUpperInvariant();
            string m = (mac ?? "").ToUpperInvariant();

            // Shimmer-branded device names
            if (n.Contains("SHIMMER") || n.StartsWith("SHIMMER3")) return true;

            // Default RN-42 module names (often used by Shimmer)
            if (n.StartsWith("RNBT") || n.StartsWith("RN42") || n.StartsWith("RN-42")) return true;

            // Roving Networks RN-42 OUI prefix
            if (m.StartsWith("00:06:66")) return true;

            return false;
        }


        /// <summary>
        /// BroadcastReceiver that handles Bluetooth discovery events:
        /// device found and discovery finished.
        /// </summary>
        sealed class DiscoveryReceiver : BroadcastReceiver
        {

            readonly Action<BluetoothDevice?, int?> _onFound;    // callback when a device is discovered
            readonly Action _onFinished;                         // callback when discovery ends


            /// <summary>
            /// Creates a receiver that forwards discovery events to the provided callbacks.
            /// </summary>
            /// <param name="onFound">Callback invoked when a device is discovered; receives the device and its RSSI (if available).</param>
            /// <param name="onFinished">Callback invoked when Bluetooth discovery ends.</param>
            public DiscoveryReceiver(Action<BluetoothDevice?, int?> onFound, Action onFinished)
            {
                _onFound = onFound;
                _onFinished = onFinished;
            }


            /// <summary>
            /// Handles Bluetooth discovery broadcasts and forwards them to callbacks.
            /// Listens for <see cref="BluetoothDevice.ActionFound"/> and
            /// <see cref="BluetoothAdapter.ActionDiscoveryFinished"/>.
            /// </summary>
            /// <param name="context">Android context (may be null per platform annotations).</param>
            /// <param name="intent">
            /// Broadcast intent: either <c>BluetoothDevice.ActionFound</c> or
            /// <c>BluetoothAdapter.ActionDiscoveryFinished</c> (may be null).
            /// </param>
            public override void OnReceive(Android.Content.Context? context, Android.Content.Intent? intent)
            {
                if (intent is null) return;

                var action = intent.Action;
                if (action == BluetoothDevice.ActionFound)
                {
                    // Device discovered: extract BluetoothDevice and optional RSSI
                    var dev = (BluetoothDevice?)intent.GetParcelableExtra(BluetoothDevice.ExtraDevice);

                    int? rssi = null;
                    if (intent.HasExtra(BluetoothDevice.ExtraRssi))
                        rssi = intent.GetShortExtra(BluetoothDevice.ExtraRssi, short.MinValue);

                    _onFound(dev, rssi); // notify caller
                }
                else if (action == BluetoothAdapter.ActionDiscoveryFinished)
                {
                    _onFinished(); // notify caller that discovery ended
                }
            }

        }


        /// <summary>
        /// Detects the Shimmer expansion board kind on Android (IMU vs EXG).
        /// Opens a short SPP connection, queries expansion-board info via tolerant
        /// reflection (power/read if available), then disconnects. Provides helpers
        /// to locate the target object, poll for the board string, and map it to a kind.
        /// </summary>
        internal static class ShimmerBoardDetector
        {

            // Identifies the type of Shimmer device board
            public enum BoardKind { Unknown = 0, IMU = 1, EXG = 2 }


            /// <summary>
            /// Connects to a Shimmer device over SPP, detects the expansion-board kind (IMU/EXG),
            /// then disconnects.
            /// </summary>
            /// <param name="deviceName">Bluetooth device name (used by Shimmer SDK).</param>
            /// <param name="mac">Bluetooth MAC address.</param>
            /// <returns>
            /// Tuple <c>(ok, kind, rawString)</c>:
            /// - <c>ok</c>: true if detection succeeded;
            /// - <c>kind</c>: IMU/EXG/Unknown;
            /// - <c>rawString</c>: raw board identifier returned by the device (when available).
            /// </returns>
            public static async Task<Tuple<bool, BoardKind, string>> GetExpansionBoardKindAndroidAsync(
                string deviceName, string mac)
            {
                ShimmerLogAndStreamAndroidBluetoothV2? shim = null;
                try
                {

                    // Basic input guard: invalid MAC → fail fast
                    if (!BluetoothAdapter.CheckBluetoothAddress(mac))
                        return Tuple.Create(false, BoardKind.Unknown, "Invalid MAC");

                    // Create and connect the Shimmer session
                    shim = new ShimmerLogAndStreamAndroidBluetoothV2(deviceName, mac);
                    shim.Connect();

                    // Wait for connection (6s max)
                    var t0 = DateTime.UtcNow;
                    while (!shim.IsConnected() && (DateTime.UtcNow - t0).TotalMilliseconds < 6000)
                        await Task.Delay(50);

                    if (!shim.IsConnected())
                        return Tuple.Create(false, BoardKind.Unknown, "Connect timeout");

                    // Try to detect board kind (reflection-based)
                    if (TryDetectBoardKind(shim, out var kind, out var raw))
                        return Tuple.Create(true, kind, raw);

                    // Connected but no usable response
                    return Tuple.Create(false, BoardKind.Unknown, "Detection failed");
                }
                catch (Exception ex)
                {

                    // Bubble up error message in the tuple
                    return Tuple.Create(false, BoardKind.Unknown, ex.Message);
                }
                finally
                {
                    try { shim?.Disconnect(); } catch { }
                }
            }


            /// <summary>
            /// Detects the installed expansion board (IMU/EXG) on Android using reflection:
            /// powers/requests the board, then polls for GetExpansionBoard() and maps the result.
            /// </summary>
            /// <param name="shim">Connected Shimmer Bluetooth V2 session.</param>
            /// <param name="kind">Out: detected kind (Unknown/IMU/EXG).</param>
            /// <param name="rawId">Out: raw board identifier string.</param>
            /// <returns>True if a non-empty board string was read and mapped; otherwise false.</returns>
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

                    // Locate, via reflection, a target object exposing GetExpansionBoard/ReadExpansionBoard.
                    var target = FindExpansionTarget(shim, maxDepth: 3);
                    if (target == null)
                        return false;

                    // Issue the read commands (if present).
                    InvokeNoArgIfExists(target, "ReadInternalExpPower");     
                    InvokeNoArgIfExists(target, "WriteInternalExpPower");
                    InvokeNoArgIfExists(target, "ReadExpansionBoard");
                    SafeDelay(120);

                    // Poll for a non-empty board string.
                    string boardStr;
                    var ok = TryWaitExpansionString(target, out boardStr, timeoutMs: 2600);

                    // Retry once if empty.
                    if (!ok)
                    {
                        InvokeNoArgIfExists(target, "ReadExpansionBoard");
                        ok = TryWaitExpansionString(target, out boardStr, timeoutMs: 1400);
                    }

                    // Map the result.
                    if (!ok || string.IsNullOrWhiteSpace(boardStr))
                    {
                        kind = BoardKind.Unknown;  // signal that detection failed
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


            // ----- Helper -----


            /// <summary>
            /// Maps the raw expansion-board string to a <see cref="BoardKind"/>.
            /// </summary>
            /// <param name="boardStr">Raw board identifier returned by the device (e.g., "EXG", "IMU_...").</param>
            /// <returns>
            /// <see cref="BoardKind.EXG"/> if the string contains "EXG" (case-insensitive);
            /// <see cref="BoardKind.IMU"/> if non-empty and not EXG;
            /// otherwise <see cref="BoardKind.Unknown"/>.
            /// </returns>
            private static BoardKind MapBoardStringToKind(string? boardStr)
            {
                if (string.IsNullOrWhiteSpace(boardStr))
                    return BoardKind.Unknown;

                return boardStr.IndexOf("EXG", StringComparison.OrdinalIgnoreCase) >= 0
                    ? BoardKind.EXG
                    : BoardKind.IMU;
            }


            /// <summary>
            /// Performs a bounded BFS over the object graph to find an instance that exposes
            /// a parameterless <c>GetExpansionBoard()</c> method (via reflection).
            /// </summary>
            /// <param name="root">Root object to start the search from.</param>
            /// <param name="maxDepth">Maximum traversal depth (0 = root only).</param>
            /// <returns>
            /// The first object that has a parameterless <c>GetExpansionBoard()</c> method; otherwise <c>null</c>.
            /// </returns>
            /// <remarks>
            /// Uses reference-based visited tracking to avoid cycles and ignores primitives/enums/strings
            /// to keep the search cheap. Scans both fields and non-indexed properties (public and non-public).
            /// </remarks>
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

                    // If this object exposes GetExpansionBoard(), we are done.
                    if (HasMethod(obj, "GetExpansionBoard"))
                        return obj;

                    // Stop expanding beyond the depth limit.
                    if (depth >= maxDepth) continue;

                    // Scan fields and non-indexed properties (public and non-public).
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

                // Local helper: enqueue only non - primitive, non -enum, non-string objects
                // and only if not seen before (reference equality).
                void EnqueueIfNew(object o, int d)
                {
                    if (o == null) return;
                    var tt = o.GetType();
                    if (tt.IsPrimitive || tt.IsEnum || tt == typeof(string)) return;
                    if (visited.Add(o)) q.Enqueue((o, d));
                }
            }


            /// <summary>
            /// Checks via reflection whether the given instance exposes a parameterless instance method
            /// with the specified name (public or non-public).
            /// </summary>
            /// <param name="instance">Object to inspect.</param>
            /// <param name="methodName">Target method name.</param>
            /// <returns><c>true</c> if such a method exists; otherwise <c>false</c>.</returns>
            private static bool HasMethod(object instance, string methodName)
            {
                var t = instance.GetType();
                var m = t.GetMethod(methodName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                return m != null && m.GetParameters().Length == 0;
            }


            /// <summary>
            /// Polls (with timeout) for a non-empty expansion-board string via reflection.
            /// Calls <c>GetExpansionBoard()</c> repeatedly; halfway through the timeout it retries
            /// by invoking <c>ReadExpansionBoard()</c> once to refresh the value.
            /// </summary>
            /// <param name="target">Object exposing <c>GetExpansionBoard()</c> (and optionally <c>ReadExpansionBoard()</c>).</param>
            /// <param name="boardStr">Output: the retrieved board string, or empty if none was obtained.</param>
            /// <param name="timeoutMs">Maximum time to wait, in milliseconds.</param>
            /// <returns><c>true</c> if a non-empty board string was obtained within the timeout; otherwise <c>false</c>.</returns>
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

                    // One mid-timeout refresh attempt to trigger the device to provide the value.
                    if (!retried && waited >= timeoutMs / 2)
                    {
                        retried = true;
                        InvokeNoArgIfExists(target, "ReadExpansionBoard");
                    }
                }

                boardStr = GetStringNoArgIfExists(target, "GetExpansionBoard") ?? "";
                return !string.IsNullOrWhiteSpace(boardStr);
            }


            /// <summary>
            /// Tries to read a string from a parameterless member named <paramref name="methodName"/>:
            /// first calls the method if it exists; if not, falls back to common properties
            /// (ExpansionBoard/ExpansionBoardID/DaughterCardID). Returns null if nothing is available.
            /// </summary>
            /// <param name="instance">Object to inspect via reflection.</param>
            /// <param name="methodName">Parameterless method name to try (e.g., "GetExpansionBoard").</param>
            /// <returns>The string value if found; otherwise <c>null</c>.</returns>
            private static string? GetStringNoArgIfExists(object instance, string methodName)
            {
                var t = instance.GetType();

                // Prefer a parameterless method if available
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


            /// <summary>
            /// Sleeps for the specified number of milliseconds, swallowing any exceptions
            /// (best-effort delay guard for background/polling code).
            /// </summary>
            /// <param name="ms">Delay in milliseconds.</param>
            private static void SafeDelay(int ms)
            {
                try { System.Threading.Thread.Sleep(ms); } catch { }
            }


            /// <summary>
            /// Invokes a parameterless instance method named <paramref name="methodName"/> on
            /// <paramref name="instance"/> if it exists (public or non-public). Swallows exceptions.
            /// </summary>
            /// <param name="instance">Target object.</param>
            /// <param name="methodName">Parameterless method to invoke.</param>
            /// <returns>The invocation result, or <c>null</c> if the method is missing or fails.</returns>
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


            /// <summary>
            /// Reference-equality comparer used to track visited objects in BFS:
            /// objects are equal only if they are the same reference.
            /// </summary>
            private sealed class RefEqComparer : IEqualityComparer<object>
            {

                /// <summary>
                /// Determines whether the specified objects are the same reference.
                /// </summary>
                /// <param name="x">The first object to compare.</param>
                /// <param name="y">The second object to compare.</param>
                /// <returns>
                /// <c>true</c> if <paramref name="x"/> and <paramref name="y"/> refer to the same instance;
                /// otherwise <c>false</c>.
                /// </returns>
                public new bool Equals(object? x, object? y) => ReferenceEquals(x, y);


                /// <summary>
                /// Returns a hash code based on the object’s reference identity.
                /// </summary>
                /// <param name="obj">The object for which to get the hash code.</param>
                /// <returns>
                /// A hash code that uniquely represents the reference of the object.
                /// </returns>
                public int GetHashCode(object obj) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(obj);
            }
        }
    }
}
