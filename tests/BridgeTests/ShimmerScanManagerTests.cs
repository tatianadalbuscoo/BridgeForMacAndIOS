/*
 * ShimmerScanManagerTests.cs
 * Purpose: Unit tests for ShimmerScanManager file.
 */

using ShimmerBridgeScan;
using System.Diagnostics;
using System.Reflection;
using Xunit;

namespace BridgeTests
{
    public class ShimmerScanManagerTests
    {

        /// <summary>
        /// Test fixture setup: resets the Bluetooth stub to a clean baseline before each test.
        /// What it does:
        /// - Disables Bluetooth (IsEnabled=false) to start from a known state
        /// - Clears any previously bonded devices to avoid stale cross-test state
        /// - Ensures discovery is not running (CancelDiscovery)
        /// Expected: no assertions here — this only prepares deterministic preconditions for the tests.
        /// </summary>
        public ShimmerScanManagerTests()
        {
            var a = Android.Bluetooth.BluetoothAdapter.DefaultAdapter!;
            a.IsEnabled = false;            // BT off by default
            a.BondedDevices.Clear();        // no stale paired devices
            a.CancelDiscovery();            // discovery stopped
        }


        // ----- Entry class -----


        /// <summary>
        /// Verifies default values of <see cref="Entry"/>.
        /// Expected:
        /// - Name == ""
        /// - Mac == ""
        /// - Rssi == null
        /// - IsPaired == false
        /// - Type == Unknown
        /// </summary>
        [Fact]
        public void Entry_Defaults()
        {
            var e = new ShimmerScanManager.Entry();
            Assert.Equal("", e.Name);
            Assert.Equal("", e.Mac);
            Assert.Null(e.Rssi);
            Assert.False(e.IsPaired);
            Assert.Equal(ShimmerScanManager.DeviceType.Unknown, e.Type);
        }


        /// <summary>
        /// Verifies that property setters on <see cref="Entry"/> persist values.
        /// Expected: all assigned values are read back unchanged.
        /// </summary>
        [Fact]
        public void Entry_Setters()
        {
            var e = new ShimmerScanManager.Entry
            {
                Name = "Shimmer3-E0D9",
                Mac = "AA:BB:CC:DD:EE:FF",
                Rssi = -55,
                IsPaired = true,
                Type = ShimmerScanManager.DeviceType.IMU
            };

            Assert.Equal("Shimmer3-E0D9", e.Name);
            Assert.Equal("AA:BB:CC:DD:EE:FF", e.Mac);
            Assert.Equal(-55, e.Rssi);
            Assert.True(e.IsPaired);
            Assert.Equal(ShimmerScanManager.DeviceType.IMU, e.Type);
        }


        // ----- Result class -----


        /// <summary>
        /// Verifies the default state of Result.
        /// Expected: both lists are non-null, empty, and are two distinct List instances.
        /// </summary>
        [Fact]
        public void Result_Defaults()
        {
            var r = new ShimmerScanManager.Result();

            Assert.NotNull(r.Visible);          // list is created
            Assert.NotNull(r.Off);              // list is created
            Assert.Empty(r.Visible);            // no items by default
            Assert.Empty(r.Off);                // no items by default
            Assert.NotSame(r.Visible, r.Off);   // two separate lists
        }


        /// <summary>
        /// Ensures Visible and Off track entries independently.
        /// Expected: items added to Visible are not in Off (and vice versa); counts and membership match what we added.
        /// </summary>
        [Fact]
        public void Result_AddVisibleAndOff_TrackedSeparately()
        {
            var r = new ShimmerScanManager.Result();

            var v1 = new ShimmerScanManager.Entry { Name = "Shimmer3-A", Mac = "AA:BB:CC:00:00:01" };
            var v2 = new ShimmerScanManager.Entry { Name = "Shimmer3-B", Mac = "AA:BB:CC:00:00:02" };
            var off1 = new ShimmerScanManager.Entry { Name = "Shimmer3-C", Mac = "AA:BB:CC:00:00:03", IsPaired = true, Type = ShimmerScanManager.DeviceType.DeviceOff };

            r.Visible.Add(v1);
            r.Visible.Add(v2);
            r.Off.Add(off1);

            Assert.Equal(2, r.Visible.Count);
            Assert.Single(r.Off);

            Assert.Contains(v1, r.Visible);
            Assert.Contains(v2, r.Visible);
            Assert.Contains(off1, r.Off);

            Assert.DoesNotContain(off1, r.Visible);
            Assert.DoesNotContain(v1, r.Off);
        }


        /// <summary>
        /// Confirms Result does not enforce uniqueness.
        /// Expected: adding the same reference twice to Visible results in two entries (same object instance).
        /// </summary>
        [Fact]
        public void Result_AllowsDuplicates_IfCallerAddsThem()
        {
            var r = new ShimmerScanManager.Result();
            var e = new ShimmerScanManager.Entry { Name = "Shimmer3-D", Mac = "AA:BB:CC:00:00:04" };

            r.Visible.Add(e);
            r.Visible.Add(e);                        // List<T> allows duplicates

            Assert.Equal(2, r.Visible.Count);
            Assert.Same(r.Visible[0], r.Visible[1]); // exact same reference
        }


        // ----- Constructor -----


        /// <summary>
        /// Verifies the constructor accepts a valid <see cref="Activity"/> and does not throw.
        /// Expected: creating <see cref="ShimmerScanManager"/> with a non-null Activity completes without exceptions.
        /// </summary>
        [Fact]
        public void Ctor_DoesNotThrow_WithActivity()
        {
            var ex = Record.Exception(() => new ShimmerScanManager(new Activity()));
            Assert.Null(ex);
        }


        /// <summary>
        /// Ensures that when Bluetooth is disabled the scan returns an empty result.
        /// Setup: the stubbed <see cref="Android.Bluetooth.BluetoothAdapter"/> is disabled by default.
        /// Expected: <c>ScanAsync</c> exits early and both <c>Visible</c> and <c>Off</c> lists are empty.
        /// </summary>
        [Fact]
        public async Task Ctor_SetsAdapter_AndScanWithBluetoothDisabled_ReturnsEmpty()
        {

            // With the current stubs, DefaultAdapter starts with IsEnabled = false.
            var mgr = new ShimmerScanManager(new Activity());

            var res = await mgr.ScanAsync(TimeSpan.FromMilliseconds(10));

            Assert.Empty(res.Visible);
            Assert.Empty(res.Off);
        }


        /// <summary>
        /// Confirms that paired-but-not-discovered Shimmer devices are surfaced in the <c>Off</c> list.
        /// Setup: enable Bluetooth, add one paired Shimmer-like device, perform a short scan that discovers nothing.
        /// Expected: <c>Visible</c> is empty; <c>Off</c> contains exactly that paired device with <c>IsPaired = true</c>
        /// and <c>Type = DeviceOff</c>.
        /// </summary>
        [Fact]
        public async Task ScanAsync_ListsPairedAsOff_WhenNotDiscovered()
        {
            var adapter = Android.Bluetooth.BluetoothAdapter.DefaultAdapter!;
            adapter.IsEnabled = true;
            adapter.BondedDevices.Clear();
            adapter.BondedDevices.Add(new Android.Bluetooth.BluetoothDevice
            {
                Name = "Shimmer3-E123",
                Address = "AA:BB:CC:00:00:42"
            });

            var mgr = new ShimmerBridgeScan.ShimmerScanManager(new Activity());
            var res = await mgr.ScanAsync(TimeSpan.FromMilliseconds(5));

            Assert.Empty(res.Visible);
            Assert.Single(res.Off);
            Assert.Equal("AA:BB:CC:00:00:42", res.Off[0].Mac);
            Assert.True(res.Off[0].IsPaired);
            Assert.Equal(ShimmerBridgeScan.ShimmerScanManager.DeviceType.DeviceOff, res.Off[0].Type);
        }


        // ----- ScanAsync behavior -----


        /// <summary>
        /// When BT is ON, a paired Shimmer device that is NOT discovered during the scan
        /// must be returned in the Off list.
        /// Setup: enable Bluetooth; add one paired Shimmer-like device; short scan that discovers nothing.
        /// Expected: <c>Visible</c> is empty; <c>Off</c> has exactly that device with
        /// <c>IsPaired = true</c> and <c>Type = DeviceOff</c>.
        /// </summary>
        [Fact]
        public async Task ScanAsync_BtOn_PairedShimmerNotDiscovered_GoesToOff()
        {
            var adapter = Android.Bluetooth.BluetoothAdapter.DefaultAdapter!;
            adapter.IsEnabled = true;
            adapter.BondedDevices.Clear();
            adapter.BondedDevices.Add(new Android.Bluetooth.BluetoothDevice
            {
                Name = "Shimmer3-E123",
                Address = "AA:BB:CC:00:00:42"
            });

            var mgr = new ShimmerScanManager(new Activity());

            var res = await mgr.ScanAsync(TimeSpan.FromMilliseconds(5));

            Assert.Empty(res.Visible);
            Assert.Single(res.Off);
            Assert.Equal("AA:BB:CC:00:00:42", res.Off[0].Mac);
            Assert.True(res.Off[0].IsPaired);
            Assert.Equal(ShimmerScanManager.DeviceType.DeviceOff, res.Off[0].Type);
        }


        /// <summary>
        /// Non-Shimmer paired devices must be ignored, while Shimmer-like devices (by name or OUI)
        /// appear in the Off list if not discovered during the scan.
        /// Setup: enable Bluetooth; add three paired devices:
        /// 1) non-Shimmer (ignored),
        /// 2) Shimmer by name,
        /// 3) Shimmer by OUI prefix 00:06:66.
        /// Expected: <c>Off</c> contains the two Shimmer-like MACs only; <c>Visible</c> is empty.
        /// </summary>
        [Fact]
        public async Task ScanAsync_BtOn_IgnoresNonShimmerBonded()
        {
            var adapter = Android.Bluetooth.BluetoothAdapter.DefaultAdapter!;
            adapter.IsEnabled = true;
            adapter.BondedDevices.Clear();

            // Non-Shimmer (neither name nor OUI)
            adapter.BondedDevices.Add(new Android.Bluetooth.BluetoothDevice
            {
                Name = "Pixel-Phone",
                Address = "DE:AD:BE:EF:00:01"
            });

            // Shimmer-like by name
            adapter.BondedDevices.Add(new Android.Bluetooth.BluetoothDevice
            {
                Name = "Shimmer3-E123",
                Address = "AA:BB:CC:00:00:02"
            });

            // Shimmer-like by OUI prefix 00:06:66
            adapter.BondedDevices.Add(new Android.Bluetooth.BluetoothDevice
            {
                Name = "Whatever",
                Address = "00:06:66:12:34:56"
            });

            var mgr = new ShimmerScanManager(new Activity());
            var res = await mgr.ScanAsync(TimeSpan.FromMilliseconds(5));

            Assert.Empty(res.Visible);
            Assert.Equal(2, res.Off.Count); // only the two shimmer-like devices
            Assert.Contains(res.Off, e => e.Mac == "AA:BB:CC:00:00:02");
            Assert.Contains(res.Off, e => e.Mac == "00:06:66:12:34:56");
            Assert.DoesNotContain(res.Off, e => e.Mac == "DE:AD:BE:EF:00:01");
        }


        /// <summary>
        /// The scan must start discovery and always stop it by the end (even on short runs).
        /// Setup: enable Bluetooth and ensure the adapter is not discovering.
        /// Expected: after <c>ScanAsync</c>, <c>IsDiscovering</c> is <c>false</c>,
        /// proving the manager toggled discovery and cleaned up correctly.
        /// </summary>
        [Fact]
        public async Task ScanAsync_TogglesDiscovery_StartsAndStops()
        {
            var adapter = Android.Bluetooth.BluetoothAdapter.DefaultAdapter!;
            adapter.IsEnabled = true;
            adapter.BondedDevices.Clear();
            adapter.CancelDiscovery(); // precondition: not discovering

            var mgr = new ShimmerScanManager(new Activity());
            var res = await mgr.ScanAsync(TimeSpan.FromMilliseconds(5));

            // We don't assert on result contents here; we only verify the Start/Cancel lifecycle.
            Assert.False(adapter.IsDiscovering);    // must be stopped after finally
        }


        // ----- LooksLikeShimmer behavior -----


        /// <summary>
        /// Verifies the Shimmer heuristic on many name/MAC combinations.
        /// Expected:
        /// - TRUE when the device name contains "Shimmer" (any case), starts with "SHIMMER3",
        ///   or when it looks like an RN-42 module ("RN42", "RNBT", "RN-42").
        /// - TRUE when the MAC uses the RN-42 OUI prefix "00:06:66" (case-insensitive).
        /// - FALSE for unrelated names/MACs and for null/empty inputs.
        /// </summary>
        [Theory]

        // Match by name containing "Shimmer"
        [InlineData("Shimmer3-E123", "11:22:33:44:55:66", true)]
        [InlineData("my shimmer device", "AA:BB:CC:DD:EE:FF", true)]

        // Match by "SHIMMER3" prefix
        [InlineData("SHIMMER3-E123", "AA:BB:CC:00:00:01", true)]

        // Match by typical RN-42 module name prefixes
        [InlineData("RN42-Module", "AA:BB:CC:00:00:02", true)]
        [InlineData("RNBT-1234", "AA:BB:CC:00:00:03", true)]
        [InlineData("RN-42-foo", "AA:BB:CC:00:00:04", true)]

        // Match by RN-42 OUI prefix
        [InlineData("Whatever", "00:06:66:12:34:56", true)]

        // Non-matches
        [InlineData("Pixel-Phone", "DE:AD:BE:EF:00:01", false)]
        [InlineData("Something", "11:22:33:44:55:66", false)]

        // Case-insensitive checks
        [InlineData("rn42-something", "aa:bb:cc:dd:ee:01", true)]
        [InlineData("shimmerX", "aa:bb:cc:dd:ee:02", true)]

        // Null/empty handling
        [InlineData(null, null, false)]
        [InlineData("", "", false)]
        public void LooksLikeShimmer_VariousCases(string? name, string? mac, bool expected)
        {
            var ok = ShimmerScanManager.LooksLikeShimmer(name, mac);
            Assert.Equal(expected, ok);
        }


        /// <summary>
        /// Confirms the heuristic is tolerant to whitespace/case on name and MAC.
        /// Expected:
        /// - Leading whitespace before a name that contains "Shimmer" still returns TRUE.
        /// - Lowercase OUI "00:06:66" is treated the same as uppercase and returns TRUE.
        /// </summary>
        [Fact]
        public void LooksLikeShimmer_AllowsWhitespaceAndMixedCase()
        {
            Assert.True(ShimmerScanManager.LooksLikeShimmer("  Shimmer-Unit", "AA:BB:CC:00:00:10"));
            Assert.True(ShimmerScanManager.LooksLikeShimmer("foo", "00:06:66:aa:bb:cc"));
        }


        // ----- DiscoveryReceiver class -----
        // ----- Constructor -----


        /// <summary>
        /// Helper: fetches the non-public nested type
        /// 'DiscoveryReceiver' from <see cref="ShimmerScanManager"/> via reflection.
        /// Asserts the type exists and returns it for further reflective use.
        /// </summary>
        private static Type GetReceiverType()
        {
            var t = typeof(ShimmerScanManager).GetNestedType("DiscoveryReceiver", BindingFlags.NonPublic);
            Assert.NotNull(t);
            return t!;
        }


        /// <summary>
        /// Helper: constructs a 'DiscoveryReceiver' instance via reflection,
        /// passing the two callbacks required by its constructor (onFound, onFinished).
        /// Asserts the constructor is present and returns the created instance.
        /// </summary>
        private static object CreateReceiver(Action<Android.Bluetooth.BluetoothDevice?, int?> onFound, Action onFinished)
        {
            var t = GetReceiverType();
            var ctor = t.GetConstructor(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
                                        binder: null,
                                        types: new[] { typeof(Action<Android.Bluetooth.BluetoothDevice?, int?>), typeof(Action) },
                                        modifiers: null);
            Assert.NotNull(ctor);
            return ctor!.Invoke(new object[] { onFound, onFinished });
        }


        /// <summary>
        /// Helper: invokes the public 'OnReceive' method on a 'DiscoveryReceiver'
        /// instance via reflection, passing a null Context and the provided Intent.
        /// Used to simulate broadcast delivery in tests without Android runtime.
        /// </summary>
        private static void InvokeOnReceive(object receiver, Android.Content.Intent? intent)
        {
            var t = receiver.GetType();
            var mi = t.GetMethod("OnReceive", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mi);

            // Context can be null in our stubs.
            mi!.Invoke(receiver, new object?[] { null, intent });
        }


        /// <summary>
        /// Verifies that when an intent with action 'BluetoothDevice.ActionFound' is received,
        /// the receiver invokes the 'onFound' callback and does NOT invoke 'onFinished'.
        /// Expected: 'calledFound' is true, no exception is thrown, and since our stubs
        /// don't expose extras, both 'devArg' and 'rssiArg' remain null.
        /// </summary>
        [Fact]
        public void DiscoveryReceiver_OnFound_IsInvoked_On_ActionFound()
        {
            bool calledFound = false;
            Android.Bluetooth.BluetoothDevice? devArg = null;
            int? rssiArg = null;

            var recv = CreateReceiver(
                onFound: (dev, rssi) => { calledFound = true; devArg = dev; rssiArg = rssi; },
                 onFinished: () => Assert.Fail("Should not be called for ActionFound")
            );

            var intent = new Android.Content.Intent { Action = Android.Bluetooth.BluetoothDevice.ActionFound };

            var ex = Record.Exception(() => InvokeOnReceive(recv, intent));
            Assert.Null(ex);

            Assert.True(calledFound);

            // With current stubs, GetParcelableExtra/HasExtra return null/false.
            Assert.Null(devArg);
            Assert.Null(rssiArg);
        }


        /// <summary>
        /// Verifies that when an intent with action 'BluetoothAdapter.ActionDiscoveryFinished' is received,
        /// the receiver invokes the 'onFinished' callback and does NOT invoke 'onFound'.
        /// Expected: 'calledFinished' is true and no exception is thrown.
        /// </summary>
        [Fact]
        public void DiscoveryReceiver_OnFinished_IsInvoked_On_ActionDiscoveryFinished()
        {
            bool calledFinished = false;

            var recv = CreateReceiver(
                onFound: (dev, rssi) => Assert.Fail("Should not be called for ActionDiscoveryFinished"),
                onFinished: () => { calledFinished = true; }
            );

            var intent = new Android.Content.Intent { Action = Android.Bluetooth.BluetoothAdapter.ActionDiscoveryFinished };

            var ex = Record.Exception(() => InvokeOnReceive(recv, intent));
            Assert.Null(ex);

            Assert.True(calledFinished);
        }


        /// <summary>
        /// Verifies that passing a null intent to 'OnReceive' is a no-op:
        /// no exception is thrown and neither callback is invoked.
        /// Expected: both 'calledFound' and 'calledFinished' remain false.
        /// </summary>
        [Fact]
        public void DiscoveryReceiver_OnReceive_NullIntent_DoesNotThrow_AndNoCallbacks()
        {
            bool calledFound = false;
            bool calledFinished = false;

            var recv = CreateReceiver(
                onFound: (dev, rssi) => { calledFound = true; },
                onFinished: () => { calledFinished = true; }
            );

            var ex = Record.Exception(() => InvokeOnReceive(recv, intent: null));
            Assert.Null(ex);

            Assert.False(calledFound);
            Assert.False(calledFinished);
        }


        // ----- OnReceive behavior -----


        /// <summary>
        /// Helper: Returns the non-public nested type 'DiscoveryReceiver' from ShimmerScanManager.
        /// Used by tests to reflectively construct the receiver.
        /// Expected: never returns null; throws if the type is missing.
        /// </summary>
        static Type RxType() =>
           typeof(ShimmerScanManager).GetNestedType("DiscoveryReceiver", BindingFlags.NonPublic)!;


        /// <summary>
        /// Helper: Creates a new DiscoveryReceiver instance via reflection, wiring the provided callbacks.
        /// Expected: returns a valid instance constructed with (onFound, onFinished).
        /// </summary>
        static object NewRx(Action<Android.Bluetooth.BluetoothDevice?, int?> onFound, Action onFinished)
        {
            var t = RxType();
            var ctor = t.GetConstructor(
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
                null,
                new[] { typeof(Action<Android.Bluetooth.BluetoothDevice?, int?>), typeof(Action) },
                null
            )!;
            return ctor.Invoke(new object[] { onFound, onFinished });
        }


        /// <summary>
        /// Helper: Invokes the receiver's OnReceive method with a null context and the given intent.
        /// Expectation: forwards the broadcast to the receiver without throwing.
        /// </summary>
        static void CallOnReceive(object rx, Android.Content.Intent? intent)
        {
            var mi = rx.GetType().GetMethod("OnReceive", BindingFlags.Instance | BindingFlags.Public)!;
            mi.Invoke(rx, new object?[] { null, intent });
        }


        /// <summary>
        /// Verifies that action 'BluetoothDevice.ActionFound' triggers the onFound callback,
        /// while onFinished is NOT called. With current stubs, extras are absent.
        /// Expected: calledFound == true, dev == null, rssi == null.
        /// </summary>
        [Fact]
        public void ActionFound_calls_onFound_with_null_args_with_current_stubs()
        {
            bool calledFound = false;
            Android.Bluetooth.BluetoothDevice? dev = null; int? rssi = null;

            var rx = NewRx(
                onFound: (d, r) => { calledFound = true; dev = d; rssi = r; },
                onFinished: () => Assert.Fail("onFinished should not be called")
            );

            var intent = new Android.Content.Intent { Action = Android.Bluetooth.BluetoothDevice.ActionFound };

            CallOnReceive(rx, intent);

            Assert.True(calledFound);
            Assert.Null(dev);    // with current stubs no extras are produced
            Assert.Null(rssi);
        }


        /// <summary>
        /// Verifies that action 'BluetoothAdapter.ActionDiscoveryFinished' triggers the onFinished callback
        /// and does NOT call onFound.
        /// Expected: finished == true.
        /// </summary>
        [Fact]
        public void ActionDiscoveryFinished_calls_onFinished()
        {
            bool finished = false;

            var rx = NewRx(
                onFound: (d, r) => Assert.Fail("onFound should not be called"),
                onFinished: () => finished = true
            );

            var intent = new Android.Content.Intent { Action = Android.Bluetooth.BluetoothAdapter.ActionDiscoveryFinished };

            CallOnReceive(rx, intent);

            Assert.True(finished);
        }


        /// <summary>
        /// Verifies that a null intent results in a no-op: no callbacks are fired and no exception is thrown.
        /// Expected: calledFound == false and calledFinished == false.
        /// </summary>
        [Fact]
        public void NullIntent_does_nothing()
        {
            bool calledFound = false, calledFinished = false;

            var rx = NewRx(
                onFound: (d, r) => calledFound = true,
                onFinished: () => calledFinished = true
            );

            CallOnReceive(rx, null);

            Assert.False(calledFound);
            Assert.False(calledFinished);
        }


        // ----- ShimmerBoardDetector class -----
        // ----- GetExpansionBoardKindAndroidAsync behavior -----


        /// <summary>
        /// Ensures the detector short-circuits on invalid MAC input.
        /// Expected: returns ok == false, kind == Unknown, raw == "Invalid MAC".
        /// </summary>
        [Fact]
        public async Task GetExpansionBoardKindAndroidAsync_InvalidMac_EarlyReturns()
        {
            var (ok, kind, raw) =
                await ShimmerBridgeScan.ShimmerScanManager.ShimmerBoardDetector
                    .GetExpansionBoardKindAndroidAsync("any", mac: "");

            Assert.False(ok);
            Assert.Equal(ShimmerScanManager.ShimmerBoardDetector.BoardKind.Unknown, kind);
            Assert.Equal("Invalid MAC", raw);
        }


        /// <summary>
        /// Verifies behavior when the SDK session connects but no board info can be detected
        /// (with current test stub: Connect() succeeds; TryDetectBoardKind fails → "Detection failed").
        /// Expected: returns ok == false, kind == Unknown, raw == "Detection failed".
        /// </summary>
        [Fact]
        public async Task GetExpansionBoardKindAndroidAsync_NotConnected_TimesOut()
        {

            // With current stubs: Connect() sets Connected=true, but no expansion data is found,
            // so the method returns (false, Unknown, "Detection failed").
            var (ok, kind, raw) =
                await ShimmerBridgeScan.ShimmerScanManager.ShimmerBoardDetector
                    .GetExpansionBoardKindAndroidAsync("Shimmer3-ABC", "AA:BB:CC:00:00:42");

            Assert.False(ok);
            Assert.Equal(ShimmerScanManager.ShimmerBoardDetector.BoardKind.Unknown, kind);
            Assert.Equal("Detection failed", raw);
        }


        // ----- TryDetectBoardKind behavior -----


        /// <summary>
        /// Helper: Minimal fake “expansion target” the reflection-based detector can discover.
        /// Exposes power/read calls and <c>GetExpansionBoard()</c>, with the return
        /// value controlled by the <see cref="Board"/> property.
        /// </summary>
        internal sealed class FakeExpansionTarget
        {
            public string? Board { get; set; }
            public void ReadInternalExpPower() { }
            public void WriteInternalExpPower() { }
            public void ReadExpansionBoard() { }
            public string? GetExpansionBoard() => Board;
        }


        /// <summary>
        /// Helper: Fake target that throws from <c>GetExpansionBoard()</c>.
        /// Used to verify the detector swallows exceptions and reports failure.
        /// </summary>
        internal sealed class ThrowingExpansionTarget
        {
            public void ReadInternalExpPower() { }
            public void WriteInternalExpPower() { }
            public void ReadExpansionBoard() { }
            public string GetExpansionBoard() => throw new InvalidOperationException("boom");
        }


        /// <summary>
        /// Helper: Reflection helper that invokes the internal
        /// <c>ShimmerBoardDetector.TryDetectBoardKind</c> and returns tuple-like values:
        /// <c>(ok, kind as int, raw string)</c>.
        /// </summary>
        static (bool ok, int kindVal, string raw) Invoke_TryDetect(ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2 shim)
        {
            var detectorType = typeof(ShimmerBridgeScan.ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic)!;
            var boardKindType = detectorType.GetNestedType("BoardKind", BindingFlags.Public)!;

            var mi = detectorType.GetMethod(
                "TryDetectBoardKind",
                BindingFlags.Public | BindingFlags.Static,
                binder: null,
                types: new[] {
            typeof(ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2),
            boardKindType.MakeByRefType(),
            typeof(string).MakeByRefType()
                },
                modifiers: null
            )!;

            var args = new object?[] { shim, null, null };
            var ok = (bool)mi.Invoke(null, args)!;

            int kindVal = Convert.ToInt32(args[1]);    // enum boxed -> int
            string raw = (string)(args[2] ?? "");
            return (ok, kindVal, raw);
        }


        /// <summary>
        /// Not connected -> detector should fail early.
        /// Expected: ok=false, kind=Unknown(0), raw="".
        /// </summary>
        [Fact]
        public void TryDetectBoardKind_NotConnected_ReturnsFalseUnknown()
        {
            var shim = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("dev", "AA:BB:CC");
            var (ok, kind, raw) = Invoke_TryDetect(shim);
            Assert.False(ok);
            Assert.Equal(0, kind);     // Unknown
            Assert.Equal("", raw);
        }


        /// <summary>
        /// Board string contains "EXG" (any case) -> map to EXG.
        /// Expected: ok=true, kind=EXG(2), raw="EXG_DAUGHTER".
        /// </summary>
        [Fact]
        public void TryDetectBoardKind_Maps_EXG()
        {
            var shim = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("dev", "AA:BB:CC");
            shim.Connect();
            shim.ExpansionTarget = new FakeExpansionTarget { Board = "EXG_DAUGHTER" };

            var (ok, kind, raw) = Invoke_TryDetect(shim);
            Assert.True(ok);
            Assert.Equal(2, kind);                 // EXG
            Assert.Equal("EXG_DAUGHTER", raw);
        }


        /// <summary>
        /// Non-empty board string without "EXG" -> fallback to IMU.
        /// Expected: ok=true, kind=IMU(1), raw="IMU_MPU".
        /// </summary>
        [Fact]
        public void TryDetectBoardKind_Maps_IMU()
        {
            var shim = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("dev", "AA:BB:CC");
            shim.Connect();
            shim.ExpansionTarget = new FakeExpansionTarget { Board = "IMU_MPU" };

            var (ok, kind, raw) = Invoke_TryDetect(shim);
            Assert.True(ok);
            Assert.Equal(1, kind);                 // IMU
            Assert.Equal("IMU_MPU", raw);
        }


        /// <summary>
        /// Empty/whitespace board string means detection didn’t yield a value.
        /// Expected: ok=false, kind=Unknown(0), raw="".
        /// </summary>
        [Theory]
        [InlineData(null)]
        [InlineData("")]
        [InlineData("   ")]
        public void TryDetectBoardKind_EmptyOrWhitespace_ReturnsFalse(string? board)
        {
            var shim = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("dev", "AA:BB:CC");
            shim.Connect();
            shim.ExpansionTarget = new FakeExpansionTarget { Board = board };

            var (ok, kind, raw) = Invoke_TryDetect(shim);
            Assert.False(ok);
            Assert.Equal(0, kind);     // Unknown
            Assert.Equal("", raw);
        }


        /// <summary>
        /// If the expansion target throws during read, the detector must catch and fail gracefully.
        /// Expected: ok=false, kind=Unknown(0), raw="".
        /// </summary>
        [Fact]
        public void TryDetectBoardKind_Exceptions_AreCaught_ReturnsFalse()
        {
            var shim = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("dev", "AA:BB:CC");
            shim.Connect();
            shim.ExpansionTarget = new ThrowingExpansionTarget();

            var (ok, kind, raw) = Invoke_TryDetect(shim);
            Assert.False(ok);
            Assert.Equal(0, kind);     // Unknown
            Assert.Equal("", raw);
        }


        // ----- MapBoardStringToKind behavior -----


        /// <summary>
        /// Helper: Reflection helper that retrieves the private static
        /// <c>MapBoardStringToKind</c> method from <c>ShimmerBoardDetector</c>.
        /// Used by the tests below to invoke the non-public API. Expected: returns a valid MethodInfo.
        /// </summary>
        static MethodInfo Get_MapBoardStringToKind()
        {
            var detType = typeof(ShimmerBridgeScan.ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(detType);

            var mi = detType!.GetMethod("MapBoardStringToKind",
                        BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Helper: Invokes the private <c>MapBoardStringToKind</c> via reflection.
        /// Expected: returns a boxed <c>BoardKind</c> enum for the given input string.
        /// </summary>
        static object Call_MapBoardStringToKind(string? s)
            => Get_MapBoardStringToKind().Invoke(null, new object?[] { s })!;


        /// <summary>
        /// Helper: Returns the enum name (e.g., "IMU", "EXG", "Unknown")
        /// from a boxed enum value. Expected: the symbolic name of the enum.
        /// </summary>
        static string EnumName(object enumVal)
            => Enum.GetName(enumVal.GetType(), enumVal)!;


        /// <summary>
        /// Verifies that null/empty/whitespace inputs map to <c>Unknown</c>.
        /// Expected: "Unknown" for all provided inputs.
        /// </summary>
        [Theory]
        [InlineData(null, "Unknown")]
        [InlineData("", "Unknown")]
        [InlineData("   ", "Unknown")]
        public void MapBoardStringToKind_NullOrWhitespace_Unknown(string? input, string expected)
        {
            var res = Call_MapBoardStringToKind(input);
            Assert.Equal(expected, EnumName(res));
        }


        /// <summary>
        /// Verifies that any case-insensitive occurrence of "EXG" maps to <c>EXG</c>.
        /// Expected: "EXG" for all provided inputs (upper/lower/mixed/embedded).
        /// </summary>
        [Theory]
        [InlineData("EXG", "EXG")]
        [InlineData("exg", "EXG")]
        [InlineData("abcEXGdef", "EXG")]
        [InlineData("pre_exg_post", "EXG")]
        public void MapBoardStringToKind_ContainsEXG_EXG(string input, string expected)
        {
            var res = Call_MapBoardStringToKind(input);
            Assert.Equal(expected, EnumName(res));
        }


        /// <summary>
        /// Verifies that a non-empty string that does NOT contain "EXG" is mapped to <c>IMU</c>.
        /// Expected: "IMU" for all provided inputs (no "EXG" present).
        /// </summary
        [Theory]
        [InlineData("IMU_9250", "IMU")]
        [InlineData("imu", "IMU")]
        [InlineData("anything-not-imu", "IMU")]  
        [InlineData("foo-bar", "IMU")]            
        public void MapBoardStringToKind_NonEmptyNoEXG_IMU(string input, string expected)
        {
            var res = Call_MapBoardStringToKind(input);
            Assert.Equal(expected, EnumName(res));
        }


        // ----- FindExpansionTarget behavior -----


        /// <summary>
        /// Helper: obtains the private static method ShimmerBoardDetector.FindExpansionTarget.
        /// </summary>
        /// <returns>
        /// A non-null <see cref="System.Reflection.MethodInfo"/> for the private
        /// <c>FindExpansionTarget</c> method; the test will fail if it cannot be located.
        /// </returns>
        static MethodInfo Get_FindExpansionTarget()
        {
            var tSbm = typeof(ShimmerScanManager).GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(tSbm);
            var mi = tSbm!.GetMethod("FindExpansionTarget", BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Reflection helper: invokes FindExpansionTarget(root, maxDepth).
        /// Expected: returns the object that exposes GetExpansionBoard(), or null when not found/allowed by depth.
        /// </summary>
        /// <returns>
        /// The discovered target object that implements a parameterless <c>GetExpansionBoard()</c>,
        /// or <c>null</c> if no suitable target is found within the specified depth.
        /// </returns>
        static object? Call_FindExpansionTarget(object? root, int maxDepth)
        {
            var mi = Get_FindExpansionTarget();
            return mi.Invoke(null, new object?[] { root!, maxDepth });
        }


        // ----- test types (object graph dummies) -----

        /// <summary>
        /// Dummy node with a direct parameterless GetExpansionBoard() method.
        /// Used to simulate a target at the root level.
        /// </summary>
        class TargetOnRoot
        {
            public string GetExpansionBoard() => "EXG";
        }


        /// <summary>
        /// Dummy holder exposing a field that may point to a target node.
        /// </summary>
        class WithField
        {
            public object? Child;
        }


        /// <summary>
        /// Dummy holder exposing a non-indexed property that may point to a target node.
        /// </summary>
        class WithProperty
        {
            public object? Child { get; set; }
        }


        /// <summary>
        /// Dummy target node exposing GetExpansionBoard().
        /// </summary>
        class Target
        {
            public string GetExpansionBoard() => "EXG";
        }


        /// <summary>
        /// Simple linked node used to build cycles and deeper graphs.
        /// </summary>
        class Node
        {
            public object? Next;
        }


        // ----- tests -----

        /// <summary>
        /// Returns null when root is null.
        /// Expected: null.
        /// </summary>
        [Fact]
        public void FindExpansionTarget_NullRoot_ReturnsNull()
        {
            var res = Call_FindExpansionTarget(null!, maxDepth: 3);
            Assert.Null(res);
        }


        /// <summary>
        /// Returns null when maxDepth is negative.
        /// Expected: null for any negative depth.
        /// </summary>
        [Theory]
        [InlineData(-1)]
        [InlineData(-5)]
        public void FindExpansionTarget_NegativeDepth_ReturnsNull(int depth)
        {
            var res = Call_FindExpansionTarget(new object(), depth);
            Assert.Null(res);
        }


        /// <summary>
        /// Finds the target on the root when depth is 0.
        /// Expected: returns the root instance (TargetOnRoot).
        /// </summary>
        [Fact]
        public void FindExpansionTarget_FindsOnRoot()
        {
            var root = new TargetOnRoot();
            var res = Call_FindExpansionTarget(root, maxDepth: 0);
            Assert.Same(root, res);
        }


        /// <summary>
        /// Finds a target reachable via a field within the allowed depth.
        /// Expected: returns the object stored in WithField.Child.
        /// </summary>
        [Fact]
        public void FindExpansionTarget_FindsInField_WithinDepth()
        {
            var holder = new WithField { Child = new Target() };
            var res = Call_FindExpansionTarget(holder, maxDepth: 1);
            Assert.Same(holder.Child, res);
        }


        /// <summary>
        /// Finds a target reachable via a property within the allowed depth.
        /// Expected: returns the Target instance referenced by WithProperty.Child.
        /// </summary>
        [Fact]
        public void FindExpansionTarget_FindsInProperty_WithinDepth()
        {
            var target = new Target();
            var holder = new WithProperty { Child = target };
            var res = Call_FindExpansionTarget(holder, maxDepth: 1);
            Assert.Same(target, res);
        }


        /// <summary>
        /// Respects the maxDepth limit; does not search deeper than allowed.
        /// Expected: returns null when the target is below the permitted depth.
        /// </summary>
        [Fact]
        public void FindExpansionTarget_RespectsMaxDepth_TargetTooDeep_ReturnsNull()
        {
            var deep = new WithField { Child = new WithField { Child = new Target() } };
            var res = Call_FindExpansionTarget(deep, maxDepth: 1);
            Assert.Null(res);
        }


        /// <summary>
        /// Handles cycles without infinite loops by tracking visited references.
        /// Expected: returns the reachable target despite graph cycles.
        /// </summary>
        [Fact]
        public void FindExpansionTarget_HandlesCycles_WithoutInfiniteLoop()
        {
            var a = new Node();
            var b = new Node();
            var c = new WithField { Child = new Target() };

            a.Next = b;
            b.Next = a;     // cycle
            a.Next = c;     // path to target

            var res = Call_FindExpansionTarget(a, maxDepth: 3);
            Assert.Same(c.Child, res);
        }


        /// <summary>
        /// Ignores primitives, enums, and strings when traversing the graph.
        /// Expected: returns null when only primitive/string branches are present.
        /// </summary>
        [Fact]
        public void FindExpansionTarget_IgnoresPrimitivesEnumsStrings()
        {
            var holder = new WithField { Child = "not-an-object-graph-node" };
            var res = Call_FindExpansionTarget(holder, maxDepth: 2);
            Assert.Null(res);
        }


        // ----- HasMethod behavior -----


        /// <summary>
        /// Helper: Reflection helper that fetches the private static ShimmerBoardDetector.HasMethod.
        /// </summary>
        /// <returns>
        /// A <see cref="MethodInfo"/> representing the private static <c>HasMethod</c>
        /// defined on <c>ShimmerScanManager.ShimmerBoardDetector</c>.
        /// </returns>
        static MethodInfo Get_HasMethod()
        {
            var tDet = typeof(ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(tDet);

            var mi = tDet!.GetMethod("HasMethod",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Reflection helper: invokes HasMethod(instance, methodName) and returns its bool result.
        /// </summary>
        /// <param name="instance">Object to inspect for the target method.</param>
        /// <param name="name">Method name to look up on <paramref name="instance"/>.</param>
        /// <returns>
        /// <c>true</c> if <paramref name="instance"/> exposes a parameterless instance method
        /// named <paramref name="name"/> (public or non-public); otherwise <c>false</c>.
        /// </returns>
        static bool Call_HasMethod(object instance, string name)
        {
            var mi = Get_HasMethod();
            return (bool)mi.Invoke(null, new object[] { instance, name })!;
        }


        // ----- test types (dummies used by the tests) -----

        /// <summary>
        /// Public parameterless instance method: should be detected as true.
        /// </summary>
        class PubParamless 
        { 
            public void Foo() { } 
        }


        /// <summary>
        /// Private parameterless instance method: should be detected as true.
        /// </summary>
        class PrivParamless 
        { 
            private void Bar() { } 
        }


        /// <summary>
        /// Instance method with parameters: should NOT be considered parameterless.
        /// </summary>
        class WithParams 
        { 
            public void Baz(int x) { } 
        }


        /// <summary>
        /// Static parameterless method: should NOT be considered (instance methods only).
        /// </summary>
        class StaticOnly 
        { 
            public static void Zap() { } 
        }


        // ----- tests -----

        /// <summary>
        /// Verifies that a public parameterless instance method is found.
        /// Expected: true.
        /// </summary>
        [Fact]
        public void HasMethod_PublicParameterless_ReturnsTrue()
        {
            var ok = Call_HasMethod(new PubParamless(), "Foo");
            Assert.True(ok);
        }


        /// <summary>
        /// Verifies that a non-public (private) parameterless instance method is found.
        /// Expected: true.
        /// </summary>
        [Fact]
        public void HasMethod_NonPublicParameterless_ReturnsTrue()
        {
            var ok = Call_HasMethod(new PrivParamless(), "Bar");
            Assert.True(ok);
        }


        /// <summary>
        /// Verifies that methods with parameters are excluded.
        /// Expected: false.
        /// </summary>
        [Fact]
        public void HasMethod_WithParameters_ReturnsFalse()
        {
            var ok = Call_HasMethod(new WithParams(), "Baz");
            Assert.False(ok);
        }


        /// <summary>
        /// Verifies that static methods are not considered (the check is for instance methods).
        /// Expected: false.
        /// </summary>
        [Fact]
        public void HasMethod_StaticMethod_ReturnsFalse()
        {
            var ok = Call_HasMethod(new StaticOnly(), "Zap");
            Assert.False(ok);
        }


        /// <summary>
        /// Verifies that a missing method name returns false.
        /// Expected: false.
        /// </summary>
        [Fact]
        public void HasMethod_MissingMethod_ReturnsFalse()
        {
            var ok = Call_HasMethod(new PubParamless(), "Nope");
            Assert.False(ok);
        }


        /// <summary>
        /// Verifies that passing a null instance throws when HasMethod dereferences instance.GetType().
        /// Expected: TargetInvocationException with InnerException = NullReferenceException.
        /// </summary>
        [Fact]
        public void HasMethod_NullInstance_Throws()
        {
            var mi = Get_HasMethod();
            var ex = Assert.Throws<TargetInvocationException>(() =>
                mi.Invoke(null, new object?[] { null, "Foo" }));
            Assert.IsType<NullReferenceException>(ex.InnerException);
        }


        // ----- TryWaitExpansionString behavior -----


        /// <summary>
        /// Reflection helper: resolves the private static ShimmerBoardDetector.TryWaitExpansionString.
        /// </summary>
        /// <returns>
        /// A <see cref="MethodInfo"/> representing the non-public static method
        /// <c>TryWaitExpansionString(object target, out string boardStr, int timeoutMs)</c>
        /// on <c>ShimmerBoardDetector</c>.
        /// </returns>
        static MethodInfo Get_TryWaitExpansionString()
        {
            var tDet = typeof(ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(tDet);

            var mi = tDet!.GetMethod("TryWaitExpansionString",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Reflection helper: invokes TryWaitExpansionString(target, out boardStr, timeoutMs).
        /// </summary>
        /// <param name="target">Object exposing <c>GetExpansionBoard()</c> (and optionally <c>ReadExpansionBoard()</c>).</param>
        /// <param name="timeoutMs">Maximum time to wait, in milliseconds.</param>
        /// <returns>
        /// A tuple <c>(ok, board)</c> where:
        /// - <c>ok</c> is <c>true</c> if a non-empty board string was obtained within the timeout; otherwise <c>false</c>.
        /// - <c>board</c> is the retrieved board string (or empty if none was obtained).
        /// </returns>
        static (bool ok, string board) Call_TryWait(object target, int timeoutMs)
        {
            var mi = Get_TryWaitExpansionString();

            object?[] args = new object?[] { target, null, timeoutMs };
            var ok = (bool)mi.Invoke(null, args)!;
            var board = (string?)args[1] ?? "";
            return (ok, board);
        }


        // ----- test targets (fakes used by reflection in TryWaitExpansionString) -----

        /// <summary>
        /// Obtains the private static method ShimmerBoardDetector.FindExpansionTarget.
        /// </summary>
        /// <returns>The <see cref="MethodInfo"/> for the private FindExpansionTarget method.</returns>
        class ImmediateTarget
        {
            public string GetExpansionBoard() => "EXG_READY";
        }


        /// <summary>
        /// Fake target that only returns a value after ReadExpansionBoard is invoked once (retry path).
        /// Expected: TryWaitExpansionString should fail first checks, trigger ReadExpansionBoard at mid-timeout,
        /// then succeed when GetExpansionBoard starts returning a non-empty string.
        /// </summary>
        class RetryTarget
        {
            private bool _ready = false;

            public void ReadExpansionBoard() => _ready = true;

            public string GetExpansionBoard() => _ready ? "IMU_READY" : "";
        }


        /// <summary>
        /// Fake target that never returns a value, even after ReadExpansionBoard.
        /// Expected: TryWaitExpansionString should time out and return false with an empty board string.
        /// </summary>
        class NeverTarget
        {
            public void ReadExpansionBoard() {}
            public string GetExpansionBoard() => "";
        }


        // ----- tests -----

        /// <summary>
        /// Ensures an immediate non-empty board string short-circuits the loop.
        /// Expected: ok = true and board = "EXG_READY" with a tiny timeout.
        /// </summary>
        [Fact]
        public void TryWaitExpansionString_ImmediateValue_ReturnsTrue_NoSleep()
        {
            var t = new ImmediateTarget();

            // The method checks once before the loop: should exit immediately.
            var (ok, board) = Call_TryWait(t, timeoutMs: 5);

            Assert.True(ok);
            Assert.Equal("EXG_READY", board);
        }


        /// <summary>
        /// Ensures the mid-timeout retry via ReadExpansionBoard enables a later success.
        /// Expected: ok = true and board = "IMU_READY" with a timeout large enough to cross the retry threshold.
        /// </summary>
        [Fact]
        public void TryWaitExpansionString_ValueAppearsAfterRetry_ReturnsTrue()
        {
            var t = new RetryTarget();

            var (ok, board) = Call_TryWait(t, timeoutMs: 120);

            Assert.True(ok);
            Assert.Equal("IMU_READY", board);
        }


        /// <summary>
        /// Ensures the method returns false when no non-empty string is obtained before timeout.
        /// Expected: ok = false and board = "".
        /// </summary>
        [Fact]
        public void TryWaitExpansionString_Timeout_ReturnsFalse_EmptyBoard()
        {
            var t = new NeverTarget();

            var (ok, board) = Call_TryWait(t, timeoutMs: 120);

            Assert.False(ok);
            Assert.Equal("", board);
        }


        // ----- GetStringNoArgIfExists behavior -----


        /// <summary>
        /// Helper: resolves the private static ShimmerBoardDetector.GetStringNoArgIfExists.
        /// </summary>
        /// <returns>The <see cref="MethodInfo"/> for the private GetStringNoArgIfExists helper.</returns>
        static MethodInfo MI_GetStringNoArgIfExists()
        {
            var tDet = typeof(ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(tDet);

            var mi = tDet!.GetMethod("GetStringNoArgIfExists",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Helper: invokes GetStringNoArgIfExists(instance, methodName).
        /// </summary>
        /// <param name="instance">Object to inspect via reflection.</param>
        /// <param name="methodName">Parameterless method name to try (e.g., "GetExpansionBoard").</param>
        /// <returns>The string value produced by the private helper, or <c>null</c> if none.</returns>
        static string? Call_GetStringNoArgIfExists(object instance, string methodName)
        {
            var mi = MI_GetStringNoArgIfExists();
            var res = mi.Invoke(null, new object?[] { instance, methodName });
            return (string?)res;
        }


        // ----- Fake targets for scenarios -----

        /// <summary>
        /// Method exists and returns a string.
        /// </summary>
        /// <returns>"EXG_FROM_METHOD"</returns>
        class MethodReturnsString
        {
            public string GetExpansionBoard() => "EXG_FROM_METHOD";
        }


        /// <summary>
        /// Method exists but returns a non-string value.
        /// </summary>
        /// <returns>42</returns>
        class MethodReturnsNonString
        {
            public int GetExpansionBoard() => 42;
        }


        /// <summary>
        /// Method exists but throws; a property fallback is available.
        /// </summary>
        /// <returns>Throws on <c>GetExpansionBoard()</c>; fallback property provides "EXG_FROM_PROP".</returns>
        class MethodThrowsThenProp
        {
            public string GetExpansionBoard() => throw new System.Exception("boom");
            public string ExpansionBoard => "EXG_FROM_PROP";
        }


        /// <summary>
        /// No method; primary property present.
        /// </summary>
        /// <returns>"IMU_FROM_PROP" via <c>ExpansionBoard</c> property.</returns>
        class OnlyExpansionBoardProp
        {
            public string ExpansionBoard => "IMU_FROM_PROP";
        }


        /// <summary>
        /// No method; <c>ExpansionBoard</c> is empty, <c>ExpansionBoardID</c> is set.
        /// </summary>
        /// <returns>"EXG_ID_123" via <c>ExpansionBoardID</c> property.</returns>
        class ExpansionBoardIdProp
        {
            public string ExpansionBoard => "   ";
            public string ExpansionBoardID => "EXG_ID_123";
        }


        /// <summary>
        /// No method; first two properties are empty, <c>DaughterCardID</c> is set.
        /// </summary>
        /// <returns>"IMU_DAUGHTER" via <c>DaughterCardID</c> property.</returns>
        class DaughterCardProp
        {
            public string? ExpansionBoard => null;
            public string ExpansionBoardID => "";
            public string DaughterCardID => "IMU_DAUGHTER";
        }


        /// <summary>
        /// No method and all properties empty/null.
        /// </summary>
        /// <returns><c>null</c></returns>
        class NothingUseful
        {
            public string? ExpansionBoard => "   ";
            public string? ExpansionBoardID => null;
            public string DaughterCardID => "";
        }


        // ----- tests -----

        /// <summary>
        /// Method path: value comes from the parameterless method.
        /// Expected: "EXG_FROM_METHOD".
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_MethodReturnsString()
        {
            var t = new MethodReturnsString();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Equal("EXG_FROM_METHOD", s);
        }


        /// <summary>
        /// Method path: non-string return should be converted via ToString().
        /// Expected: "42".
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_MethodReturnsNonString_UsesToString()
        {
            var t = new MethodReturnsNonString();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Equal("42", s);
        }


        /// <summary>
        /// Method throws: helper should fall back to ExpansionBoard property.
        /// Expected: "EXG_FROM_PROP".
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_MethodThrows_FallsBackToProperty()
        {
            var t = new MethodThrowsThenProp();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Equal("EXG_FROM_PROP", s);
        }


        /// <summary>
        /// No method present: helper should read ExpansionBoard.
        /// Expected: "IMU_FROM_PROP".
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_NoMethod_UsesExpansionBoardProp()
        {
            var t = new OnlyExpansionBoardProp();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Equal("IMU_FROM_PROP", s);
        }


        /// <summary>
        /// ExpansionBoard empty/whitespace: helper should fall back to ExpansionBoardID.
        /// Expected: "EXG_ID_123".
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_UsesExpansionBoardID_WhenExpansionBoardEmpty()
        {
            var t = new ExpansionBoardIdProp();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Equal("EXG_ID_123", s);
        }


        /// <summary>
        /// Both ExpansionBoard and ExpansionBoardID unusable: fall back to DaughterCardID.
        /// Expected: "IMU_DAUGHTER".
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_UsesDaughterCardID_AsLastFallback()
        {
            var t = new DaughterCardProp();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Equal("IMU_DAUGHTER", s);
        }


        /// <summary>
        /// All sources are empty/null: nothing to return.
        /// Expected: null.
        /// </summary>
        [Fact]
        public void GetStringNoArgIfExists_AllEmptyOrNull_ReturnsNull()
        {
            var t = new NothingUseful();
            var s = Call_GetStringNoArgIfExists(t, "GetExpansionBoard");
            Assert.Null(s);
        }


        // ----- SafeDelay behavior -----


        /// <summary>
        /// Helper: resolves the private static ShimmerBoardDetector.SafeDelay method.
        /// </summary>
        /// <returns>The <see cref="MethodInfo"/> for the private static <c>SafeDelay</c> method.</returns>
        static MethodInfo MI_SafeDelay()
        {
            var tDet = typeof(ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(tDet);

            var mi = tDet!.GetMethod("SafeDelay",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Helper: invokes SafeDelay(ms) with the provided milliseconds.
        /// </summary>
        static void Call_SafeDelay(int ms)
        {
            MI_SafeDelay().Invoke(null, new object[] { ms });
        }


        /// <summary>
        /// Verifies SafeDelay handles negative values without throwing and returns quickly.
        /// Expected: no exception and elapsed time is well under 100 ms (no real sleep).
        /// </summary>
        [Fact]
        public void SafeDelay_Negative_NoThrow_AndFast()
        {
            var sw = Stopwatch.StartNew();
            var ex = Record.Exception(() => Call_SafeDelay(-5));
            sw.Stop();

            Assert.Null(ex);
            // Should return immediately (no actual sleep)
            Assert.True(sw.ElapsedMilliseconds < 100);
        }


        /// <summary>
        /// Verifies SafeDelay(0) does not throw and returns quickly.
        /// Expected: no exception and elapsed time is well under 100 ms (no real sleep).
        /// </summary>
        [Fact]
        public void SafeDelay_Zero_NoThrow_AndFast()
        {
            var sw = Stopwatch.StartNew();
            var ex = Record.Exception(() => Call_SafeDelay(0));
            sw.Stop();

            Assert.Null(ex);
            Assert.True(sw.ElapsedMilliseconds < 100);
        }


        /// <summary>
        /// Verifies SafeDelay with a small positive value performs an actual delay (roughly).
        /// Expected: no exception; elapsed time >= ~15 ms and &lt; 500 ms to allow wide tolerance.
        /// </summary>
        [Fact]
        public void SafeDelay_Positive_RoughlySleeps()
        {
            const int requested = 20; // ms
            var sw = Stopwatch.StartNew();
            var ex = Record.Exception(() => Call_SafeDelay(requested));
            sw.Stop();

            Assert.Null(ex);

            // Wide tolerance: at least ~15ms, but not absurdly long
            Assert.True(sw.ElapsedMilliseconds >= 15, $"Slept only {sw.ElapsedMilliseconds}ms");
            Assert.True(sw.ElapsedMilliseconds < 500, $"Slept too long: {sw.ElapsedMilliseconds}ms");
        }


        // ----- InvokeNoArgIfExists behavior -----


        /// <summary>
        /// Helper: resolves the private static ShimmerBoardDetector.InvokeNoArgIfExists method.
        /// </summary>
        /// <returns>The <see cref="MethodInfo"/> for the private static <c>InvokeNoArgIfExists</c> method.</returns>
        static MethodInfo MI_InvokeNoArgIfExists()
        {
            var tDet = typeof(ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(tDet);

            var mi = tDet!.GetMethod("InvokeNoArgIfExists",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Helper: invokes InvokeNoArgIfExists(instance, methodName) and returns the result.
        /// </summary>
        /// <returns>The invocation result, or <c>null</c> if the method is missing or throws.</returns>
        static object? Call_InvokeNoArgIfExists(object instance, string methodName)
        {
            return MI_InvokeNoArgIfExists().Invoke(null, new object?[] { instance, methodName });
        }


        // ----- test doubles (fake targets) for the tests -----

        /// <summary>
        /// Test double exposing a public parameterless method that succeeds.
        /// Used to verify that InvokeNoArgIfExists can find and invoke a no-arg method
        /// and return its value ("OK") without errors.
        /// </summary>
        private class HasParamlessOk
        {
            public string Hello() => "OK";
        }


        /// <summary>
        /// Test double exposing a method with a parameter (no parameterless overload).
        /// Used to verify that InvokeNoArgIfExists ignores methods that require arguments
        /// and returns null instead of attempting invocation.
        /// </summary>
        private class HasMethodWithParam
        {
            public string Hello(string name) => "NOPE";
        }


        /// <summary>
        /// Test double whose parameterless method throws when invoked.
        /// Used to verify that InvokeNoArgIfExists swallows invocation exceptions
        /// and returns null instead of propagating the error.
        /// </summary>
        private class ThrowsOnInvoke
        {
            public string Boom() => throw new System.InvalidOperationException("boom");
        }


        // ----- tests -----

        /// <summary>
        /// Calls InvokeNoArgIfExists on a public parameterless method that returns a value.
        /// Expected: the actual return value ("OK") is returned (not null).
        /// </summary>
        [Fact]
        public void InvokeNoArgIfExists_ParamlessMethod_ReturnsValue()
        {
            var target = new HasParamlessOk();
            var res = Call_InvokeNoArgIfExists(target, "Hello");
            Assert.Equal("OK", res);
        }


        /// <summary>
        /// Calls InvokeNoArgIfExists where a method exists but requires parameters.
        /// Expected: method is ignored due to parameters; returns null (no throw).
        /// </summary>
        [Fact]
        public void InvokeNoArgIfExists_MethodWithParam_Ignored_ReturnsNull()
        {
            var target = new HasMethodWithParam();
            var res = Call_InvokeNoArgIfExists(target, "Hello");
            Assert.Null(res);
        }


        /// <summary>
        /// Calls InvokeNoArgIfExists where the target method throws.
        /// Expected: exception is swallowed by the implementation; returns null.
        /// </summary>
        [Fact]
        public void InvokeNoArgIfExists_MethodThrows_ExceptionSwallowed_ReturnsNull()
        {
            var target = new ThrowsOnInvoke();
            var res = Call_InvokeNoArgIfExists(target, "Boom");
            Assert.Null(res);
        }


        /// <summary>
        /// Calls InvokeNoArgIfExists for a missing method name.
        /// Expected: no invocation occurs and null is returned.
        /// </summary>
        [Fact]
        public void InvokeNoArgIfExists_MissingMethod_ReturnsNull()
        {
            var target = new HasParamlessOk();
            var res = Call_InvokeNoArgIfExists(target, "DoesNotExist");
            Assert.Null(res);
        }


        // ----- RefEqComparer class -----


        /// <summary>
        /// Helper: Locates the private nested type <c>ShimmerBoardDetector.RefEqComparer</c>
        /// via reflection so we can instantiate and call it from tests.
        /// </summary>
        /// <returns>The non-public <see cref="Type"/> of <c>RefEqComparer</c>.</returns>
        static Type RefEqComparerType()
        {
            var det = typeof(ShimmerScanManager)
                .GetNestedType("ShimmerBoardDetector", BindingFlags.NonPublic);
            Assert.NotNull(det);

            var t = det!.GetNestedType("RefEqComparer", BindingFlags.NonPublic);
            Assert.NotNull(t);
            return t!;
        }


        /// <summary>
        /// Helper: Creates an instance of the private <c>RefEqComparer</c> using its non-public constructor.
        /// </summary>
        /// <returns>A new comparer instance of the private <c>RefEqComparer</c> type.</returns>
        static object NewRefEqComparer()
        {
            var t = RefEqComparerType();
            var inst = Activator.CreateInstance(t, nonPublic: true);
            Assert.NotNull(inst);
            return inst!;
        }


        /// <summary>
        /// Helper: Invokes <c>IEqualityComparer.Equals(object, object)</c> on the comparer via reflection.
        /// </summary>
        /// <returns><c>true</c> if <paramref name="x"/> and <paramref name="y"/> are the same reference; otherwise <c>false</c>.</returns>
        static bool RefEqEquals(object cmp, object? x, object? y)
        {
            var t = RefEqComparerType();
            var mi = t.GetMethod(
                "Equals",
                BindingFlags.Public | BindingFlags.Instance,
                binder: null,
                types: new[] { typeof(object), typeof(object) },
                modifiers: null
            );
            Assert.NotNull(mi);
            return (bool)mi!.Invoke(cmp, new object?[] { x, y })!;
        }


        /// <summary>
        /// Helper: Invokes <c>IEqualityComparer.GetHashCode(object)</c> on the comparer via reflection.
        /// </summary>
        /// <returns>A hash code derived from the object's reference identity.</returns>
        static int RefEqGetHashCode(object cmp, object obj)
        {
            var t = RefEqComparerType();
            var mi = t.GetMethod(
                "GetHashCode",
                BindingFlags.Public | BindingFlags.Instance,
                binder: null,
                types: new[] { typeof(object) },
                modifiers: null
            );
            Assert.NotNull(mi);
            return (int)mi!.Invoke(cmp, new object[] { obj })!;
        }


        /// <summary>
        /// Verifies that two references pointing to the same instance are considered equal.
        /// Expected: <c>true</c>.
        /// </summary>
        [Fact]
        public void RefEqComparer_SameReference_IsEqual()
        {
            var cmp = NewRefEqComparer();
            var o = new object();

            Assert.True(RefEqEquals(cmp, o, o));
        }


        /// <summary>
        /// Verifies that two different instances are not equal even if their contents are identical.
        /// Expected: <c>false</c>.
        /// </summary>
        [Fact]
        public void RefEqComparer_DifferentInstances_NotEqual_EvenIfStructurallyEqual()
        {
            var cmp = NewRefEqComparer();

            // Two arrays with the same content but different references
            var a = new byte[] { 1, 2, 3 };
            var b = new byte[] { 1, 2, 3 };

            Assert.False(RefEqEquals(cmp, a, b));
        }


        /// <summary>
        /// Verifies that comparing <c>null</c> vs <c>null</c> is treated as equal by reference equality.
        /// Expected: <c>true</c>.
        /// </summary>
        [Fact]
        public void RefEqComparer_NullVsNull_IsEqual()
        {
            var cmp = NewRefEqComparer();
            Assert.True(RefEqEquals(cmp, null, null));
        }


        /// <summary>
        /// Verifies that <c>null</c> compared to a non-null object is not equal (both directions).
        /// Expected: <c>false</c>.
        /// </summary>
        [Fact]
        public void RefEqComparer_NullVsObject_NotEqual()
        {
            var cmp = NewRefEqComparer();
            Assert.False(RefEqEquals(cmp, null, new object()));
            Assert.False(RefEqEquals(cmp, new object(), null));
        }


        /// <summary>
        /// Verifies that the hash code is stable for the same object when computed twice.
        /// Expected: the two hash codes are equal.
        /// </summary>
        [Fact]
        public void RefEqComparer_GetHashCode_SameObject_SameCode()
        {
            var cmp = NewRefEqComparer();
            var o = new object();

            var h1 = RefEqGetHashCode(cmp, o);
            var h2 = RefEqGetHashCode(cmp, o);

            Assert.Equal(h1, h2);
        }


        /// <summary>
        /// Verifies that the comparer makes a <c>HashSet&lt;object&gt;</c> behave by reference:
        /// the same reference is a duplicate, a different instance with same content is distinct.
        /// Expected: set contains exactly two elements (a1 once, a2 once).
        /// </summary>
        [Fact]
        public void RefEqComparer_WorksInHashSet_ByReference()
        {
            var cmp = NewRefEqComparer();

            // Build a HashSet<object> that uses our comparer (via reflection-created instance)
            var hsType = typeof(HashSet<object>);
            var ctor = hsType.GetConstructor(new[] { typeof(IEqualityComparer<object>) });
            Assert.NotNull(ctor);

            var hs = (HashSet<object>)ctor!.Invoke(new object[] { cmp });

            var a1 = new byte[] { 42 };
            var a2 = new byte[] { 42 };     // same content, different reference

            Assert.True(hs.Add(a1));
            Assert.False(hs.Add(a1));       // same reference -> duplicate
            Assert.True(hs.Add(a2));        // different reference -> accepted

            Assert.Equal(2, hs.Count);
        }
    }
}
