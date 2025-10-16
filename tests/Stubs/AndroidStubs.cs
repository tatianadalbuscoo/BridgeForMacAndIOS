#if TEST_STUBS

// -----------------------------------------------------------------------------
// TEST STUBS
// These types mimic a tiny subset of Android + Shimmer SDK APIs so we can
// compile and unit-test logic on .NET (without Android runtime or devices).
// -----------------------------------------------------------------------------

namespace Android.Bluetooth
{

    /// <summary>
    /// Minimal stand-in for Android.Bluetooth.BluetoothDevice.
    /// Name/Address are enough for tests (classification and OUI check).
    /// </summary>
    public class BluetoothDevice
    {
        public string? Name { get; set; }
        public string? Address { get; set; }

        // Intent action / extra names used by the production code.
        public const string ActionFound = "BT_DEVICE_FOUND";
        public const string ExtraDevice = "EXTRA_DEVICE";
        public const string ExtraRssi = "EXTRA_RSSI";
    }

    /// <summary>
    /// Minimal stand-in for Android.Bluetooth.BluetoothAdapter.
    /// Singleton semantics via DefaultAdapter to reflect the real API.
    /// </summary>
    public class BluetoothAdapter
    {

        // Single shared instance so tests and code under test see the same state.
        private static BluetoothAdapter? _instance;
        public static BluetoothAdapter? DefaultAdapter => _instance ??= new BluetoothAdapter();

        /// <summary>
        /// Toggle in tests to simulate Bluetooth radio availability.
        /// </summary>
        public bool IsEnabled { get; set; } = false;

        /// <summary>
        /// Discovery lifecycle is tracked for assertions (Start/Cancel).
        /// No actual device discovery happens in stubs.
        /// </summary>
        public bool IsDiscovering { get; private set; } = false;
        public void StartDiscovery() { IsDiscovering = true; }
        public void CancelDiscovery() { IsDiscovering = false; }


        /// <summary>
        /// Represents already paired devices. Tests can add/remove entries freely.
        /// </summary>
        public ISet<BluetoothDevice> BondedDevices { get; } = new HashSet<BluetoothDevice>();

        public const string ActionDiscoveryFinished = "BT_DISCOVERY_FINISHED";

        /// <summary>
        /// Very permissive address validator for tests.
        /// </summary>
        public static bool CheckBluetoothAddress(string mac) =>
            !string.IsNullOrWhiteSpace(mac);
    }
}

namespace Android.Content
{

    /// <summary>
    /// Minimal Intent stub.
    /// Methods are virtual so tests can subclass (e.g., TestIntent) and override
    /// HasExtra/GetParcelableExtra/GetShortExtra to simulate payloads.
    /// </summary>
    public class Intent
    {
        public string? Action { get; set; }

        public virtual bool HasExtra(string name) => false;
        public virtual short GetShortExtra(string name, short defaultValue) => defaultValue;
        public virtual object? GetParcelableExtra(string name) => null;
    }

    /// <summary>
    /// No-op container to match RegisterReceiver signature in code under test.
    /// </summary>
    public class IntentFilter : System.IDisposable
    {
        public void AddAction(string action) { }
        public void Dispose() { /* no-op */ }
    }

    /// <summary>
    /// Placeholder for Android context (never used by tests).
    /// </summary>
    public class Context { }

    /// <summary>
    /// Base type for broadcast receivers. Tests invoke OnReceive directly.
    /// </summary>
    public abstract class BroadcastReceiver
    {
        public abstract void OnReceive(Context? context, Intent? intent);
    }
}

/// <summary>
/// Very small stand-in for Android.App.Activity.
/// Register/Unregister are no-ops; tests call receivers manually.
/// </summary>
public class Activity
{
    public void RegisterReceiver(Android.Content.BroadcastReceiver r, Android.Content.IntentFilter f) { }
    public void UnregisterReceiver(Android.Content.BroadcastReceiver r) { }
}

/// <summary>
/// Test double for ShimmerLogAndStreamAndroidBluetoothV2.
/// It exposes a controllable connection state and an ExpansionTarget object
/// that the production reflection-based probing can discover.
///
/// Usage in tests:
///   var shim = new ShimmerLogAndStreamAndroidBluetoothV2("dev","AA:BB:CC");
///   shim.Connect(); // or leave disconnected to simulate timeout/guards
///   shim.ExpansionTarget = new {
///       // Provide any of:
///       string ExpansionBoard { get; }  // or ExpansionBoardID / DaughterCardID
///       string GetExpansionBoard()      // optional method
///       void   ReadExpansionBoard()     // optional method
///   };
/// </summary>
namespace ShimmerSDK.Android
{
    public class ShimmerLogAndStreamAndroidBluetoothV2
    {
        public string DeviceName { get; }
        public string Mac { get; }

        // Connection flag the tests can flip via Connect/Disconnect.
        public bool Connected { get; private set; }

        // Object that reflection-based BFS should find to read board info.
        public object? ExpansionTarget { get; set; }

        public ShimmerLogAndStreamAndroidBluetoothV2(string deviceName, string mac)
        {
            DeviceName = deviceName;
            Mac = mac;
        }

        /// <summary>
        /// In the stub, Connect just marks Connected = true.
        /// Throw here in tests to simulate failures if needed.
        /// </summary>
        public void Connect()
        {
            Connected = true;
        }

        public bool IsConnected() => Connected;

        public void Disconnect() { Connected = false; }
    }
}

#endif
