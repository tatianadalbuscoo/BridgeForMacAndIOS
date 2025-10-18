#if TEST_STUBS
// Global alias visible to ALL files compiled by the test project,
// including those linked from the Android project.
global using Activity = Android.App.Activity;
#endif

#if TEST_STUBS

// -----------------------------------------------------------------------------
// TEST STUBS
// These types mimic a tiny subset of Android + Shimmer SDK APIs so we can
// compile and unit-test logic on .NET (without Android runtime or devices).
// -----------------------------------------------------------------------------

using System;
using System.Net.WebSockets;
using System.Text;

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
    public class Context
    {
        public virtual object? GetSystemService(string name) => null;
    }

    /// <summary>
    /// Base type for broadcast receivers. Tests invoke OnReceive directly.
    /// </summary>
    public abstract class BroadcastReceiver
    {
        public abstract void OnReceive(Context? context, Intent? intent);
    }
}

namespace Android.App
{
    /// <summary>
    /// Very small stand-in for Android.App.Activity.
    /// Register/Unregister are no-ops; tests call receivers manually.
    /// Also exposes ApplicationContext + WifiService to satisfy GetLocalIp.
    /// </summary>
    public class Activity : Android.Content.Context
    {
        public Android.Content.Context ApplicationContext => this;
        public const string WifiService = "wifi";

        public override object? GetSystemService(string name)
        {
            if (name == WifiService) return new Android.Net.Wifi.WifiManager();
            return null;
        }

        public void RegisterReceiver(Android.Content.BroadcastReceiver r, Android.Content.IntentFilter f) { }
        public void UnregisterReceiver(Android.Content.BroadcastReceiver r) { }
    }
}

namespace Android.Net.Wifi
{
    /// <summary>
    /// Minimal stand-in for the Wi-Fi manager used to obtain local IP.
    /// </summary>
    public class WifiManager
    {
        public WifiInfo? ConnectionInfo { get; set; } = new WifiInfo();
    }

    /// <summary>
    /// Minimal Wi-Fi connection info with an IPv4 address in Android's int form.
    /// </summary>
    public class WifiInfo
    {
        // 192.168.1.42 expressed in little-endian int like Android does.
        public int IpAddress { get; set; } = (42 << 24) | (1 << 16) | (168 << 8) | 192;
    }
}

namespace Java.Lang
{
    /// <summary>
    /// Tiny placeholder for Java.Lang.Integer used by IsConnectedState.
    /// </summary>
    public class Integer
    {
        private readonly int _v;
        public Integer(int v) { _v = v; }
        public int IntValue() => _v;
    }
}

namespace WatsonWebsocket
{
    /// <summary>
    /// Minimal stubs for Watson WebSocket server to compile and unit test logic.
    /// These do not implement real networking; they only expose events and API shape.
    /// </summary>
    public class WatsonWsClient
    {
        public WatsonWsClient(Guid id) { Guid = id; }
        public Guid Guid { get; }
    }

    public class ClientConnectedEventArgs : EventArgs
    {
        public ClientConnectedEventArgs(WatsonWsClient c) { Client = c; }
        public WatsonWsClient Client { get; }
    }

    public class ClientDisconnectedEventArgs : EventArgs
    {
        public ClientDisconnectedEventArgs(WatsonWsClient c) { Client = c; }
        public WatsonWebsocket.WatsonWsClient Client { get; }
    }

    public class MessageReceivedEventArgs : EventArgs
    {
        public MessageReceivedEventArgs(WatsonWsClient client, WebSocketMessageType mt, ArraySegment<byte> data)
        {
            Client = client; MessageType = mt; Data = data;
        }
        public WatsonWsClient Client { get; }
        public WebSocketMessageType MessageType { get; }
        public ArraySegment<byte> Data { get; }
    }

    public class WatsonWsServer : IDisposable
    {
        public WatsonWsServer(string ip, int port, bool ssl) { }
        public bool IsListening { get; private set; }

        public event EventHandler<ClientConnectedEventArgs>? ClientConnected;
        public event EventHandler<ClientDisconnectedEventArgs>? ClientDisconnected;
        public event EventHandler<MessageReceivedEventArgs>? MessageReceived;

        public void Start() => IsListening = true;
        public void Stop() => IsListening = false;
        public void Dispose() { }

        public System.Threading.Tasks.Task SendAsync(Guid clientId, string message)
            => System.Threading.Tasks.Task.CompletedTask;

        // Helper methods to simulate events during tests.
        public void RaiseConnected(Guid id) =>
            ClientConnected?.Invoke(this, new ClientConnectedEventArgs(new WatsonWsClient(id)));

        public void RaiseDisconnected(Guid id) =>
            ClientDisconnected?.Invoke(this, new ClientDisconnectedEventArgs(new WatsonWsClient(id)));

        public void RaiseText(Guid id, string text) =>
            MessageReceived?.Invoke(
                this,
                new MessageReceivedEventArgs(
                    new WatsonWsClient(id),
                    WebSocketMessageType.Text,
                    new ArraySegment<byte>(Encoding.UTF8.GetBytes(text))
                )
            );
    }
}

/// <summary>
/// Test double for ShimmerLogAndStreamAndroidBluetoothV2.
/// It exposes a controllable connection state and an ExpansionTarget object
/// that the production reflection-based probing can discover.
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
        public void Connect() => Connected = true;

        public bool IsConnected() => Connected;

        public void Disconnect() => Connected = false;

        // ---------------------------------------------------------------------
        // ADDITIONS REQUIRED BY PRODUCTION CODE (no-op implementations)
        // ---------------------------------------------------------------------

        /// <summary>
        /// Event used by the production code to observe state/data callbacks.
        /// Tests can attach/detach handlers; the stub doesn’t raise by itself.
        /// </summary>
        public event EventHandler? UICallback;

        /// <summary>
        /// Sets the sampling rate on the device (no-op in the stub).
        /// </summary>
        public void WriteSamplingRate(int hz) { /* no-op for stub */ }

        /// <summary>
        /// Writes the sensor bitmap (enable/disable sensors) (no-op in the stub).
        /// </summary>
        public void WriteSensors(int sensorBitmap) { /* no-op for stub */ }

        /// <summary>
        /// Triggers a device inquiry/refresh (no-op in the stub).
        /// </summary>
        public void Inquiry() { /* no-op for stub */ }

        /// <summary>
        /// Reads calibration parameters (no-op in the stub).
        /// </summary>
        public void ReadCalibrationParameters(string scope) { /* no-op for stub */ }

        /// <summary>
        /// Starts streaming (no-op in the stub).
        /// </summary>
        public void StartStreaming() { /* no-op for stub */ }

        /// <summary>
        /// Stops streaming (no-op in the stub).
        /// </summary>
        public void StopStreaming() { /* no-op for stub */ }

        // (Optional) Helper to let tests manually raise UI callbacks if ever needed.
        public void RaiseUi(EventArgs e) => UICallback?.Invoke(this, e);
    }
}

#endif
