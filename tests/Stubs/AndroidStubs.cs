/*
 * AndroidStubs.cs
 * These types mimic a tiny subset of Android + Shimmer SDK APIs so we can
 * compile and unit-test logic on .NET (without Android runtime or devices).
 */

#if TEST_STUBS


global using Activity = Android.App.Activity;
using System.Net.WebSockets;
using System.Text;


namespace Android.Bluetooth
{

    /// <summary>
    /// Minimal stand-in for <c>Android.Bluetooth.BluetoothDevice</c>.
    /// Name/Address are enough for tests (classification and OUI checks).
    /// </summary>
    public class BluetoothDevice
    {

        // Intent action for "device found" used by production code.
        public const string ActionFound = "BT_DEVICE_FOUND";

        // Intent extra key for the device payload.
        public const string ExtraDevice = "EXTRA_DEVICE";

        // Intent extra key for the RSSI payload.
        public const string ExtraRssi = "EXTRA_RSSI";


        /// <summary>
        /// Bluetooth device name.
        /// </summary>
        /// <value>The advertised device name; may be <c>null</c> in tests.</value>
        public string? Name { get; set; }


        /// <summary>Bluetooth MAC address.</summary>
        /// <value>
        /// String MAC (may be <c>null</c> in tests).
        /// </value>
        public string? Address { get; set; }
    }


    /// <summary>
    /// Minimal stand-in for <c>Android.Bluetooth.BluetoothAdapter</c>.
    /// Uses singleton semantics via <see cref="DefaultAdapter"/> to mirror the real API.
    /// </summary>
    public class BluetoothAdapter
    {
        // Single shared instance so tests and code under test see the same state.
        private static BluetoothAdapter? _instance;

        // Singleton accessor that returns the shared adapter instance.
        public static BluetoothAdapter? DefaultAdapter => _instance ??= new BluetoothAdapter();

        // Intent action raised when discovery finishes
        public const string ActionDiscoveryFinished = "BT_DISCOVERY_FINISHED";


        /// <summary>
        /// Indicates whether "Bluetooth is enabled" in tests (toggle to simulate radio availability).
        /// </summary>
        /// <value><c>true</c> if enabled; otherwise <c>false</c>.</value>
        public bool IsEnabled { get; set; } = false;


        /// <summary>
        /// Discovery lifecycle is tracked for assertions (Start/Cancel).
        /// No actual device discovery happens in stubs.
        /// </summary>
        /// <value><c>true</c> while discovery is simulated as running; otherwise <c>false</c>.</value>
        public bool IsDiscovering { get; private set; } = false;


        /// <summary>
        /// Simulates starting discovery by flipping <see cref="IsDiscovering"/> to <c>true</c>.
        /// </summary>
        public void StartDiscovery() { IsDiscovering = true; }


        /// <summary>
        /// Simulates stopping discovery by flipping <see cref="IsDiscovering"/> to <c>false</c>.
        /// </summary>
        public void CancelDiscovery() { IsDiscovering = false; }


        /// <summary>
        /// Represents already paired devices. Tests can add/remove entries freely.
        /// </summary>
        /// <value>The mutable set of paired devices.</value>
        public ISet<BluetoothDevice> BondedDevices { get; } = new HashSet<BluetoothDevice>();


        /// <summary>
        /// Very permissive address validator for tests.
        /// </summary>
        /// <param name="mac">MAC address string.</param>
        /// <returns><c>true</c> if <paramref name="mac"/> is non-empty; otherwise <c>false</c>.</returns>
        public static bool CheckBluetoothAddress(string mac) =>
            !string.IsNullOrWhiteSpace(mac);
    }
}


namespace Android.Content
{

    /// <summary>
    /// Minimal Intent stub.
    /// Methods are <c>virtual</c> so tests can subclass (e.g., TestIntent) and override
    /// <see cref="HasExtra"/>, <see cref="GetParcelableExtra"/>, <see cref="GetShortExtra"/> to simulate payloads.
    /// </summary>
    public class Intent
    {

        /// <summary>
        /// Gets or sets the intent action.
        /// </summary>
        /// <value>The action string; may be <c>null</c>.</value>
        public string? Action { get; set; }


        /// <summary>
        /// Checks for an extra by key.
        /// </summary>
        /// <param name="name">Extra name.</param>
        /// <returns><c>true</c> if present; otherwise <c>false</c>.</returns>
        public virtual bool HasExtra(string name) => false;


        /// <summary>
        /// Reads a 16-bit extra value.
        /// </summary>
        /// <param name="name">Extra name.</param>
        /// <param name="defaultValue">Fallback value.</param>
        /// <returns>The stored extra or <paramref name="defaultValue"/>.</returns>
        public virtual short GetShortExtra(string name, short defaultValue) => defaultValue;


        /// <summary>
        /// Reads a parcelable extra value.
        /// </summary>
        /// <param name="name">Extra name.</param>
        /// <returns>The stored extra object or <c>null</c>.</returns>
        public virtual object? GetParcelableExtra(string name) => null;

    }


    /// <summary>
    /// No-op container to match <c>RegisterReceiver</c> signature in code under test.
    /// </summary>
    public class IntentFilter : System.IDisposable
    {

        /// <summary>
        /// Adds an action to the filter (no-op).
        /// </summary>
        /// <param name="action">Action to add.</param>
        public void AddAction(string action) {}


        /// <summary>
        /// Disposes the filter (no-op).
        /// </summary>
        public void Dispose() {}
    }


    /// <summary>
    /// Placeholder for Android context (never used by tests).
    /// </summary>
    public class Context
    {

        /// <summary>Gets a system service by name.</summary>
        /// <param name="name">Service name.</param>
        /// <returns>The service instance or <c>null</c>.</returns>
        public virtual object? GetSystemService(string name) => null;
    }


    /// <summary>
    /// Base type for broadcast receivers. Tests invoke <see cref="OnReceive"/> directly.
    /// </summary>
    public abstract class BroadcastReceiver
    {

        /// <summary>Handles an incoming broadcast.</summary>
        /// <param name="context">Sender context (may be <c>null</c> in tests).</param>
        /// <param name="intent">Intent payload (may be <c>null</c>).</param>
        public abstract void OnReceive(Context? context, Intent? intent);
    }
}


namespace Android.App
{

    /// <summary>
    /// Very small stand-in for <c>Android.App.Activity</c>.
    /// Register/Unregister are no-ops; tests call receivers manually.
    /// Also exposes <see cref="ApplicationContext"/> and <c>WifiService</c>.
    /// </summary>
    public class Activity : Android.Content.Context
    {

        // Name of the Wi-Fi service
        public const string WifiService = "wifi";

        private readonly Android.Net.Wifi.WifiManager _wifiManager = new Android.Net.Wifi.WifiManager();

        /// <summary>
        /// Gets the application context.
        /// </summary>
        /// <value>The same instance (self).</value>
        public Android.Content.Context ApplicationContext => this;


        /// <summary>
        /// Resolves a system service.
        /// </summary>
        /// <param name="name">Service name.</param>
        /// <returns>The Wi-Fi manager for <c>"wifi"</c>; otherwise <c>null</c>.</returns>
        public override object? GetSystemService(string name)
        {
            if (name == WifiService) return _wifiManager;
            return null;
        }


        /// <summary>
        /// Registers a receiver (no-op).
        /// </summary>
        public void RegisterReceiver(Android.Content.BroadcastReceiver r, Android.Content.IntentFilter f) { }


        /// <summary>
        /// Unregisters a receiver (no-op).
        /// </summary>
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

        /// <summary>
        /// Gets or sets the current Wi-Fi connection info.
        /// </summary>
        /// <value>An instance with a test IPv4 address.</value>
        public WifiInfo? ConnectionInfo { get; set; } = new WifiInfo();
    }


    /// <summary>
    /// Minimal Wi-Fi connection info with an IPv4 address in Android's int form.
    /// </summary>
    public class WifiInfo
    {

        /// <summary>
        /// Gets or sets the IPv4 address in little-endian Android format.
        /// </summary>
        /// <value>The packed IPv4 address as <see cref="int"/>.</value>
        public int IpAddress { get; set; } = (42 << 24) | (1 << 16) | (168 << 8) | 192;
    }
}


namespace Java.Lang
{

    /// <summary>
    /// Tiny placeholder for <c>Java.Lang.Integer</c> used by IsConnectedState.
    /// </summary>
    public class Integer
    {
        private readonly int _v;

        /// <summary>
        /// Initializes a new instance wrapping <paramref name="v"/>.
        /// </summary>
        public Integer(int v) 
        {
            _v = v; 
        }


        /// <summary>
        /// Returns the wrapped integer value.
        /// </summary>
        /// <returns>The integer value passed to the constructor.</returns>
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

        /// <summary>
        /// Initializes with a client <paramref name="id"/>.
        /// </summary>
        public WatsonWsClient(Guid id) 
        {
            Guid = id; 
        }


        /// <summary>
        /// Gets the client identifier.
        /// </summary>
        /// <value>The client <see cref="Guid"/>.</value>
        public Guid Guid { get; }
    }


    /// <summary>
    /// Event args for a newly connected client.
    /// </summary>
    public class ClientConnectedEventArgs : EventArgs
    {

        /// <summary>
        /// Initializes with a <paramref name="c"/>.
        /// </summary>
        public ClientConnectedEventArgs(WatsonWsClient c) 
        { 
            Client = c; 
        }


        /// <summary>
        /// Gets the connected client.
        /// </summary>
        /// <value>The <see cref="WatsonWsClient"/> instance.</value>
        public WatsonWsClient Client 
        { 
            get; 
        }
    }


    /// <summary>
    /// Event args for a disconnected client.
    /// </summary>
    public class ClientDisconnectedEventArgs : EventArgs
    {

        /// <summary>
        /// Initializes with a <paramref name="c"/>.
        /// </summary>
        public ClientDisconnectedEventArgs(WatsonWsClient c) 
        { 
            Client = c; 
        }


        /// <summary>
        /// Gets the disconnected client.
        /// </summary>
        /// <value>The <see cref="WatsonWsClient"/> instance.</value>
        public WatsonWebsocket.WatsonWsClient Client 
        { 
            get; 
        }
    }


    /// <summary>
    /// Event args for a received message.
    /// </summary>
    public class MessageReceivedEventArgs : EventArgs
    {

        /// <summary>
        /// Initializes a new instance.
        /// </summary>
        public MessageReceivedEventArgs(WatsonWsClient client, WebSocketMessageType mt, ArraySegment<byte> data)
        {
            Client = client; MessageType = mt; Data = data;
        }


        /// <summary>
        /// Gets the sending client.
        /// </summary>
        /// <value>The <see cref="WatsonWsClient"/>.</value>
        public WatsonWsClient Client 
        { 
            get; 
        }


        /// <summary>
        /// Gets the message type.
        /// </summary>
        /// <value>The <see cref="WebSocketMessageType"/>.</value>
        public WebSocketMessageType MessageType 
        { 
            get;
        }


        /// <summary>
        /// Gets the message payload.
        /// </summary>
        /// <value>The data buffer segment.</value>
        public ArraySegment<byte> Data { get; }
    }


    /// <summary>
    /// Very small server stub that records sends and exposes connect/disconnect/message events.
    /// </summary>
    public class WatsonWsServer : IDisposable
    {

        /// <summary>
        /// Global log of sent messages (for assertions).
        /// </summary>
        public static List<(Guid clientId, string message)> SentLog { get; } = new();


        /// <summary>
        /// Clears the global sent log.
        /// </summary>
        public static void ClearSentLog() => SentLog.Clear();


        /// <summary>
        /// Creates a new server stub.
        /// </summary>
        public WatsonWsServer(string ip, int port, bool ssl) {}


        /// <summary>
        /// Gets whether the server is "listening".
        /// </summary>
        /// <value><c>true</c> after <see cref="Start"/>; otherwise <c>false</c>.</value>
        public bool IsListening { get; private set; }


        /// <summary>
        /// Raised when a client connects.
        /// </summary>
        public event EventHandler<ClientConnectedEventArgs>? ClientConnected;


        /// <summary>
        /// Raised when a client disconnects.
        /// </summary>
        public event EventHandler<ClientDisconnectedEventArgs>? ClientDisconnected;


        /// <summary>
        /// Raised when a message is received.
        /// </summary>
        public event EventHandler<MessageReceivedEventArgs>? MessageReceived;


        /// <summary>
        /// Starts the server (no-op beyond setting <see cref="IsListening"/>).
        /// </summary>
        public void Start() => IsListening = true;


        /// <summary>
        /// Stops the server (no-op beyond clearing <see cref="IsListening"/>).
        /// </summary>
        public void Stop() => IsListening = false;


        /// <summary>
        /// Disposes server resources (no-op).
        /// </summary>
        public void Dispose() {}


        /// <summary>
        /// Records a sent text message.
        /// </summary>
        /// <param name="clientId">Recipient client id.</param>
        /// <param name="message">Message body.</param>
        /// <returns>A completed task.</returns>
        public System.Threading.Tasks.Task SendAsync(Guid clientId, string message)
        {
            Sent.Add((clientId, message));
            return System.Threading.Tasks.Task.CompletedTask;
        }


        /// <summary>
        /// Simulates a client connection event.
        /// </summary>
        /// <param name="id">Client id.</param>
        public void RaiseConnected(Guid id) =>
            ClientConnected?.Invoke(this, new ClientConnectedEventArgs(new WatsonWsClient(id)));


        /// <summary>
        /// Simulates a client disconnection event.
        /// </summary>
        /// <param name="id">Client id.</param>
        public void RaiseDisconnected(Guid id) =>
            ClientDisconnected?.Invoke(this, new ClientDisconnectedEventArgs(new WatsonWsClient(id)));


        /// <summary>
        /// Local per-instance send log (for convenience).
        /// </summary>
        public readonly System.Collections.Generic.List<(Guid clientId, string message)> Sent
            = new System.Collections.Generic.List<(Guid clientId, string message)>();


        /// <summary>
        /// Simulates receiving a text message.
        /// </summary>
        /// <param name="id">Client id.</param>
        /// <param name="text">Text message.</param>
        public void RaiseText(Guid id, string text) =>
            MessageReceived?.Invoke(
                this,
                new MessageReceivedEventArgs(
                    new WatsonWsClient(id),
                    WebSocketMessageType.Text,
                    new ArraySegment<byte>(Encoding.UTF8.GetBytes(text))
                )
            );


        /// <summary>
        /// Simulates receiving a binary message.
        /// </summary>
        /// <param name="id">Client id.</param>
        /// <param name="payload">Binary payload.</param>
        public void RaiseBinary(Guid id, byte[] payload) =>
        MessageReceived?.Invoke(
            this,
            new MessageReceivedEventArgs(
                new WatsonWsClient(id),
                WebSocketMessageType.Binary,
                new ArraySegment<byte>(payload)
            )
        );

    }
}


namespace ShimmerSDK.Android
{

    /// <summary>
    /// Test double for <c>ShimmerLogAndStreamAndroidBluetoothV2</c>.
    /// It exposes a controllable connection state and an <c>ExpansionTarget</c> object
    /// that the production reflection-based probing can discover.
    /// </summary>
    public class ShimmerLogAndStreamAndroidBluetoothV2
    {

        /// <summary>
        /// Gets the device name passed at construction.
        /// </summary>
        /// <value>The Bluetooth device name.</value>
        public string DeviceName { get; }


        /// <summary>
        /// Gets the MAC address passed at construction.
        /// </summary>
        /// <value>The Bluetooth MAC address.</value>
        public string Mac { get; }


        /// <summary>
        /// Flag toggled by <see cref="Connect"/>/<see cref="Disconnect"/>.
        /// </summary>
        /// <value><c>true</c> if connected; otherwise <c>false</c>.</value>
        public bool Connected { get; private set; }


        /// <summary>
        /// Object that the reflection-based BFS should find to read board info.
        /// </summary>
        /// <value>Arbitrary test object graph node.</value>
        public object? ExpansionTarget { get; set; }


        /// <summary>
        /// Constructs a new stub session.
        /// </summary>
        /// <param name="deviceName">Device name.</param>
        /// <param name="mac">Device MAC.</param>
        public ShimmerLogAndStreamAndroidBluetoothV2(string deviceName, string mac)
        {
            DeviceName = deviceName;
            Mac = mac;
        }


        /// <summary>
        /// Marks the session as connected and raises a "connected" UI callback.
        /// Throw here in tests to simulate failures if needed.
        /// </summary>
        public void Connect()
        {
            Connected = true;

            var evt = new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_STATE_CHANGE,
                2 // "connected"
            );

            UICallback?.Invoke(this, evt);
        }


        /// <summary>
        /// Returns the current connection state.
        /// </summary>
        /// <returns><c>true</c> if connected; otherwise <c>false</c>.</returns>
        public bool IsConnected() => Connected;


        /// <summary>
        /// Marks the session as disconnected.
        /// </summary>
        public void Disconnect() => Connected = false;


        // ----- Additions required by production code (no-op implementations) -----


        /// <summary>
        /// Event used by the production code to observe state/data callbacks.
        /// Tests can attach/detach handlers; the stub doesn’t raise by itself.
        /// </summary>
        public event EventHandler? UICallback;


        /// <summary>
        /// Sets the sampling rate (no-op).
        /// </summary>
        /// <param name="hz">Requested sampling rate in Hz.</param>
        public void WriteSamplingRate(int hz) { /* no-op for stub */ }


        /// <summary>
        /// Writes the sensor bitmap (no-op).
        /// </summary>
        /// <param name="sensorBitmap">Bitmask of sensors.</param>
        public void WriteSensors(int sensorBitmap) { /* no-op for stub */ }


        /// <summary>
        /// Triggers a device inquiry/refresh (no-op).
        /// </summary>
        public void Inquiry() { /* no-op for stub */ }


        /// <summary>
        /// Reads calibration parameters (no-op).
        /// </summary>
        /// <param name="scope">Parameter scope.</param>
        public void ReadCalibrationParameters(string scope) { /* no-op for stub */ }


        /// <summary>
        /// Starts streaming (no-op).
        /// </summary>
        public void StartStreaming() { /* no-op for stub */ }


        /// <summary>
        /// Stops streaming (no-op).
        /// </summary>
        public void StopStreaming() { /* no-op for stub */ }


        /// <summary>
        /// Lets tests manually raise a UI callback.
        /// </summary>
        /// <param name="e">Event payload.</param>
        public void RaiseUi(EventArgs e) => UICallback?.Invoke(this, e);
    }
}

namespace ShimmerBridgeScan
{

    /// <summary>
    /// Extremely small detector stub whose API shape matches production code.
    /// Always returns <c>false</c> so the caller takes fallback paths in tests.
    /// </summary>
    public static class ShimmerBoardDetector
    {

        /// <summary>
        /// Board kind enum used by the stub.
        /// </summary>
        public enum BoardKind { Unknown = 0, EXG = 1, IMU = 2 }


        /// <summary>
        /// Tries to detect the board kind; the stub never succeeds.
        /// </summary>
        /// <param name="_core">Ignored by the stub.</param>
        /// <param name="kind">Set to <see cref="BoardKind.Unknown"/>.</param>
        /// <param name="rawId">Set to <c>"STUB"</c>.</param>
        /// <returns>Always <c>false</c>.</returns>
        public static bool TryDetectBoardKind(object _core, out BoardKind kind, out string rawId)
        {
            kind = BoardKind.Unknown;
            rawId = "STUB";
            return false;
        }
    }
}

#endif
