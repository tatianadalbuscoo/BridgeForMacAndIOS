namespace Android.Bluetooth
{
    public class BluetoothDevice
    {
        public string? Name { get; set; }
        public string? Address { get; set; }
        public const string ActionFound = "BT_DEVICE_FOUND";
        public const string ExtraDevice = "EXTRA_DEVICE";
        public const string ExtraRssi = "EXTRA_RSSI";
    }

    public class BluetoothAdapter
    {
        public static BluetoothAdapter? DefaultAdapter => new BluetoothAdapter();
        public bool IsEnabled => false;
        public bool IsDiscovering => false;
        public void StartDiscovery() { }
        public void CancelDiscovery() { }
        public ISet<BluetoothDevice> BondedDevices { get; } = new HashSet<BluetoothDevice>();
        public const string ActionDiscoveryFinished = "BT_DISCOVERY_FINISHED";
        public static bool CheckBluetoothAddress(string mac) => true;
    }
}

namespace Android.Content
{
    public class Intent
    {
        public string? Action { get; set; }
        public bool HasExtra(string name) => false;
        public short GetShortExtra(string name, short defaultValue) => defaultValue;
        public object? GetParcelableExtra(string name) => null;
    }

    public class IntentFilter : System.IDisposable
    {
        public void AddAction(string action) { }
        public void Dispose() { /* niente */ }
    }

    public class Context { }

    public abstract class BroadcastReceiver
    {
        public abstract void OnReceive(Context? context, Intent? intent);
    }
}

namespace Android.App
{
    public class Activity
    {
        public void RegisterReceiver(Android.Content.BroadcastReceiver r, Android.Content.IntentFilter f) { }
        public void UnregisterReceiver(Android.Content.BroadcastReceiver r) { }
    }
}

// Tipo globale per soddisfare gli usi non qualificati di "Activity" nel file originale.
// Non ha namespace apposta.
public class Activity
{
    public void RegisterReceiver(Android.Content.BroadcastReceiver r, Android.Content.IntentFilter f) { }
    public void UnregisterReceiver(Android.Content.BroadcastReceiver r) { }
}
