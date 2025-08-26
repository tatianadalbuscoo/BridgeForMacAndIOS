#if ANDROID
using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Android.Bluetooth;
using Java.Util;
using Java.Lang.Reflect; // Method

namespace XR2Learn_ShimmerAPI.IMU.Android
{
    /// <summary>Connessione RFCOMM SPP verso Shimmer con retry/fallback e timeout.</summary>
    internal sealed class AndroidBluetoothConnection : IShimmerConnection
    {
        private static readonly UUID SPP_UUID =
            UUID.FromString("00001101-0000-1000-8000-00805F9B34FB");

        // serializza i connect per stabilità multi-device
        private static readonly SemaphoreSlim ConnectGate = new(1, 1);

        private readonly string _mac;
        private readonly int _connectTimeoutMs;

        private BluetoothSocket? _socket;
        private Stream? _in;
        private Stream? _out;

        public AndroidBluetoothConnection(string mac, int connectTimeoutMs = 8000)
        {
            if (string.IsNullOrWhiteSpace(mac)) throw new ArgumentException("MAC vuoto", nameof(mac));
            _mac = mac.Trim();
            _connectTimeoutMs = System.Math.Max(2000, connectTimeoutMs);
        }

        public bool IsOpen => _socket?.IsConnected == true && _in != null && _out != null;

        public void Open()
        {
            if (IsOpen) return;

            var adapter = BluetoothAdapter.DefaultAdapter
                          ?? throw new InvalidOperationException("BluetoothAdapter non disponibile");
            if (!adapter.IsEnabled) throw new InvalidOperationException("Bluetooth disabilitato");

            var device = adapter.GetRemoteDevice(_mac);

            ConnectGate.Wait();
            try
            {
                if (adapter.IsDiscovering) adapter.CancelDiscovery();
                SpinWait.SpinUntil(() => !adapter.IsDiscovering, 500);
                System.Threading.Thread.Sleep(200);

                System.Exception? last = null;

                if (TryConnect(CreateSecure(device), out last)) return;
                if (TryConnect(CreateInsecure(device), out last)) return;
                if (TryConnect(CreateReflectChannel1(device), out last)) return;

                throw new IOException($"Unable to connect to {_mac}");
            }
            finally
            {
                ConnectGate.Release();
            }
        }

        public void Close()
        {
            try { _in?.Close(); } catch { }
            try { _out?.Close(); } catch { }
            try { _socket?.Close(); } catch { }
            _in = null; _out = null; _socket = null;
        }

        public int ReadByte()
        {
            var s = _in ?? throw new IOException("Input stream non disponibile");
            int b = s.ReadByte();
            if (b < 0) throw new IOException("End of stream");
            return b & 0xFF;
        }

        public void WriteBytes(byte[] buffer, int index, int length)
        {
            var s = _out ?? throw new IOException("Output stream non disponibile");
            s.Write(buffer, index, length);
            s.Flush();
        }

        public void Flush() { /* no-op */ }
        public void FlushInput() { /* no-op */ }

        // ---- Helpers --------------------------------------------------------

        private static BluetoothSocket CreateSecure(BluetoothDevice d) =>
            d.CreateRfcommSocketToServiceRecord(SPP_UUID);

        private static BluetoothSocket CreateInsecure(BluetoothDevice d) =>
            d.CreateInsecureRfcommSocketToServiceRecord(SPP_UUID);

        /// <summary>Fallback via reflection: createRfcommSocket(int channel=1).</summary>
        private static BluetoothSocket CreateReflectChannel1(BluetoothDevice d)
        {
            Java.Lang.Reflect.Method m = d.Class.GetMethod(
                "createRfcommSocket",
                new Java.Lang.Class[] { Java.Lang.Integer.Type } // parametro "int"
            );
            var socketObj = m.Invoke(d, new Java.Lang.Object[] { Java.Lang.Integer.ValueOf(1) });
            return (BluetoothSocket)socketObj;
        }

        /// <summary>Tenta la connessione con timeout; prepara gli stream su successo.</summary>
        private bool TryConnect(BluetoothSocket sock, out System.Exception? error)
        {
            error = null;
            try
            {
                var connectTask = Task.Run(() =>
                {
                    try { sock.Connect(); }
                    catch (System.Exception ex) { throw new IOException($"Connect fallita: {ex.Message}", ex); }
                });

                if (!connectTask.Wait(_connectTimeoutMs))
                    throw new TimeoutException($"Timeout ({_connectTimeoutMs} ms) durante Connect()");

                if (!sock.IsConnected)
                    throw new IOException("Socket non connesso dopo Connect()");

                _socket = sock;
                _in = _socket.InputStream;
                _out = _socket.OutputStream;

                Debug.WriteLine($"[BT] Connesso a {_mac} ({_socket.RemoteDevice?.Name ?? "?"})");
                return true;
            }
            catch (System.Exception ex)
            {
                error = ex;
                try { sock.Close(); } catch { }
                _socket = null; _in = null; _out = null;
                Debug.WriteLine($"[BT] Tentativo fallito su {_mac}: {ex.Message}");
                System.Threading.Thread.Sleep(200);
                return false;
            }
        }
    }
}
#endif
