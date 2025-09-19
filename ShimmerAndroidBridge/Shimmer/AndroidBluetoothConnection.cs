/*
 * Android-specific helper that manages Bluetooth RFCOMM (SPP) connections to Shimmer devices.
 * Provides an implementation of IShimmerConnection using Android's Bluetooth APIs.
 * Handles connect/disconnect logic, input/output streams, timeouts and fallback strategies.
 */


#if ANDROID
using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Android.Bluetooth;
using Java.Util;

namespace ShimmerSDK.Android
{

    /// <summary>
    /// Android Bluetooth RFCOMM connection (SPP) implementing IShimmerConnection.
    /// Handles connect/close, I/O streams and basic retries.
    /// </summary>
    internal sealed class AndroidBluetoothConnection : IShimmerConnection
    {

        // Standard Serial Port Profile UUID used by classic Bluetooth devices
        private static readonly UUID SPP_UUID =
            UUID.FromString("00001101-0000-1000-8000-00805F9B34FB");

        // Global semaphore to serialize connection attempts across instances and avoid adapter contention
        private static readonly SemaphoreSlim ConnectGate = new(1, 1);

        private readonly string _mac;
        private readonly int _connectTimeoutMs;
        private BluetoothSocket? _socket;
        private Stream? _in;
        private Stream? _out;


        /// <summary>
        /// Creates a new Android Bluetooth RFCOMM connection to a Shimmer device.
        /// </summary>
        /// <param name="mac">Target device MAC address (e.g., "00:11:22:33:44:55").</param>
        /// <param name="connectTimeoutMs">Connection timeout in milliseconds.</param>
        /// <exception cref="ArgumentException">Thrown if <paramref name="mac"/> is null/empty/whitespace.</exception>
        public AndroidBluetoothConnection(string mac, int connectTimeoutMs = 8000)
        {
            if (string.IsNullOrWhiteSpace(mac)) throw new ArgumentException("MAC vuoto", nameof(mac));
            _mac = mac.Trim();
            _connectTimeoutMs = System.Math.Max(2000, connectTimeoutMs);
        }


        /// <summary>
        /// Indicates whether the underlying socket is connected and both I/O streams are ready.
        /// </summary>
        public bool IsOpen => _socket?.IsConnected == true && _in != null && _out != null;


        /// <summary>
        /// Opens the Bluetooth RFCOMM connection to the target MAC, preparing input/output streams.
        /// Tries, in order: secure SPP, insecure SPP, reflection-based channel 1.
        /// </summary>
        /// <remarks>
        /// Cancels discovery if running, then attempts connection with a timeout guard.
        /// Method is idempotent: if already open, it returns immediately.
        /// </remarks>
        /// <exception cref="InvalidOperationException">Bluetooth adapter unavailable or disabled.</exception>
        /// <exception cref="IOException">If all connection strategies fail or socket is not connected.</exception>
        public void Open()
        {

            // No-op if already connected and streams are ready.
            if (IsOpen) return;

            // Get the default adapter and validate availability/state.
            var adapter = BluetoothAdapter.DefaultAdapter
                          ?? throw new InvalidOperationException("Bluetooth Adapter not available");
            if (!adapter.IsEnabled) throw new InvalidOperationException("Bluetooth disabled");

            // Resolve the remote device from its MAC address.
            var device = adapter.GetRemoteDevice(_mac);

            // Serialize connect attempts across instances to avoid collisions while
            // discovery may be running or the adapter is busy
            ConnectGate.Wait();
            try
            {

                // Discovery interferes with RFCOMM connect; ensure it's stopped first.
                if (adapter.IsDiscovering) adapter.CancelDiscovery();
                SpinWait.SpinUntil(() => !adapter.IsDiscovering, 500);
                System.Threading.Thread.Sleep(200);

                System.Exception? last = null;

                // Try secure, insecure, then reflection-based RFCOMM connection (stop at first success).
                if (TryConnect(CreateSecure(device), out last)) return;
                if (TryConnect(CreateInsecure(device), out last)) return;
                if (TryConnect(CreateReflectChannel1(device), out last)) return;

                throw new IOException($"Unable to connect to {_mac}. Last error: {last?.Message}", last);
            }
            finally
            {

                // Always release the semaphore to avoid deadlocks.
                ConnectGate.Release();
            }
        }

        /// <summary>
        /// Closes the connection and disposes the underlying streams and socket.
        /// Safe to call multiple times; exceptions during Close are intentionally swallowed.
        /// </summary>
        public void Close()
        {
            try { _in?.Close(); } catch { }
            try { _out?.Close(); } catch { }
            try { _socket?.Close(); } catch { }
            _in = null; _out = null; _socket = null;
        }


        /// <summary>
        /// Reads a single byte from the input stream.
        /// </summary>
        /// <returns>The byte read as an integer in the range [0..255].</returns>
        /// <exception cref="IOException">
        /// Thrown if the input stream is unavailable or end-of-stream is reached.
        /// </exception>
        public int ReadByte()
        {
            var s = _in ?? throw new IOException("Input stream not available");
            int b = s.ReadByte();
            if (b < 0) throw new IOException("End of stream");
            return b & 0xFF;
        }


        /// <summary>
        /// Writes a segment of bytes to the output stream and flushes it.
        /// </summary>
        /// <param name="buffer">The source buffer containing data to send.</param>
        /// <param name="index">Zero-based offset in <paramref name="buffer"/> at which to begin writing.</param>
        /// <param name="length">Number of bytes to write.</param>
        /// <exception cref="IOException">
        /// Thrown if the output stream is unavailable or the write/flush operation fails.
        /// </exception>
        public void WriteBytes(byte[] buffer, int index, int length)
        {
            var s = _out ?? throw new IOException("Output stream not available");
            s.Write(buffer, index, length);
            s.Flush();
        }


        // Present for API compatibility: ShimmerBluetooth calls these to clear serial I/O
        // (e.g., COM port buffers on Windows). On Android’s RFCOMM streams, nothing to flush.
        public void Flush() { /* no-op */ }
        public void FlushInput() { /* no-op */ }


        // ---- Helpers --------------------------------------------------------


        /// <summary>
        /// Creates a secure RFCOMM socket bound to the standard SPP UUID.
        /// </summary>
        /// <param name="d">Target Bluetooth device.</param>
        /// <returns>A secure <see cref="BluetoothSocket"/>.</returns>
        private static BluetoothSocket CreateSecure(BluetoothDevice d) =>
            d.CreateRfcommSocketToServiceRecord(SPP_UUID);


        /// <summary>
        /// Creates an insecure RFCOMM socket bound to the standard SPP UUID.
        /// Useful as a fallback when secure sockets fail due to pairing issues.
        /// </summary>
        /// <param name="d">Target Bluetooth device.</param>
        /// <returns>An insecure <see cref="BluetoothSocket"/>.</returns>
        private static BluetoothSocket CreateInsecure(BluetoothDevice d) =>
            d.CreateInsecureRfcommSocketToServiceRecord(SPP_UUID);


        /// <summary>
        /// Reflection-based fallback that invokes <c>createRfcommSocket(int channel)</c> with channel = 1.
        /// Some stacks/ROMs require this legacy path when SPP UUID sockets fail.
        /// </summary>
        /// <param name="d">Target Bluetooth device.</param>
        /// <returns>A <see cref="BluetoothSocket"/> created via reflection.</returns>
        private static BluetoothSocket CreateReflectChannel1(BluetoothDevice d)
        {

            // Lookup method: BluetoothDevice.createRfcommSocket(int)
            Java.Lang.Reflect.Method m = d.Class.GetMethod(
                "createRfcommSocket",
                new Java.Lang.Class[] { Java.Lang.Integer.Type }    // parameter type: int
            );

            // Invoke with channel 1 (common for SPP)
            var socketObj = m.Invoke(d, new Java.Lang.Object[] { Java.Lang.Integer.ValueOf(1) });
            return (BluetoothSocket)socketObj;
        }


        /// <summary>
        /// Attempts to connect the given socket within the configured timeout and, on success,
        /// initializes input/output streams.
        /// </summary>
        /// <param name="sock">Socket to connect.</param>
        /// <param name="error">Outputs the last thrown exception if the attempt fails.</param>
        /// <returns><c>true</c> if the connection succeeds; otherwise <c>false</c>.</returns>
        /// <exception cref="TimeoutException">Thrown if <c>Connect()</c> does not complete within the timeout.</exception>
        /// <exception cref="IOException">
        /// Thrown if <c>Connect()</c> fails or the socket is not connected after <c>Connect()</c>.
        /// </exception>
        private bool TryConnect(BluetoothSocket sock, out System.Exception? error)
        {
            error = null;
            try
            {

                // Guard Connect() with a timeout
                var connectTask = Task.Run(() =>
                {
                    try { sock.Connect(); }
                    catch (System.Exception ex) { throw new IOException($"Connect failed: {ex.Message}", ex); }
                });

                if (!connectTask.Wait(_connectTimeoutMs))
                    throw new TimeoutException($"Timeout ({_connectTimeoutMs} ms) durante Connect()");

                if (!sock.IsConnected)
                    throw new IOException("Socket not connected after Connect()");

                // Bind streams on success
                _socket = sock;
                _in = _socket.InputStream;
                _out = _socket.OutputStream;

                return true;
            }
            catch (System.Exception ex)
            {
                error = ex;
                try { sock.Close(); } catch { }
                _socket = null; _in = null; _out = null;
                System.Threading.Thread.Sleep(200); // brief backoff before next strategy
                return false;
            }
        }
    }
}
#endif
