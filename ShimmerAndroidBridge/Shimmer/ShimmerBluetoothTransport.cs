/*
 * Android-specific transport adapter that bridges ShimmerBluetooth with IShimmerConnection.
 * Delegates all I/O and lifecycle calls to an Android Bluetooth RFCOMM connection.
 * Required by the ShimmerAPI to provide a platform-specific implementation
 * of open/close, read/write, flush, and address handling.
 */


#if ANDROID
using System;
using ShimmerAPI;

namespace ShimmerSDK.Android
{

    /// <summary>
    /// Android transport adapter that connects <see cref="ShimmerBluetooth"/> 
    /// with <see cref="IShimmerConnection"/> using classic Bluetooth RFCOMM.
    /// </summary>
    internal sealed class ShimmerBluetoothTransport : ShimmerBluetooth
    {

        private readonly IShimmerConnection _conn;
        private string _address = "ANDROID-BT";


        /// <summary>
        /// Creates a transport that bridges <see cref="ShimmerBluetooth"/> with an Android Bluetooth connection.
        /// </summary>
        /// <param name="deviceName">Logical device name exposed to the base class.</param>
        /// <param name="conn">Concrete Android connection implementing <see cref="IShimmerConnection"/>.</param>
        /// <exception cref="ArgumentNullException">Thrown when <paramref name="conn"/> is null.</exception>
        public ShimmerBluetoothTransport(string deviceName, IShimmerConnection conn)
            : base(deviceName)
        {
            _conn = conn ?? throw new ArgumentNullException(nameof(conn));
        }


        // ---- Required overrides from ShimmerBluetooth (delegate to IShimmerConnection) -------


        /// <summary>Returns whether the underlying connection is currently open.</summary>
        protected override bool IsConnectionOpen() => _conn.IsOpen;

        /// <summary>Opens the underlying Android Bluetooth connection.</summary>
        protected override void OpenConnection() => _conn.Open();

        /// <summary>Closes the underlying Android Bluetooth connection.</summary>
        protected override void CloseConnection() => _conn.Close();

        /// <summary>Reads a single byte from the underlying connection.</summary>
        protected override int ReadByte() => _conn.ReadByte();

        /// <summary>Writes a segment of bytes to the underlying connection.</summary>
        protected override void WriteBytes(byte[] buffer, int index, int length) => _conn.WriteBytes(buffer, index, length);

        /// <summary>
        /// Flushes pending output if supported by the transport.
        /// On Android RFCOMM this is a no-op but kept for API parity with other platforms.
        /// </summary>
        protected override void FlushConnection() => _conn.Flush();

        /// <summary>
        /// Clears pending input if supported by the transport.
        /// On Android RFCOMM this is a no-op but kept for API parity with other platforms.
        /// </summary>
        protected override void FlushInputConnection() => _conn.FlushInput();


        // ---- Required by ShimmerAPI: ShimmerBluetooth declares abstract
        // GetShimmerAddress/SetShimmerAddress; this adapter must implement them
        // to provide the connection address (COM port on Windows, Bluetooth MAC on Android). ----


        /// <summary>
        /// Returns the address used by the SDK to identify/connect to this Shimmer device
        /// (Bluetooth MAC on Android).
        /// </summary>
        /// <returns>The connection address string currently set for this device.</returns>
        public override string GetShimmerAddress() => _address;


        /// <summary>
        /// Sets the address used by the SDK to identify/connect to this Shimmer device.
        /// Falls back to "ANDROID-BT" when the provided value is null/empty/whitespace.
        /// </summary>
        /// <param name="address">Connection address (Bluetooth MAC on Android).</param>
        public override void SetShimmerAddress(string address)
        {
            _address = string.IsNullOrWhiteSpace(address) ? "ANDROID-BT" : address.Trim();
        }


        // ---- Useful public accessors -------


        /// <summary>
        /// Indicates whether the underlying Android Bluetooth connection is open.
        /// </summary>
        /// <returns><c>true</c> if the connection is open; otherwise <c>false</c>.</returns>
        public bool ConnectionOpen => IsConnectionOpen();  
        

        /// <summary>
        /// Exposes the firmware full name computed by the base <see cref="ShimmerBluetooth"/>.
        /// </summary>
        /// <returns>A human-readable firmware identifier.</returns>
        public string FirmwareVersionFullNamePublic => FirmwareVersionFullName;

        
        // ---- Public forwards for Flush/FlushInput (used by wrappers) -------


        /// <summary>
        /// Forwards to <see cref="FlushConnection"/>. No-op on Android RFCOMM, kept for API parity.
        /// </summary>
        public void Flush() => FlushConnection();


        /// <summary>
        /// Forwards to <see cref="FlushInputConnection"/>. No-op on Android RFCOMM, kept for API parity.
        /// </summary>
        public void FlushInput() => FlushInputConnection();
    }
}
#endif
