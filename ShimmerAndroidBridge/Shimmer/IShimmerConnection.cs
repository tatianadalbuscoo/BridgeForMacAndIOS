/*
 * Android-only transport interface for ShimmerBluetooth.
 * Provides a minimal, stream-like contract (open/close/read/write/flush)
 * so higher-level SDK code can talk to Shimmer devices without depending
 * on Android Bluetooth APIs directly.
 *
 * Implemented by: AndroidBluetoothConnection.
 * Consumed by:    ShimmerBluetoothTransport (wires SDK logic to this transport).
 */

#if ANDROID
using System.IO;

namespace ShimmerSDK.Android
{
    internal interface IShimmerConnection
    {
        bool IsOpen { get; }
        void Open();
        void Close();
        int ReadByte();
        void WriteBytes(byte[] buffer, int index, int length);
        void Flush();
        void FlushInput();
    }
}
#endif
