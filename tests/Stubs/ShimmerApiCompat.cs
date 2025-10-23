/* 
 * ShimmerApiStubs.cs
 * Purpose: Test stubs for ShimmerAPI types used by unit tests.
 */

#if TEST_STUBS


using System;


namespace ShimmerAPI
{

    /// <summary>
    /// Minimal event args used by the bridge/tests to simulate Shimmer SDK callbacks.
    /// </summary>
    public class CustomEventArgs : EventArgs
    {
        private readonly int _indicator;
        private readonly object? _object;


        /// <summary>
        /// Creates a new instance.
        /// </summary>
        /// <param name="indicator">Numeric "what happened" indicator.</param>
        /// <param name="obj">Optional payload object.</param>
        public CustomEventArgs(int indicator, object? obj) 
        {
            _indicator = indicator; 
            _object = obj; 
        }


        /// <summary>
        /// Gets the numeric indicator associated with the event.
        /// </summary>
        /// <returns>The indicator value provided at construction time.</returns>
        public int getIndicator() => _indicator;


        /// <summary>
        /// Gets the optional payload object attached to the event.
        /// </summary>
        /// <returns>The payload object (possibly <c>null</c>).</returns>
        public object? getObject() => _object;
    }


    /// <summary>
    /// Minimal subset of Shimmer Bluetooth identifiers and constants referenced by the bridge/tests.
    /// </summary>
    public static class ShimmerBluetooth
    {

        /// <summary>
        /// Identifiers that tag UI/SDK callback messages.
        /// </summary>
        public enum ShimmerIdentifier
        {
            MSG_IDENTIFIER_STATE_CHANGE = 100,  // State change messages
            MSG_IDENTIFIER_DATA_PACKET = 101    // Data packet messages
        }


        /// <summary>
        /// Shimmer3 sensor bitmap flags (bitfield). Combine with bitwise OR to enable multiple sensors.
        /// </summary>
        public static class SensorBitmapShimmer3
        {
            public const int SENSOR_EXG1_24BIT = 1 << 0;
            public const int SENSOR_EXG2_24BIT = 1 << 1;
            public const int SENSOR_A_ACCEL = 1 << 2;           
            public const int SENSOR_D_ACCEL = 1 << 3;
            public const int SENSOR_MPU9150_GYRO = 1 << 4;
            public const int SENSOR_LSM303DLHC_MAG = 1 << 5;
            public const int SENSOR_BMP180_PRESSURE = 1 << 6;
            public const int SENSOR_VBATT = 1 << 7;
            public const int SENSOR_EXT_A6 = 1 << 8;
            public const int SENSOR_EXT_A7 = 1 << 9;
            public const int SENSOR_EXT_A15 = 1 << 10;
        }
    }


    /// <summary>
    /// Minimal constants for general Shimmer signal names referenced by the code under test.
    /// </summary>
    public static class ShimmerConfiguration
    {

        /// <summary>
        /// Common signal name strings.
        /// </summary>
        public static class SignalNames
        {

            // System timestamp signal name
            public const string SYSTEM_TIMESTAMP = "SYSTEM_TIMESTAMP";
        }
    }


    /// <summary>
    /// Minimal constants for Shimmer3-specific signal names referenced by the code under test.
    /// </summary>
    public static class Shimmer3Configuration
    {

        /// <summary>
        /// Shimmer3 signal name strings.
        /// </summary>
        public static class SignalNames
        {
            public const string EXG1_CH1 = "EXG1_CH1";
            public const string EXG2_CH1 = "EXG2_CH1";
        }
    }


    /// <summary>
    /// Base stub for the data container returned by the Shimmer SDK (very minimal for tests).
    /// </summary>
    public class ObjectCluster
    {

        /// <summary>
        /// Parameterless constructor required by tests.
        /// </summary>
        public ObjectCluster() {}


        /// <summary>
        /// Looks up the index of a signal by name and format.
        /// </summary>
        /// <param name="name">Signal name.</param>
        /// <param name="format">Requested format (e.g., "CAL" or "RAW").</param>
        /// <returns>
        /// <list type="bullet">
        /// <item><description>Throws for name <c>"__THROW__"</c> (to test exception paths).</description></item>
        /// <item><description>Returns <c>5</c> for name <c>"__FIVE__"</c>.</description></item>
        /// <item><description>Returns <c>-1</c> for unknown names.</description></item>
        /// </list>
        /// </returns>
        public virtual int GetIndex(string name, string format)
        {
            if (name == "__THROW__") throw new Exception("boom");
            if (name == "__FIVE__") return 5;
            return -1;
        }


        /// <summary>
        /// Retrieves sensor data by index.
        /// </summary>
        /// <param name="idx">Signal index.</param>
        /// <returns>A <see cref="SensorData"/> instance (default value <c>0</c> if not set).</returns>
        /// <exception cref="Exception">Thrown when <paramref name="idx"/> equals <c>777</c> (for testing).</exception>
        public virtual SensorData GetData(int idx)
        {
            if (idx == 777) throw new Exception("boom");
            return new SensorData(0);
        }
    }


    /// <summary>
    /// Minimal wrapper holding a single data payload used by tests.
    /// </summary>
    public class SensorData
    {

        /// <summary>
        /// The payload value stored in this instance.
        /// </summary>
        public object Data { get; }


        /// <summary>
        /// Creates a new sensor data wrapper.
        /// </summary>
        /// <param name="d">Payload to store.</param>
        public SensorData(object d) 
        { 
            Data = d; 
        }
    }


    /// <summary>
    /// Configurable mock of <see cref="ObjectCluster"/> used by tests.
    /// Allows mapping (name, format) → index, providing per-index data, and forcing exceptions.
    /// </summary>
    public class ObjectClusterMock : ObjectCluster
    {

        // Map (name, fmt) -> index (case-insensitive)
        private readonly System.Collections.Generic.Dictionary<(string name, string fmt), int> _map =
            new System.Collections.Generic.Dictionary<(string, string), int>(new NameFmtComparer());

        // Data per index
        private readonly System.Collections.Generic.Dictionary<int, SensorData> _data =
            new System.Collections.Generic.Dictionary<int, SensorData>();

        // Exception triggers
        private readonly System.Collections.Generic.HashSet<string> _throwOnName =
            new System.Collections.Generic.HashSet<string>(StringComparer.OrdinalIgnoreCase);
        private readonly System.Collections.Generic.HashSet<int> _throwOnIdx =
            new System.Collections.Generic.HashSet<int>();


        /// <summary>
        /// Associates an index with a (name, format) tuple.
        /// </summary>
        /// <param name="name">Signal name (case-insensitive; <c>null</c> treated as empty).</param>
        /// <param name="fmt">Signal format (case-insensitive; <c>null</c> treated as empty).</param>
        /// <param name="index">Index to return for the tuple.</param>
        public void SetIndex(string name, string fmt, int index) =>
            _map[(name ?? string.Empty, fmt ?? string.Empty)] = index;


        /// <summary>
        /// Forces <see cref="GetIndex"/> to throw for the specified signal name.
        /// </summary>
        /// <param name="name">Signal name to trigger an exception for.</param>
        public void ThrowOnName(string name)
        {
            if (!string.IsNullOrEmpty(name)) _throwOnName.Add(name);
        }


        /// <summary>
        /// Forces <see cref="GetData"/> to throw for the specified index.
        /// </summary>
        /// <param name="idx">Index to trigger an exception for.</param>
        public void ThrowOnIdx(int idx) => _throwOnIdx.Add(idx);


        /// <summary>
        /// Looks up the index mapped for the given (name, format) pair (case-insensitive).
        /// </summary>
        /// <param name="name">Signal name; <c>null</c> is treated as empty string.</param>
        /// <param name="format">Signal format (e.g., "RAW", "CAL"); <c>null</c> is treated as empty string.</param>
        /// <returns>
        /// The mapped index if present; otherwise <c>-1</c>.
        /// </returns>
        /// <exception cref="Exception">
        /// Thrown with message <c>"boom (GetIndex)"</c> when <paramref name="name"/> was registered via
        /// <c>ThrowOn(name)</c> or <c>ThrowOn(name, "GetIndex")</c>.
        /// </exception>
        public override int GetIndex(string name, string format)
        {
            if (!string.IsNullOrEmpty(name) && _throwOnName.Contains(name))
                throw new Exception("boom (GetIndex)");
            return _map.TryGetValue((name ?? string.Empty, format ?? string.Empty), out var idx) ? idx : -1;
        }


        /// <summary>
        /// Retrieves the <see cref="SensorData"/> associated with the given index, if any.
        /// </summary>
        /// <param name="idx">Index previously provided via <c>SetData</c> or looked up from <c>GetIndex</c>.</param>
        /// <returns>
        /// The stored <see cref="SensorData"/> if found; otherwise a new <see cref="SensorData"/> with payload <c>0</c>.
        /// </returns>
        /// <exception cref="Exception">
        /// Thrown with message <c>"boom (GetData)"</c> when <paramref name="idx"/> was registered via <c>ThrowOn(idx)</c>.
        /// </exception>
        public override SensorData GetData(int idx)
        {
            if (_throwOnIdx.Contains(idx))
                throw new Exception("boom (GetData)");
            return _data.TryGetValue(idx, out var s) ? s : new SensorData(0);
        }


        /// <summary>
        /// Case-insensitive comparer for (name, format) dictionary keys.
        /// </summary>
        private sealed class NameFmtComparer : System.Collections.Generic.IEqualityComparer<(string name, string fmt)>
        {

            /// <summary>
            /// Determines whether two key tuples are equal (case-insensitive).
            /// </summary>
            /// <param name="x">First tuple.</param>
            /// <param name="y">Second tuple.</param>
            /// <returns><c>true</c> if both name and format match ignoring case; otherwise <c>false</c>.</returns>
            public bool Equals((string name, string fmt) x, (string name, string fmt) y) =>
                string.Equals(x.name, y.name, StringComparison.OrdinalIgnoreCase) &&
                string.Equals(x.fmt, y.fmt, StringComparison.OrdinalIgnoreCase);


            /// <summary>
            /// Gets a hash code for the tuple (case-insensitive).
            /// </summary>
            /// <param name="obj">Key tuple.</param>
            /// <returns>A hash code suitable for use in hash-based collections.</returns>
            public int GetHashCode((string name, string fmt) obj) =>
                StringComparer.OrdinalIgnoreCase.GetHashCode(obj.name ?? string.Empty) * 397 ^
                StringComparer.OrdinalIgnoreCase.GetHashCode(obj.fmt ?? string.Empty);
        }


        // ----- Helpers added for tests -----


        /// <summary>
        /// Fluent overload to set a mapping in a single call.
        /// </summary>
        /// <param name="name">Signal name.</param>
        /// <param name="format">Signal format.</param>
        /// <param name="index">Index to return for the tuple.</param>
        /// <returns>The same <see cref="ObjectClusterMock"/> instance (for chaining).</returns>
        public ObjectClusterMock When(string name, string format, int index)
        {
            SetIndex(name, format, index);
            return this;
        }


        /// <summary>
        /// Fluent helper: convenience overload to register a throw for a member by name.
        /// Currently only recognizes <c>"GetIndex"</c>.
        /// </summary>
        /// <param name="name">Signal name to register.</param>
        /// <param name="member">Member selector (e.g., <c>"GetIndex"</c>).</param>
        /// <returns>The same <see cref="ObjectClusterMock"/> instance (for chaining).</returns>
        public ObjectClusterMock ThrowOn(string name, string member)
        {
            if (!string.IsNullOrEmpty(member) &&
                member.Equals("GetIndex", StringComparison.OrdinalIgnoreCase))
            {
                ThrowOnName(name);
            }
            return this;
        }
    }
}

#endif
