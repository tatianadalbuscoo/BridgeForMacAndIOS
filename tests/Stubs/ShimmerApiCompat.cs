#if TEST_STUBS
using System;

namespace ShimmerAPI
{
    // Event args minimi usati dal bridge
    public class CustomEventArgs : EventArgs
    {
        private readonly int _indicator;
        private readonly object? _object;
        public CustomEventArgs(int indicator, object? obj) { _indicator = indicator; _object = obj; }
        public int getIndicator() => _indicator;
        public object? getObject() => _object;
    }

    public static class ShimmerBluetooth
    {
        public enum ShimmerIdentifier
        {
            MSG_IDENTIFIER_STATE_CHANGE = 100,
            MSG_IDENTIFIER_DATA_PACKET = 101
        }

        // >>> AGGIUNGI QUESTO BLOCCO <<<
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


    public static class ShimmerConfiguration
    {
        public static class SignalNames
        {
            public const string SYSTEM_TIMESTAMP = "SYSTEM_TIMESTAMP";
        }
    }

    public static class Shimmer3Configuration
    {
        public static class SignalNames
        {
            public const string EXG1_CH1 = "EXG1_CH1";
            public const string EXG2_CH1 = "EXG2_CH1";
        }
    }

    // Stub usati dai test
    public class ObjectCluster
    {
        public ObjectCluster() { } // ctor vuoto richiesto dai test
        public virtual int GetIndex(string name, string format)
        {
            // trigger eccezione per test del catch in SafeIdx
            if (name == "__THROW__") throw new Exception("boom");

            // ritorno “valido” per test happy-path
            if (name == "__FIVE__") return 5;

            // label sconosciuta -> non trovato
            return -1;
        }

        public virtual SensorData GetData(int idx)
        {
            // Simula un’eccezione controllata se l’indice è “speciale” e positivo
            if (idx == 777) throw new Exception("boom");
            return new SensorData(0);
        }

    }

    public class SensorData
    {
        public object Data { get; }
        public SensorData(object d) { Data = d; }
    }

    /// <summary>
    /// Mock configurabile di ObjectCluster, usata dai test:
    /// - consente di mappare (name, format) -> index
    /// - consente di associare dati per indice
    /// - può forzare eccezioni su nomi/indici specifici
    /// </summary>
    public class ObjectClusterMock : ObjectCluster
    {
        // Mappa (name, fmt) -> index, case-insensitive
        private readonly System.Collections.Generic.Dictionary<(string name, string fmt), int> _map =
            new System.Collections.Generic.Dictionary<(string, string), int>(new NameFmtComparer());

        // Dati per indice
        private readonly System.Collections.Generic.Dictionary<int, SensorData> _data =
            new System.Collections.Generic.Dictionary<int, SensorData>();

        // Trigger di eccezioni
        private readonly System.Collections.Generic.HashSet<string> _throwOnName =
            new System.Collections.Generic.HashSet<string>(StringComparer.OrdinalIgnoreCase);
        private readonly System.Collections.Generic.HashSet<int> _throwOnIdx =
            new System.Collections.Generic.HashSet<int>();

        /// <summary>Associa un indice a (name, fmt).</summary>
        public void SetIndex(string name, string fmt, int index) =>
            _map[(name ?? string.Empty, fmt ?? string.Empty)] = index;

        /// <summary>Imposta un dato per un certo indice.</summary>
        public void SetData(int index, object value) =>
            _data[index] = new SensorData(value);

        /// <summary>Forza eccezione su GetIndex per un certo name.</summary>
        public void ThrowOnName(string name)
        {
            if (!string.IsNullOrEmpty(name)) _throwOnName.Add(name);
        }

        /// <summary>Forza eccezione su GetData per un certo indice.</summary>
        public void ThrowOnIdx(int idx) => _throwOnIdx.Add(idx);

        public override int GetIndex(string name, string format)
        {
            if (!string.IsNullOrEmpty(name) && _throwOnName.Contains(name))
                throw new Exception("boom (GetIndex)");
            return _map.TryGetValue((name ?? string.Empty, format ?? string.Empty), out var idx) ? idx : -1;
        }

        public override SensorData GetData(int idx)
        {
            if (_throwOnIdx.Contains(idx))
                throw new Exception("boom (GetData)");
            return _data.TryGetValue(idx, out var s) ? s : new SensorData(0);
        }

        private sealed class NameFmtComparer : System.Collections.Generic.IEqualityComparer<(string name, string fmt)>
        {
            public bool Equals((string name, string fmt) x, (string name, string fmt) y) =>
                string.Equals(x.name, y.name, StringComparison.OrdinalIgnoreCase) &&
                string.Equals(x.fmt, y.fmt, StringComparison.OrdinalIgnoreCase);

            public int GetHashCode((string name, string fmt) obj) =>
                StringComparer.OrdinalIgnoreCase.GetHashCode(obj.name ?? string.Empty) * 397 ^
                StringComparer.OrdinalIgnoreCase.GetHashCode(obj.fmt ?? string.Empty);
        }

        // -----------------------------
        // >>> AGGIUNTE PER I TEST <<<
        // -----------------------------

        // Fluent helper: ocMock.When("Gyroscope X","CAL").ReturnsIndex(5);
        public IndexSetup When(string name, string format) => new IndexSetup(this, name, format);

        // Overload a 3 argomenti atteso dai test: When(name, format, index)
        public ObjectClusterMock When(string name, string format, int index)
        {
            SetIndex(name, format, index);
            return this; // fluent
        }

        public readonly struct IndexSetup
        {
            private readonly ObjectClusterMock _parent;
            private readonly string _name;
            private readonly string _fmt;

            public IndexSetup(ObjectClusterMock parent, string name, string fmt)
            {
                _parent = parent;
                _name = name ?? string.Empty;
                _fmt = fmt ?? string.Empty;
            }

            public void ReturnsIndex(int index) => _parent.SetIndex(_name, _fmt, index);
            public void Returns(int index) => _parent.SetIndex(_name, _fmt, index);
        }

        // ---- ThrowOn varianti fluide ----

        // Throw su GetIndex per nome
        public ObjectClusterMock ThrowOn(string name)
        {
            ThrowOnName(name);
            return this;
        }

        // Throw su GetData per indice
        public ObjectClusterMock ThrowOn(int idx)
        {
            ThrowOnIdx(idx);
            return this;
        }

        // Variante a due argomenti usata dai test, es: ThrowOn("__THROW__", "GetIndex")
        public ObjectClusterMock ThrowOn(string name, string member)
        {
            if (!string.IsNullOrEmpty(member) &&
                member.Equals("GetIndex", StringComparison.OrdinalIgnoreCase))
            {
                ThrowOnName(name);
            }
            // Se mai servisse: else if (member.Equals("GetData", StringComparison.OrdinalIgnoreCase)) { ... }
            return this;
        }
    }
    }
#endif

