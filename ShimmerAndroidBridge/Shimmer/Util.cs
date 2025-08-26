#if ANDROID
using System;
using System.Text;

namespace XR2Learn_ShimmerAPI.IMU.Android
{
    public static class Util
    {
        /// <summary>
        /// Converte un array di byte in stringa esadecimale (es. "01-0A-FF").
        /// </summary>
        public static string ByteArrayToHex(byte[] bytes)
        {
            if (bytes == null) return string.Empty;
            return BitConverter.ToString(bytes);
        }

        /// <summary>
        /// Converte un array di byte in stringa esadecimale continua (es. "010AFF").
        /// </summary>
        public static string ByteArrayToHexCompact(byte[] bytes)
        {
            if (bytes == null) return string.Empty;
            var sb = new StringBuilder(bytes.Length * 2);
            foreach (byte b in bytes)
                sb.AppendFormat("{0:X2}", b);
            return sb.ToString();
        }

        /// <summary>
        /// Stampa su log Android un array di byte in esadecimale.
        /// </summary>
        public static void PrintHex(string tag, byte[] bytes)
        {
            global::Android.Util.Log.Debug(tag ?? "ShimmerUtil", ByteArrayToHex(bytes));
        }

        /// <summary>
        /// Arrotonda un valore double al numero di decimali specificato.
        /// </summary>
        public static double RoundToDecimals(double value, int decimals)
        {
            return Math.Round(value, decimals);
        }

        /// <summary>
        /// Restituisce true se l'array è nullo o vuoto.
        /// </summary>
        public static bool IsNullOrEmpty(byte[] array)
        {
            return array == null || array.Length == 0;
        }
    }
}
#endif

