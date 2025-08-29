using Android.App;
using Android.Bluetooth;
using Android.OS;
using Android.Widget;
using Android.Content.PM;
using Android.Views;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using STimers = System.Timers;
using Com.Example.ShimmerBridge;
using AndroidResource = ShimmerAndroidBridge.Resource;

namespace com.example.shimmerbridge.cs
{
    [Activity(Label = "Shimmer Bridge", Theme = "@style/AppTheme", MainLauncher = true)]
    public class MainActivity : Activity
    {
        LinearLayout _deviceGroup = null!;
        TextView _status = null!;
        Button _btnStart = null!, _btnStop = null!;

        readonly WsBridgeManager _ws = new WsBridgeManager();
        readonly List<DeviceUi> _devices = new(); // per-device sensors + connect flag
        // debounce per MAC
        readonly Dictionary<string, STimers.Timer> _debouncers = new(StringComparer.OrdinalIgnoreCase);

        const string TAG = "ShimmerBridgeUI";

        protected override void OnCreate(Bundle? savedInstanceState)
        {
            base.OnCreate(savedInstanceState);

            try
            {
                SetContentView(AndroidResource.Layout.activity_main);

                _deviceGroup = RequireView<LinearLayout>(AndroidResource.Id.deviceGroup, "deviceGroup");
                _status = RequireView<TextView>(AndroidResource.Id.txtStatus, "txtStatus");
                _btnStart = RequireView<Button>(AndroidResource.Id.btnStart, "btnStart");
                _btnStop = RequireView<Button>(AndroidResource.Id.btnStop, "btnStop");

                _ws.Log += s => RunOnUiThread(() => _status.Text = "Status: " + s);

                _btnStart.Click += async (_, __) => await ConnectAndStartAsync();
                _btnStop.Click += async (_, __) => await StopAllWithNoticeAsync();

                EnsureRuntimePermissions();
                PopulateBondedDevices();
            }
            catch (Exception ex)
            {
                Android.Util.Log.Error(TAG, "Init error:\n" + ex);
                Toast.MakeText(this, ex.Message, ToastLength.Long).Show();
            }
        }

        // helper to require view by id
        T RequireView<T>(int id, string name) where T : class
        {
            var v = FindViewById(id) as T;
            if (v == null) throw new NullReferenceException($"View '{name}' not found in activity_main.axml");
            return v;
        }

        void EnsureRuntimePermissions()
        {
            if (Build.VERSION.SdkInt >= BuildVersionCodes.S)
            {
                var needed = new[]
                {
                    Android.Manifest.Permission.BluetoothScan,
                    Android.Manifest.Permission.BluetoothConnect,
                    Android.Manifest.Permission.AccessFineLocation
                }.Where(p => CheckSelfPermission(p) != Permission.Granted).ToArray();

                if (needed.Length > 0) RequestPermissions(needed, 42);
            }
            else
            {
                if (CheckSelfPermission(Android.Manifest.Permission.AccessFineLocation) != Permission.Granted)
                    RequestPermissions(new[] { Android.Manifest.Permission.AccessFineLocation }, 42);
            }
        }

        void PopulateBondedDevices()
        {
            _devices.Clear();
            _deviceGroup.RemoveAllViews();

            var bt = BluetoothAdapter.DefaultAdapter;
            if (bt == null)
            {
                Android.Util.Log.Warn(TAG, "BluetoothAdapter.DefaultAdapter == null (emulator?)");
                _status.Text = "Status: no Bluetooth adapter (use a physical device)";
                return;
            }

            var paired = bt.BondedDevices?.Where(d => d?.Name != null).ToList() ?? new List<BluetoothDevice>();
            if (paired.Count == 0)
            {
                var tv = new TextView(this) { Text = "No paired devices. Pair RN-42/Shimmer in Android settings (PIN 1234)." };
                _deviceGroup.AddView(tv);
                return;
            }

            foreach (var d in paired
                     .Where(d => LooksLikeShimmer(d.Name, d.Address))
                     .OrderBy(d => d.Name ?? d.Address))
            {
                var ui = BuildDeviceCard(d);
                _devices.Add(ui);
                _deviceGroup.AddView(ui.Root);
            }
        }

        static bool LooksLikeShimmer(string? name, string? mac)
        {
            string n = (name ?? "").ToUpperInvariant();
            string m = (mac ?? "").ToUpperInvariant();
            if (n.Contains("SHIMMER")) return true;
            if (n.StartsWith("RNBT") || n.StartsWith("RN42") || n.StartsWith("RN-42")) return true;
            if (m.StartsWith("00:06:66")) return true; // Microchip/Roving
            return false;
        }

        DeviceUi BuildDeviceCard(BluetoothDevice d)
        {
            // outer card
            var card = new LinearLayout(this)
            {
                Orientation = Orientation.Vertical
            };
            card.SetPadding(Dp(12), Dp(12), Dp(12), Dp(12));
            var lp = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MatchParent, ViewGroup.LayoutParams.WrapContent)
            {
                TopMargin = Dp(8),
                BottomMargin = Dp(8)
            };
            card.LayoutParameters = lp;
            card.SetBackgroundColor(Android.Graphics.Color.ParseColor("#EFE9E3")); // soft beige

            // title row: name [MAC]  +  [ ] Connect
            var titleRow = new LinearLayout(this) { Orientation = Orientation.Horizontal };
            var title = new TextView(this)
            {
                Text = $"{d.Name} [{d.Address}]",
                TextSize = 18
            };
            title.SetTypeface(title.Typeface, Android.Graphics.TypefaceStyle.Bold);

            var cbConnect = new CheckBox(this) { Text = "Connect", Checked = false };

            titleRow.AddView(title, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WrapContent, 1f));
            titleRow.AddView(cbConnect, new LinearLayout.LayoutParams(ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent));
            card.AddView(titleRow);

            // subtitle
            var sub = new TextView(this) { Text = "Select sensors for this device" };
            sub.SetPadding(0, Dp(6), 0, Dp(6));
            card.AddView(sub);

            // sensors (default: checked = true)
            CheckBox NewCb(string text) => new CheckBox(this) { Text = text, Checked = true };

            var lnAcc = NewCb("Low-noise accelerometer");
            var wrAcc = NewCb("Wide-range accelerometer");
            var gyro = NewCb("Gyroscope");
            var mag = NewCb("Magnetometer");
            var press = NewCb("Pressure & Temperature");
            var batt = NewCb("Battery");
            var a6 = NewCb("Ext A6");
            var a7 = NewCb("Ext A7");
            var a15 = NewCb("Ext A15");

            card.AddView(lnAcc);
            card.AddView(wrAcc);
            card.AddView(gyro);
            card.AddView(mag);
            card.AddView(press);
            card.AddView(batt);
            card.AddView(a6);
            card.AddView(a7);
            card.AddView(a15);

            var ui = new DeviceUi(card, d.Name ?? "?", d.Address ?? "?", cbConnect, lnAcc, wrAcc, gyro, mag, press, batt, a6, a7, a15);

            // --- hook: quando cambi i sensori, riconfiguriamo il MAC con debounce
            void Hook(CheckBox cb)
            {
                cb.CheckedChange += (s, e) =>
                {
                    // puoi anche rimuovere questa condizione se vuoi accettare toggles "pre-connessione"
                    if (!ui.Connect.Checked) return;
                    DebouncedReconfigure(ui.Mac, () => BuildConfigFromUi(ui));
                };
            }
            Hook(lnAcc); Hook(wrAcc); Hook(gyro); Hook(mag); Hook(press); Hook(batt); Hook(a6); Hook(a7); Hook(a15);

            return ui;
        }

        // Costruisce la config corrente dai checkbox della card
        ShimmerConfig BuildConfigFromUi(DeviceUi u) => new ShimmerConfig
        {
            EnableLowNoiseAccelerometer = u.LnAcc.Checked,
            EnableWideRangeAccelerometer = u.WrAcc.Checked,
            EnableGyroscope = u.Gyro.Checked,
            EnableMagnetometer = u.Mag.Checked,
            EnablePressureTemperature = u.Press.Checked,
            EnableBattery = u.Batt.Checked,
            EnableExtA6 = u.A6.Checked,
            EnableExtA7 = u.A7.Checked,
            EnableExtA15 = u.A15.Checked,
            SamplingRate = null // opzionale: aggancia qui un controllo SR se lo aggiungi in UI
        };

        // Debounce lato UI per non tempestare il server mentre l’utente clicca più caselle
        void DebouncedReconfigure(string mac, Func<ShimmerConfig> build)
        {
            if (_debouncers.TryGetValue(mac, out var t))
            {
                try { t.Stop(); t.Dispose(); } catch { }
            }
            var timer = new STimers.Timer(250) { AutoReset = false };
            timer.Elapsed += async (_, __) =>
            {
                try
                {
                    var cfg = build();
                    await _ws.UpdateConfigAsync(mac, cfg); // se mask=0, lo stream viene fermato dal server
                    RunOnUiThread(() => _status.Text = $"Status: reconfigured {mac}");
                }
                catch (Exception ex)
                {
                    RunOnUiThread(() => _status.Text = $"Status: reconfig error {ex.Message}");
                }
            };
            _debouncers[mac] = timer;
            timer.Start();
        }

        int Dp(int dp) => (int)(dp * Resources.DisplayMetrics.Density + 0.5f);

        async Task ConnectAndStartAsync()
        {
            // Collect selected devices (those with Connect checked)
            var selected = _devices
                .Where(x => x.Connect.Checked)
                .ToList();

            if (selected.Count == 0)
            {
                Toast.MakeText(this, "Please select at least one device.", ToastLength.Long).Show();
                _status.Text = "Status: no device selected";
                return;
            }

            // Validate sensori per device (almeno uno all’avvio)
            var noSensor = selected.Where(x => !AnySensorEnabled(x)).ToList();
            if (noSensor.Count > 0)
            {
                var names = string.Join("\n", noSensor.Select(x => $"- {x.Name} [{x.Mac}]"));
                new Android.App.AlertDialog.Builder(this)
                    .SetTitle("No sensors selected")
                    .SetMessage($"Enable at least one sensor for:\n{names}")
                    .SetPositiveButton("OK", (s, e) => { })
                    .Show();
                _status.Text = "Status: sensor selection required";
                return;
            }

            // Build configs
            var targets = selected.Select(x => (x, Cfg: BuildConfigFromUi(x))).ToList();

            // Start WS server
            await _ws.StartAsync(this, 8787);
            _status.Text = "Status: starting selected devices…";

            // Progress dialog
            var progress = new ProgressBar(this) { Indeterminate = true };
            var dlg = new AlertDialog.Builder(this)
                .SetTitle("Connecting…")
                .SetView(progress)
                .SetCancelable(false)
                .Create();
            dlg.Show();

            var results = new List<(string name, string mac, bool ok, string? error)>();
            try
            {
                // sequential open su RN-42
                foreach (var t in targets)
                {
                    try
                    {
                        await Task.Run(async () => await _ws.OpenConfigureAndStartAsync(t.x.Mac, t.Cfg));
                        results.Add((t.x.Name, t.x.Mac, true, null));
                    }
                    catch (Exception ex)
                    {
                        results.Add((t.x.Name, t.x.Mac, false, ex.Message));
                    }
                }

                var okCount = results.Count(r => r.ok);
                var errCount = results.Count - okCount;
                _status.Text = $"Status: streaming {okCount} device(s); errors: {errCount}";
            }
            finally
            {
                try { dlg.Dismiss(); } catch { }
            }

            var msg = string.Join("\n", results.Select(r => r.ok
                ? $"✓ {r.name} [{r.mac}] — OK"
                : $"✗ {r.name} [{r.mac}] — {r.error}"));
            new AlertDialog.Builder(this)
                .SetTitle("Connection result")
                .SetMessage(msg)
                .SetPositiveButton("OK", (s, e) => { })
                .Show();
        }

        // Helper: almeno un sensore spuntato per la validazione iniziale
        static bool AnySensorEnabled(DeviceUi d) =>
            d.LnAcc.Checked || d.WrAcc.Checked || d.Gyro.Checked ||
            d.Mag.Checked || d.Press.Checked || d.Batt.Checked ||
            d.A6.Checked || d.A7.Checked || d.A15.Checked;

        private async Task StopAllWithNoticeAsync()
        {
            int before = _ws.ActiveSessionCount;

            // progress dialog
            var pb = new ProgressBar(this) { Indeterminate = true };
            var dlg = new AlertDialog.Builder(this)
                .SetTitle("Stopping…")
                .SetView(pb)
                .SetCancelable(false)
                .Create();
            dlg.Show();

            try
            {
                await _ws.CloseAllAsync();
            }
            finally
            {
                try { dlg.Dismiss(); } catch { }
            }

            _status.Text = "Status: Idle";

            new AlertDialog.Builder(this)
                .SetTitle("Stopped")
                .SetMessage(before > 0
                    ? $"Streaming stopped on {before} device(s)."
                    : "No active streams.")
                .SetPositiveButton("OK", (s, e) => { })
                .Show();
        }

        protected override async void OnDestroy()
        {
            base.OnDestroy();
            // stop debounce timers
            foreach (STimers.Timer t in _debouncers.Values)
            {
                try { t.Stop(); t.Dispose(); } catch { }
            }
            _debouncers.Clear();

            await _ws.StopAsync(); // closes WS & all sessions
        }

        sealed class DeviceUi
        {
            public View Root { get; }
            public string Name { get; }
            public string Mac { get; }

            public CheckBox Connect { get; }  // per-device connect toggle

            public CheckBox LnAcc { get; }
            public CheckBox WrAcc { get; }
            public CheckBox Gyro { get; }
            public CheckBox Mag { get; }
            public CheckBox Press { get; }
            public CheckBox Batt { get; }
            public CheckBox A6 { get; }
            public CheckBox A7 { get; }
            public CheckBox A15 { get; }

            public DeviceUi(View root, string name, string mac, CheckBox connect,
                CheckBox ln, CheckBox wr, CheckBox gy, CheckBox mg, CheckBox pr,
                CheckBox bt, CheckBox a6, CheckBox a7, CheckBox a15)
            {
                Root = root; Name = name; Mac = mac;
                Connect = connect;
                LnAcc = ln; WrAcc = wr; Gyro = gy; Mag = mg; Press = pr; Batt = bt; A6 = a6; A7 = a7; A15 = a15;
            }
        }
    }
}
