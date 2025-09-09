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
using Android.Graphics;
using Android.Graphics.Drawables;
using AndroidResource = ShimmerAndroidBridge.Resource;

namespace com.example.shimmerbridge.cs
{
    [Activity(Label = "Shimmer Bridge", Theme = "@style/AppTheme", MainLauncher = true)]
    public class MainActivity : Activity
    {
        LinearLayout _deviceGroup = null!;
        TextView _status = null!;
        Button _btnStart = null!, _btnStop = null!;
        Button _btnScan = null!;

        readonly WsBridgeManager _ws = new WsBridgeManager();
        readonly List<DeviceUi> _devices = new(); // per-device sensors + connect flag
        readonly Dictionary<string, STimers.Timer> _debouncers = new(StringComparer.OrdinalIgnoreCase);

        ShimmerScanManager _scanner = null!;
        Dictionary<string, ShimmerScanManager.DeviceType> _lastTypes =
            new(StringComparer.OrdinalIgnoreCase);

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

                _scanner = new ShimmerScanManager(this);

                EnsureRuntimePermissions();
                AddScanControls();          // bottone Scan programm.
                PopulateBondedDevices();    // cards dei paired
                _ = StartScanNow();         // primo scan e badge
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

        void AddScanControls()
        {
            // Riga con bottone "Refresh"
            var row = new LinearLayout(this) { Orientation = Orientation.Horizontal };
            row.SetPadding(Dp(6), Dp(6), Dp(6), Dp(6));

            _btnScan = new Button(this) { Text = "Refresh" };
            _btnScan.Click += async (_, __) => await StartScanNow();

            row.AddView(_btnScan, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent));

            // Inserisco come primo child del deviceGroup
            _deviceGroup.AddView(row, 0);
        }

        void PopulateBondedDevices()
        {
            // pulizia mantenendo la prima riga di scan (indice 0)
            while (_deviceGroup.ChildCount > 1)
                _deviceGroup.RemoveViewAt(1);

            _devices.Clear();

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
                     .Where(d => Com.Example.ShimmerBridge.ShimmerScanManager.LooksLikeShimmer(d.Name, d.Address))
                     .OrderBy(d => d.Name ?? d.Address))
            {
                var ui = BuildDeviceCard(d);
                _devices.Add(ui);
                _deviceGroup.AddView(ui.Root);
            }
        }



        DeviceUi BuildDeviceCard(BluetoothDevice d)
        {
            // outer card
            var card = new LinearLayout(this) { Orientation = Orientation.Vertical };
            card.SetPadding(Dp(12), Dp(12), Dp(12), Dp(12));
            var lp = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MatchParent, ViewGroup.LayoutParams.WrapContent)
            {
                TopMargin = Dp(8),
                BottomMargin = Dp(8)
            };
            card.LayoutParameters = lp;
            card.SetBackgroundColor(Color.ParseColor("#EFE9E3")); // soft beige

            var titleRow = new LinearLayout(this)
            {
                Orientation = Orientation.Horizontal
            };
            titleRow.SetGravity(GravityFlags.CenterVertical);


            var title = new TextView(this)
            {
                Text = $"{d.Name} [{d.Address}]",
                TextSize = 18
            };
            title.SetTypeface(title.Typeface, TypefaceStyle.Bold);

            var cbConnect = new CheckBox(this) { Text = "Connect", Checked = false };

            var badge = MakeBadgeView("?");

            // ordine: titolo (peso 1) -> connect -> badge (a destra)
            titleRow.AddView(title, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WrapContent, 1f));
            titleRow.AddView(cbConnect, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent)
            {
                RightMargin = Dp(8)
            });
            titleRow.AddView(badge, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent)
            {
                LeftMargin = Dp(8)
            });

            card.AddView(titleRow);

            // EXG mode row (radio) - visibile solo se EXG
            var exgRow = new LinearLayout(this) { Orientation = Orientation.Horizontal, Visibility = ViewStates.Gone };
            var lblExg = new TextView(this) { Text = "EXG mode: " };
            lblExg.SetPadding(0, Dp(8), Dp(8), Dp(8));

            var rg = new RadioGroup(this) { Orientation = Orientation.Horizontal };
            var rbTest = new RadioButton(this) { Text = "EXG Test" };
            var rbECG = new RadioButton(this) { Text = "ECG" };
            var rbEMG = new RadioButton(this) { Text = "EMG" };
            var rbResp = new RadioButton(this) { Text = "Respiration" };
            rg.AddView(rbTest); rg.AddView(rbECG); rg.AddView(rbEMG); rg.AddView(rbResp);
            exgRow.AddView(lblExg);
            exgRow.AddView(rg);
            card.AddView(exgRow);

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

            var ui = new DeviceUi(card, d.Name ?? "?", d.Address ?? "?", cbConnect,
                                  lnAcc, wrAcc, gyro, mag, press, batt, a6, a7, a15,
                                  badge, exgRow, rg);

            // Debounce riconfigurazione al cambio sensori (solo se connesso)
            void Hook(CheckBox cb)
            {
                cb.CheckedChange += (s, e) =>
                {
                    if (!ui.Connect.Checked) return;
                    DebouncedReconfigure(ui.Mac, () => BuildConfigFromUi(ui));
                };
            }
            Hook(lnAcc); Hook(wrAcc); Hook(gyro); Hook(mag); Hook(press); Hook(batt); Hook(a6); Hook(a7); Hook(a15);

            // EXG mode selection (solo UI; opzionale wiring)
            rg.CheckedChange += (s, e) =>
            {
                var id = rg.CheckedRadioButtonId;
                var rb = rg.FindViewById<RadioButton>(id);
                ui.SelectedExgMode = rb?.Text ?? "EXG Test";
            };

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

        async Task StartScanNow()
        {
            try
            {
                _status.Text = "Status: scanning…";
                var res = await _scanner.ScanAsync(TimeSpan.FromSeconds(7));
                _lastTypes = res.Visible.ToDictionary(e => e.Mac, e => e.Type, StringComparer.OrdinalIgnoreCase);
                var offSet = new HashSet<string>(res.Off.Select(e => e.Mac), StringComparer.OrdinalIgnoreCase);

                foreach (var ui in _devices)
                {
                    ShimmerScanManager.DeviceType type;
                    if (_lastTypes.TryGetValue(ui.Mac, out var t))
                        type = (t == ShimmerScanManager.DeviceType.Unknown) ? ShimmerScanManager.DeviceType.IMU : t; // euristica: Unknown -> IMU
                    else if (offSet.Contains(ui.Mac))
                        type = ShimmerScanManager.DeviceType.DeviceOff;
                    else
                        type = ShimmerScanManager.DeviceType.Unknown;

                    ApplyTypeToUi(ui, type);
                }

                _status.Text = "Status: scan complete";
            }
            catch (Exception ex)
            {
                _status.Text = $"Status: scan error {ex.Message}";
            }
        }

        void ApplyTypeToUi(DeviceUi ui, ShimmerScanManager.DeviceType type)
        {
            ui.Type = type;
            SetBadge(ui.Badge, type);
            ui.ExgRow.Visibility = type == ShimmerScanManager.DeviceType.EXG ? ViewStates.Visible : ViewStates.Gone;
        }

        TextView MakeBadgeView(string text)
        {
            var tv = new TextView(this) { Text = text, TextSize = 12 };
            tv.SetTextColor(Color.White);
            var bg = new GradientDrawable();
            bg.SetCornerRadius(Dp(8));
            bg.SetColor(Color.Gray);
            tv.SetPadding(Dp(8), Dp(2), Dp(8), Dp(2));
            tv.Background = bg;
            return tv;
        }
        void SetBadge(TextView tv, ShimmerScanManager.DeviceType type)
        {
            var (txt, col) = type switch
            {
                ShimmerScanManager.DeviceType.IMU => ("IMU", Color.ParseColor("#2E7D32")),
                ShimmerScanManager.DeviceType.EXG => ("EXG", Color.ParseColor("#1565C0")),
                ShimmerScanManager.DeviceType.DeviceOff => ("OFF", Color.ParseColor("#757575")),
                _ => ("?", Color.ParseColor("#EF6C00")),
            };
            tv.Text = txt;
            if (tv.Background is GradientDrawable gd) gd.SetColor(col);
            else
            {
                var bg = new GradientDrawable();
                bg.SetCornerRadius(Dp(8));
                bg.SetColor(col);
                tv.Background = bg;
            }
        }


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
                new AlertDialog.Builder(this)
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

            public TextView Badge { get; }
            public LinearLayout ExgRow { get; }
            public RadioGroup ExgGroup { get; }
            public string SelectedExgMode { get; set; } = "EXG Test";
            public ShimmerScanManager.DeviceType Type { get; set; } = ShimmerScanManager.DeviceType.Unknown;

            public DeviceUi(View root, string name, string mac, CheckBox connect,
                CheckBox ln, CheckBox wr, CheckBox gy, CheckBox mg, CheckBox pr,
                CheckBox bt, CheckBox a6, CheckBox a7, CheckBox a15,
                TextView badge, LinearLayout exgRow, RadioGroup exgGroup)
            {
                Root = root; Name = name; Mac = mac;
                Connect = connect;
                LnAcc = ln; WrAcc = wr; Gyro = gy; Mag = mg; Press = pr; Batt = bt; A6 = a6; A7 = a7; A15 = a15;
                Badge = badge; ExgRow = exgRow; ExgGroup = exgGroup;
            }
        }
    }
}
