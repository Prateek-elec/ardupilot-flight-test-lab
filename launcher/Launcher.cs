// FC Tuner launcher: starts MATLAB without its desktop, runs fc_tuner_launch (the tuner
// window) and shows a small "Starting..." splash until the tuner window appears.
// Build: launcher\build_launcher.bat   (uses the C# compiler that ships with Windows)
using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows.Forms;

static class Launcher
{
    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    static extern IntPtr FindWindow(string cls, string title);

    const string AppTitle = "FC PID Tuner";

    [STAThread]
    static void Main()
    {
        Application.EnableVisualStyles();
        string dir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd('\\');
        if (!File.Exists(Path.Combine(dir, "fc_tuner_app.m")))
        {
            // exe moved elsewhere: fall back to the project folder
            string p = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "MATLAB", "fc_test");
            if (File.Exists(Path.Combine(p, "fc_tuner_app.m"))) dir = p;
        }
        if (FindWindow(null, AppTitle) != IntPtr.Zero)
        {
            MessageBox.Show("The FC PID Tuner is already open.", AppTitle, MessageBoxButtons.OK, MessageBoxIcon.Information);
            return;
        }
        string matlab = FindMatlab();
        if (matlab == null)
        {
            MessageBox.Show("MATLAB was not found on this computer.\nThe tuner needs MATLAB (with Control System, UAV and Report Generator toolboxes).",
                AppTitle, MessageBoxButtons.OK, MessageBoxIcon.Error);
            return;
        }
        var psi = new ProcessStartInfo(matlab,
            "-nosplash -nodesktop -minimize -sd \"" + dir + "\" -r \"fc_tuner_launch\"");
        psi.UseShellExecute = false;
        psi.CreateNoWindow = true;
        try { Process.Start(psi); }
        catch (Exception e)
        {
            MessageBox.Show("Could not start MATLAB:\n" + e.Message, AppTitle, MessageBoxButtons.OK, MessageBoxIcon.Error);
            return;
        }
        Application.Run(new Splash());
    }

    static string FindMatlab()
    {
        string[] fixedPaths = { @"D:\matlab\bin\matlab.exe" };
        foreach (var f in fixedPaths) if (File.Exists(f)) return f;
        foreach (var root in new[] { @"C:\Program Files\MATLAB", @"D:\Program Files\MATLAB" })
        {
            if (!Directory.Exists(root)) continue;
            var vers = Directory.GetDirectories(root);
            Array.Sort(vers); Array.Reverse(vers);              // newest release first
            foreach (var v in vers)
            {
                string f = Path.Combine(v, "bin", "matlab.exe");
                if (File.Exists(f)) return f;
            }
        }
        foreach (var d in (Environment.GetEnvironmentVariable("PATH") ?? "").Split(';'))
        {
            try { string f = Path.Combine(d.Trim(), "matlab.exe"); if (File.Exists(f)) return f; } catch { }
        }
        return null;
    }

    class Splash : Form
    {
        readonly Timer timer = new Timer();
        readonly DateTime t0 = DateTime.Now;
        readonly Label status;

        public Splash()
        {
            Text = "Starting " + AppTitle; FormBorderStyle = FormBorderStyle.None; StartPosition = FormStartPosition.CenterScreen;
            Size = new Size(460, 170); BackColor = Color.FromArgb(32, 32, 32); TopMost = true; ShowInTaskbar = true;
            try { Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath); } catch { }
            var title = new Label { Text = AppTitle, ForeColor = Color.White, Font = new Font("Segoe UI", 18, FontStyle.Bold),
                AutoSize = true, Location = new Point(24, 22) };
            status = new Label { Text = "Starting MATLAB... this takes 20-40 seconds", ForeColor = Color.FromArgb(200, 200, 200),
                Font = new Font("Segoe UI", 10), AutoSize = true, Location = new Point(26, 72) };
            var bar = new ProgressBar { Style = ProgressBarStyle.Marquee, Location = new Point(26, 112), Size = new Size(408, 14), MarqueeAnimationSpeed = 30 };
            Controls.Add(title); Controls.Add(status); Controls.Add(bar);
            timer.Interval = 500;
            timer.Tick += (s, e) =>
            {
                if (FindWindow(null, AppTitle) != IntPtr.Zero) { timer.Stop(); Close(); return; }
                double sec = (DateTime.Now - t0).TotalSeconds;
                if (sec > 45) status.Text = "Still starting... MATLAB can be slow the first time";
                if (sec > 180) { timer.Stop(); Close(); }
            };
            timer.Start();
        }
    }
}
