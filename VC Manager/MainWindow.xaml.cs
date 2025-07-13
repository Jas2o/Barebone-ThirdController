using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Pipes;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Valve.VR;
using System.Data;

namespace VC_Manager {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {

        CVRSystem vr;
        readonly string pipeName = "OpenVRvcPipeIn";

        public MainWindow() {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e) {
            GetDevices();
        }

        private void BtnDisable_Click(object sender, RoutedEventArgs e) {
            byte b = 127;
            SendMessage(b);
        }

        private bool SendMessage(byte msg) {
            if (msg == 0 || msg > 127)
                return false;

            NamedPipeClientStream pipeClient = new NamedPipeClientStream(".", pipeName, PipeDirection.InOut, PipeOptions.None);
            pipeClient.Connect(5000);

            if (pipeClient.IsConnected) {
                pipeClient.WriteByte(msg); //Don't do zero!
                pipeClient.Flush();

                //// The other end closes automatically.
                //pipeClient.WaitForPipeDrain();
                //pipeClient.Close();
                return true;
            }
            return false;
        }

        private void BtnGetDevices_Click(object sender, RoutedEventArgs e) {
            GetDevices();
        }

        private void GetDevices() {
            stackDevices.Children.Clear();

            if (vr == null) {
                EVRInitError initError = EVRInitError.None;
                vr = OpenVR.Init(ref initError, EVRApplicationType.VRApplication_Background);
                if (vr == null) //initError == Init_NoServerForBackgroundApp
                    return;
            }

            Thickness marginBottom = new Thickness(0, 0, 0, 5);

            //string lookFor = "LIV Virtual Camera";

            //0 is always the HMD so we skip
            //k_unMaxTrackedDeviceCount in newer versions goes up to 64
            for (byte idx = 1; idx < OpenVR.k_unMaxTrackedDeviceCount; idx++) {
                StringBuilder sb = new StringBuilder();
                ETrackedPropertyError err = ETrackedPropertyError.TrackedProp_NotYetAvailable;
                vr.GetStringTrackedDeviceProperty(idx, ETrackedDeviceProperty.Prop_ModelNumber_String, sb, (uint)sb.MaxCapacity, ref err);
                string name = sb.ToString();

                if (name.Length > 0) {
                    Button btn = new Button() {
                        Content = name,
                        Tag = idx,
                        Margin = marginBottom
                    };
                    btn.Click += BtnSelectDevice_Click;
                    stackDevices.Children.Add(btn);
                }
            }
        }

        private void BtnSelectDevice_Click(object sender, RoutedEventArgs e) {
            byte idx = (byte)((Button)sender).Tag;
            SendMessage((byte)idx);
        }

    }
}
