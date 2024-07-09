using System;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json.Linq;
using System.Diagnostics;

namespace test_ros2
{
    public partial class Form1 : Form
    {
        private readonly string localIP;
        private Publisher? publisher;
        private Subscriber? subscriber;
        private TaskCompletionSource<bool>? tcs;
        private readonly object tcsLock = new object();
        private Stopwatch joint1Stopwatch = new Stopwatch();
        private Stopwatch joint2Stopwatch = new Stopwatch();
        private Stopwatch joint3Stopwatch = new Stopwatch();
        private Stopwatch joint4Stopwatch = new Stopwatch();
        private Stopwatch joint5Stopwatch = new Stopwatch();
        private Stopwatch joint6Stopwatch = new Stopwatch();
        private Stopwatch moveForwardStopwatch = new Stopwatch();

        public Form1()
        {
            InitializeComponent();

            localIP = GetLocalIPAddress();
            if (string.IsNullOrEmpty(localIP))
            {
                MessageBox.Show("Failed to get local IP address");
                Close();
                return;
            }

            publisher = null;
            try
            {
                publisher = new Publisher();
                publisher.Initialize(localIP, 12345);
                MessageBox.Show($"Publisher bound to {localIP}:12345");
                labelIPAddress.Text = $"IP Address: {localIP}:12345";
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Failed to initialize Publisher: {ex.Message}");
                Close();
                return;
            }

            subscriber = null;
            try
            {
                subscriber = new Subscriber();
                subscriber.Initialize("192.168.0.143", 12346);
                subscriber.MessageReceived += Subscriber_MessageReceived;
                MessageBox.Show($"Subscribed to /cylinder/ticks topic via ZeroMQ");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Failed to initialize Subscriber: {ex.Message}");
                Close();
                return;
            }
        }

        private void Subscriber_MessageReceived(object? sender, string message)
        {
            try
            {
                var jsonData = JObject.Parse(message);
                string? topic = jsonData["topic"]?.ToString();

                if (topic == "/cylinder/ticks")
                {
                    int data = (int)(jsonData["message"]?["data"] ?? 0);
                    Invoke(new Action(() =>
                    {
                        labelTicks.Text = $"Cylinder Height: {data} cm";
                    }));
                }

                if (topic == "/joint_states")
                {
                    var positions = jsonData["message"]?["positions"]?.ToObject<double[]>();
                    if (positions != null && positions.Length >= 6)
                    {
                        Invoke(new Action(() =>
                        {
                            joint1_value.Text = $"joint1_value: {positions[0] * 180 / 3.14}";
                            joint2_value.Text = $"joint2_value: {positions[1] * 180 / 3.14}";
                            joint3_value.Text = $"joint3_value: {positions[2] * 180 / 3.14}";
                            joint4_value.Text = $"joint4_value: {positions[3] * 180 / 3.14}";
                            joint5_value.Text = $"joint5_value: {positions[4] * 180 / 3.14}";
                            joint6_value.Text = $"joint6_value: {positions[5] * 180 / 3.14}";

                            lock (tcsLock)
                            {
                                if (tcs != null && !tcs.Task.IsCompleted)
                                {
                                    tcs.SetResult(true);
                                }

                                manipulator_connect.Enabled = false;
                            }
                        }));
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error handling message: {ex.Message}");
            }
        }

        private async void manipulator_connect_Click(object sender, EventArgs e)
        {
            lock (tcsLock)
            {
                tcs = new TaskCompletionSource<bool>();
            }

            var message = new
            {
                topic = "manipulator_connect",
                message = new
                {
                    data = 1
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("manipulator_connect", jsonMessage);

            SetAllButtonsEnabled(false);
            manipulator_connect.Enabled = false;

            using (var connectingBox = new Form())
            {
                var label = new Label
                {
                    Text = "Connecting...",
                    AutoSize = true,
                    Location = new System.Drawing.Point(10, 10)
                };
                connectingBox.Controls.Add(label);
                connectingBox.StartPosition = FormStartPosition.CenterScreen;
                connectingBox.FormBorderStyle = FormBorderStyle.FixedDialog;
                connectingBox.ControlBox = false;
                connectingBox.Show();

                var delayTask = Task.Delay(10000);
                var completedTask = await Task.WhenAny(tcs.Task, delayTask);

                connectingBox.Close();

                if (completedTask == delayTask)
                {
                    MessageBox.Show("Failed to receive joint values. Please check the robot.");
                    manipulator_connect.Enabled = true;
                }
                else
                {
                    SetAllButtonsEnabled(true);
                    manipulator_connect.Enabled = false;
                }
            }
        }

        private void SetAllButtonsEnabled(bool enabled)
        {
            cylinder_up.Enabled = enabled;
            cylinder_stop.Enabled = enabled;
            cylinder_down.Enabled = enabled;
            emergency_stop.Enabled = enabled;
            if (enabled)
            {
                manipulator_connect.Enabled = true;
            }
        }

        private void PublishJoyCommand(string topic, sbyte commandValue)
        {
            publisher?.Publish(topic, commandValue);
        }

        private void PublishTopicMessage(string topic, string message)
        {
            publisher?.Publish(topic, message);
        }

        private string GetLocalIPAddress()
        {
            var host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var ip in host.AddressList)
            {
                if (ip.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                {
                    return ip.ToString();
                }
            }
            return string.Empty;
        }

        protected override void OnFormClosed(FormClosedEventArgs e)
        {
            base.OnFormClosed(e);
            DisposeResources();
        }

        private void DisposeResources()
        {
            publisher?.Dispose();
            subscriber?.Dispose();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            PublishJoyCommand("/joy_command", 8);
        }

        private void button2_Click(object sender, EventArgs e)
        {
            PublishJoyCommand("/joy_command", 7);
        }

        private void button3_Click(object sender, EventArgs e)
        {
            PublishJoyCommand("/joy_command", 9);
        }

        private void emergency_stop_Click(object sender, EventArgs e)
        {
            PublishJoyCommand("/joy_command", 5);
        }
        /// <summary>
        /// /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// </summary>
        private void joint1_value_up_Click(object sender, EventArgs e)
        {
            joint1Stopwatch.Start();
        }

        private async void joint1_value_up_MouseDown(object sender, MouseEventArgs e)
        {
            joint1Stopwatch.Restart();

            while (true)
            {
                if (!joint1Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint1Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint1Value += elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint1_value_up_MouseUp(object sender, MouseEventArgs e)
        {
            joint1Stopwatch.Stop();
        }

        private void joint1_value_down_Click(object sender, EventArgs e)
        {
            joint1Stopwatch.Start();
        }

        private async void joint1_value_down_MouseDown(object sender, MouseEventArgs e)
        {
            joint1Stopwatch.Restart();

            while (true)
            {
                if (!joint1Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint1Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint1Value -= elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint1_value_down_MouseUp(object sender, MouseEventArgs e)
        {
            joint1Stopwatch.Stop();
        }
        /// <summary>
        /// /////////////////////////////////////////////////////////////////////////////////////////////////
        /// </summary>
        private void joint2_value_up_Click(object sender, EventArgs e)
        {
            joint2Stopwatch.Start();
        }

        private async void joint2_value_up_MouseDown(object sender, MouseEventArgs e)
        {
            joint2Stopwatch.Restart();

            while (true)
            {
                if (!joint2Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint2Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint2Value += elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint2_value_up_MouseUp(object sender, MouseEventArgs e)
        {
            joint2Stopwatch.Stop();
        }

        private void joint2_value_down_Click(object sender, EventArgs e)
        {
            joint2Stopwatch.Start();
        }

        private async void joint2_value_down_MouseDown(object sender, MouseEventArgs e)
        {
            joint2Stopwatch.Restart();

            while (true)
            {
                if (!joint2Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint2Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint2Value -= elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint2_value_down_MouseUp(object sender, MouseEventArgs e)
        {
            joint2Stopwatch.Stop();
        }
        /// <summary>
        /// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// </summary>
        private void joint3_value_up_Click(object sender, EventArgs e)
        {
            joint3Stopwatch.Start();
        }

        private async void joint3_value_up_MouseDown(object sender, MouseEventArgs e)
        {
            joint3Stopwatch.Restart();

            while (true)
            {
                if (!joint3Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint3Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint3Value += elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint3_value_up_MouseUp(object sender, MouseEventArgs e)
        {
            joint3Stopwatch.Stop();
        }

        private void joint3_value_down_Click(object sender, EventArgs e)
        {
            joint3Stopwatch.Start();
        }

        private async void joint3_value_down_MouseDown(object sender, MouseEventArgs e)
        {
            joint3Stopwatch.Restart();

            while (true)
            {
                if (!joint3Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint3Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint3Value -= elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint3_value_down_MouseUp(object sender, MouseEventArgs e)
        {
            joint3Stopwatch.Stop();
        }
        /// <summary>
        /// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// </summary>
        private void joint4_value_up_Click(object sender, EventArgs e)
        {
            joint4Stopwatch.Start();
        }

        private async void joint4_value_up_MouseDown(object sender, MouseEventArgs e)
        {
            joint4Stopwatch.Restart();

            while (true)
            {
                if (!joint4Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint4Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint4Value += elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint4_value_up_MouseUp(object sender, MouseEventArgs e)
        {
            joint4Stopwatch.Stop();
        }

        private void joint4_value_down_Click(object sender, EventArgs e)
        {
            joint4Stopwatch.Start();
        }

        private async void joint4_value_down_MouseDown(object sender, MouseEventArgs e)
        {
            joint4Stopwatch.Restart();

            while (true)
            {
                if (!joint4Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint4Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint4Value -= elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint4_value_down_MouseUp(object sender, MouseEventArgs e)
        {
            joint4Stopwatch.Stop();
        }
        /// <summary>
        /// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// </summary>
        private void joint5_value_up_Click(object sender, EventArgs e)
        {
            joint5Stopwatch.Start();
        }

        private async void joint5_value_up_MouseDown(object sender, MouseEventArgs e)
        {
            joint5Stopwatch.Restart();

            while (true)
            {
                if (!joint5Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint5Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint5Value += elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint5_value_up_MouseUp(object sender, MouseEventArgs e)
        {
            joint5Stopwatch.Stop();
        }

        private void joint5_value_down_Click(object sender, EventArgs e)
        {
            joint5Stopwatch.Start();
        }

        private async void joint5_value_down_MouseDown(object sender, MouseEventArgs e)
        {
            joint5Stopwatch.Restart();

            while (true)
            {
                if (!joint5Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint5Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint5Value -= elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint5_value_down_MouseUp(object sender, MouseEventArgs e)
        {
            joint5Stopwatch.Stop();
        }
        /// <summary>
        /// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// </summary>
        private void joint6_value_up_Click(object sender, EventArgs e)
        {
            joint6Stopwatch.Start();
        }

        private async void joint6_value_up_MouseDown(object sender, MouseEventArgs e)
        {
            joint6Stopwatch.Restart();

            while (true)
            {
                if (!joint6Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint6Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint6Value += elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint6_value_up_MouseUp(object sender, MouseEventArgs e)
        {
            joint6Stopwatch.Stop();
        }

        private void joint6_value_down_Click(object sender, EventArgs e)
        {
            joint6Stopwatch.Start();
        }

        private async void joint6_value_down_MouseDown(object sender, MouseEventArgs e)
        {
            joint6Stopwatch.Restart();

            while (true)
            {
                if (!joint6Stopwatch.IsRunning)
                {
                    break;
                }

                double elapsedSeconds = joint6Stopwatch.Elapsed.TotalSeconds;

                double joint1Value = double.Parse(joint1_value.Text.Replace("joint1_value: ", ""));
                double joint2Value = double.Parse(joint2_value.Text.Replace("joint2_value: ", ""));
                double joint3Value = double.Parse(joint3_value.Text.Replace("joint3_value: ", ""));
                double joint4Value = double.Parse(joint4_value.Text.Replace("joint4_value: ", ""));
                double joint5Value = double.Parse(joint5_value.Text.Replace("joint5_value: ", ""));
                double joint6Value = double.Parse(joint6_value.Text.Replace("joint6_value: ", ""));

                joint6Value -= elapsedSeconds;

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = new double[] { joint1Value, joint2Value, joint3Value, joint4Value, joint5Value, joint6Value }
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                PublishTopicMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }

        private void joint6_value_down_MouseUp(object sender, MouseEventArgs e)
        {
            joint6Stopwatch.Stop();
        }
        /// <summary>
        /// ////////////////////////조인트 끝 AMR 방향제어 시작
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        // Add this block of code
        private void move_forward_MouseDown(object sender, MouseEventArgs e)
        {
            moveForwardStopwatch.Start();
            PublishMoveForwardCommand(0.5);
        }

        private async void move_forward_MouseDownAsync(object sender, MouseEventArgs e)
        {
            moveForwardStopwatch.Restart();

            while (true)
            {
                if (!moveForwardStopwatch.IsRunning)
                {
                    break;
                }

                PublishMoveForwardCommand(0.5);

                await Task.Delay(100);
            }
        }

        private void move_forward_MouseUp(object sender, MouseEventArgs e)
        {
            moveForwardStopwatch.Stop();
            PublishMoveForwardCommand(0.0);
        }

        private void PublishMoveForwardCommand(double value)
        {
            var message = new
            {
                topic = "/request_move_forward",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_forward", jsonMessage);
        }
        ////////////////////////////끝
    }

    public class Publisher : IDisposable
    {
        private PublisherSocket publisherSocket;
        public string LocalEndpoint { get; private set; } = "";

        public Publisher()
        {
            publisherSocket = new PublisherSocket();
        }

        public void Initialize(string ipAddress, int port)
        {
            publisherSocket.Bind($"tcp://{ipAddress}:{port}");
            LocalEndpoint = $"{ipAddress}:{port}";
        }

        public void Publish(string topic, sbyte commandValue)
        {
            var message = new Int8 { data = commandValue };
            var topicMessage = new
            {
                topic = topic,
                message = message
            };

            byte[] byteMessage = Serialize(topicMessage);
            publisherSocket.SendFrame(byteMessage);
        }

        public void Publish(string topic, string message)
        {
            byte[] byteMessage = Encoding.UTF8.GetBytes(message);
            publisherSocket.SendFrame(byteMessage);
        }

        public void Dispose()
        {
            publisherSocket?.Close();
            publisherSocket?.Dispose();
        }

        private byte[] Serialize(object message)
        {
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            return Encoding.UTF8.GetBytes(jsonMessage);
        }
    }

    public class Subscriber : IDisposable
    {
        private SubscriberSocket subscriberSocket;
        private Task? receiveTask;
        private bool isRunning;

        public event EventHandler<string>? MessageReceived;

        public Subscriber()
        {
            subscriberSocket = new SubscriberSocket();
        }

        public void Initialize(string ipAddress, int port)
        {
            subscriberSocket.Connect($"tcp://{ipAddress}:{port}");
            subscriberSocket.Subscribe("");
            isRunning = true;
            receiveTask = Task.Run(() => ReceiveMessages());
        }

        private void ReceiveMessages()
        {
            while (isRunning)
            {
                try
                {
                    var message = subscriberSocket.ReceiveFrameString();
                    if (message != null)
                    {
                        MessageReceived?.Invoke(this, message);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error receiving message: {ex.Message}");
                }
            }
        }

        public void Dispose()
        {
            isRunning = false;
            subscriberSocket?.Close();
            subscriberSocket?.Dispose();
        }
    }

    public class Int8
    {
        public sbyte data { get; set; }
    }
}