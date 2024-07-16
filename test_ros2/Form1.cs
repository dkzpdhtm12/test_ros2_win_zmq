using System;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json.Linq;
using System.Diagnostics;
using System.Drawing.Printing;

namespace test_ros2
{
    public partial class Form1 : Form
    {
        private readonly string localIP;
        private Publisher? publisher;
        private Subscriber? subscriber;
        private TaskCompletionSource<bool>? tcs;
        private readonly object tcsLock = new object();
        private Stopwatch moveForwardStopwatch = new Stopwatch();
        private Stopwatch moveBackwardStopwatch = new Stopwatch();
        private Stopwatch moveLeftStopwatch = new Stopwatch();
        private Stopwatch moveRightStopwatch = new Stopwatch();
        private Stopwatch leftSpinStopwatch = new Stopwatch();
        private Stopwatch rightSpinStopwatch = new Stopwatch();
        private bool isManualMode;
        private JointController jointController;

        public Form1()
        {
            InitializeComponent();

            Label[] jointLabels = new Label[]
            {
                joint1_value, joint2_value, joint3_value,
                joint4_value, joint5_value, joint6_value
            };

            jointController = new JointController(jointLabels, PublishTopicMessage);

            Button[,] jointButtons = new Button[,]
            {
                { joint1_value_up, joint1_value_down },
                { joint2_value_up, joint2_value_down },
                { joint3_value_up, joint3_value_down },
                { joint4_value_up, joint4_value_down },
                { joint5_value_up, joint5_value_down },
                { joint6_value_up, joint6_value_down }
            };

            for (int jointIndex = 0; jointIndex < 6; jointIndex++)
            {
                int targetJoint = jointIndex;
                jointButtons[targetJoint, 0].MouseDown += (s, e) => jointController.JointUpMouseDown(s, e, targetJoint);
                jointButtons[targetJoint, 0].MouseUp += jointController.JointMouseUp;
                jointButtons[targetJoint, 1].MouseDown += (s, e) => jointController.JointDownMouseDown(s, e, targetJoint);
                jointButtons[targetJoint, 1].MouseUp += jointController.JointMouseUp;
            }

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
                subscriber.Initialize("192.168.0.143", 12346); //91
                subscriber.MessageReceived += Subscriber_MessageReceived;
                MessageBox.Show($"Subscribed to ROS2 robot topics via ZeroMQ");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Failed to initialize Subscriber: {ex.Message}");
                Close();
                return;
            }

            using (var modeSelectionForm = new Form2())
            {
                var result = modeSelectionForm.ShowDialog();
                if (result == DialogResult.OK)
                {
                    isManualMode = modeSelectionForm.IsManual;
                    if (isManualMode)
                    {
                        MessageBox.Show("Manual mode selected.");
                        SetAllButtonsEnabled(true);
                        auto_drive_button.Enabled = false;
                    }
                    else
                    {
                        MessageBox.Show("Automatic mode selected.");
                        SetAllButtonsEnabled(false);
                        auto_drive_button.Enabled = true;
                    }
                }
                else
                {
                    MessageBox.Show("No mode selected, closing application.");
                    Close();
                }
            }

            SetAllJointControlButtonsEnabled(false);
        }

        private void Subscriber_MessageReceived(object? sender, string message)
        {
            try
            {
                var jsonData = JObject.Parse(message);
                string? topic = jsonData["topic"]?.ToString();

                switch (topic)
                {
                    case "/cylinder/ticks":
                        int data = (int)(jsonData["message"]?["data"] ?? 0);
                        Invoke(new Action(() =>
                        {
                            labelTicks.Text = $"Cylinder Height: {data} cm";
                        }));
                        break;

                    case "/joint_states":
                        var positions = jsonData["message"]?["positions"]?.ToObject<double[]>();
                        if (positions != null && positions.Length >= 6)
                        {
                            Invoke(new Action(() =>
                            {
                                Label[] jointLabels = { joint1_value, joint2_value, joint3_value, joint4_value, joint5_value, joint6_value };

                                for (int i = 0; i < jointLabels.Length; i++)
                                {
                                    jointLabels[i].Text = $"joint{i + 1}_value: {positions[i] * 180 / 3.14:F2}";
                                }

                                lock (tcsLock)
                                {
                                    if (tcs != null && !tcs.Task.IsCompleted)
                                    {
                                        tcs.SetResult(true);
                                    }

                                    manipulator_connect.Enabled = false;
                                    SetAllJointControlButtonsEnabled(isManualMode);
                                }
                            }));
                        }
                        break;

                    case "/robot_current_work":
                        string currentWorkData = jsonData["message"]?["data"]?.ToString() ?? string.Empty;
                        Invoke(new Action(() =>
                        {
                            robot_current_work.Text = $"Current: {currentWorkData}";
                        }));
                        break;

                    case "/send_yaml_data":
                        var yamlData = jsonData["message"]?["data"];
                        if (yamlData != null)
                        {
                            Invoke(new Action(() =>
                            {
                                ShowYamlData(yamlData.ToString());
                            }));
                        }
                        break;

                    default:
                        Console.WriteLine($"Unknown topic: {topic}");
                        break;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error handling message: {ex.Message}");
            }
        }

        private void ShowYamlData(string yamlContent)
        {
            Form yamlForm = new Form
            {
                Text = "YAML Data",
                Size = new System.Drawing.Size(400, 600)
            };

            TextBox textBox = new TextBox
            {
                Multiline = true,
                Dock = DockStyle.Fill,
                ScrollBars = ScrollBars.Vertical,
                Text = yamlContent,
                ReadOnly = true
            };

            yamlForm.Controls.Add(textBox);
            yamlForm.ShowDialog();
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
                    Text = "Connecting... wait until 10s...",
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
                    SetAllButtonsEnabled(true);
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
            move_forward.Enabled = enabled;
            move_backward.Enabled = enabled;
            move_left.Enabled = enabled;
            move_right.Enabled = enabled;
            left_spin.Enabled = enabled;
            right_spin.Enabled = enabled;
            area_numUpDown.Enabled = enabled;
            growth_numUpDown.Enabled = enabled;
            save_yaml.Enabled = enabled;
            confirmation_signal_button.Enabled = enabled;
            photo_done.Enabled = enabled;
            if (enabled)
            {
                manipulator_connect.Enabled = true;
            }
        }

        private void SetAllJointControlButtonsEnabled(bool enabled)
        {
            joint1_value_up.Enabled = enabled;
            joint2_value_up.Enabled = enabled;
            joint3_value_up.Enabled = enabled;
            joint4_value_up.Enabled = enabled;
            joint5_value_up.Enabled = enabled;
            joint6_value_up.Enabled = enabled;
            joint1_value_down.Enabled = enabled;
            joint2_value_down.Enabled = enabled;
            joint3_value_down.Enabled = enabled;
            joint4_value_down.Enabled = enabled;
            joint5_value_down.Enabled = enabled;
            joint6_value_down.Enabled = enabled;
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
            PublishJoyCommand("/joy_command", 7);
        }
        /// <summary>
        /// ////////////////////////AMR 방향제어 시작 앞쪽
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
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
                topic = "/request_move_vertical",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_vertical", jsonMessage);
        }
        /// <summary>
        /// ////////////////////////조인트 끝 AMR 방향제어 시작 뒤쪽
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void move_backward_MouseDown(object sender, MouseEventArgs e)
        {
            moveBackwardStopwatch.Start();
            PublishMoveBackwardCommand(-0.5);
        }

        private async void move_backward_MouseDownAsync(object sender, MouseEventArgs e)
        {
            moveBackwardStopwatch.Restart();

            while (true)
            {
                if (!moveBackwardStopwatch.IsRunning)
                {
                    break;
                }

                PublishMoveBackwardCommand(-0.5);

                await Task.Delay(100);
            }
        }

        private void move_backward_MouseUp(object sender, MouseEventArgs e)
        {
            moveBackwardStopwatch.Stop();
            PublishMoveBackwardCommand(0.0);
        }

        private void PublishMoveBackwardCommand(double value)
        {
            var message = new
            {
                topic = "/request_move_vertical",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_vertical", jsonMessage);
        }
        /// <summary>
        /// //////////////////////////////////// 왼쪽
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void move_left_MouseDown(object sender, MouseEventArgs e)
        {
            moveLeftStopwatch.Start();
            PublishLeftCommand(0.5);
        }

        private async void move_left_MouseDownAsync(object sender, MouseEventArgs e)
        {
            moveLeftStopwatch.Restart();

            while (true)
            {
                if (!moveLeftStopwatch.IsRunning)
                {
                    break;
                }

                PublishLeftCommand(0.5);

                await Task.Delay(100);
            }
        }

        private void move_left_MouseUp(object sender, MouseEventArgs e)
        {
            moveLeftStopwatch.Stop();
            PublishLeftCommand(0.0);
        }

        private void PublishLeftCommand(double value)
        {
            var message = new
            {
                topic = "/request_move_horizontal",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_horizontal", jsonMessage);
        }
        /// <summary>
        /// ///////////////////////오른쪽
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void move_right_MouseDown(object sender, MouseEventArgs e)
        {
            moveRightStopwatch.Start();
            PublishRightCommand(-0.5);
        }

        private async void move_right_MouseDownAsync(object sender, MouseEventArgs e)
        {
            moveRightStopwatch.Restart();

            while (true)
            {
                if (!moveRightStopwatch.IsRunning)
                {
                    break;
                }

                PublishRightCommand(-0.5);

                await Task.Delay(100);
            }
        }

        private void move_right_MouseUp(object sender, MouseEventArgs e)
        {
            moveRightStopwatch.Stop();
            PublishRightCommand(0.0);
        }

        private void PublishRightCommand(double value)
        {
            var message = new
            {
                topic = "/request_move_horizontal",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_horizontal", jsonMessage);
        }
        /// <summary>
        /// /////////////////////////왼쪽 회전
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void left_spin_MouseDown(object sender, MouseEventArgs e)
        {
            leftSpinStopwatch.Start();
            PublishLeftSipnCommand(0.5);
        }

        private async void left_spin_MouseDownAsync(object sender, MouseEventArgs e)
        {
            leftSpinStopwatch.Restart();

            while (true)
            {
                if (!leftSpinStopwatch.IsRunning)
                {
                    break;
                }

                PublishLeftSipnCommand(0.5);

                await Task.Delay(100);
            }
        }

        private void left_spin_MouseUp(object sender, MouseEventArgs e)
        {
            leftSpinStopwatch.Stop();
            PublishLeftSipnCommand(0.0);
        }

        private void PublishLeftSipnCommand(double value)
        {
            var message = new
            {
                topic = "/request_move_spin",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_spin", jsonMessage);
        }
        /// <summary>
        /// /////////////////////////오른쪽 회전
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void right_spin_MouseDown(object sender, MouseEventArgs e)
        {
            rightSpinStopwatch.Start();
            PublishRightSipnCommand(-0.5);
        }

        private async void right_spin_MouseDownAsync(object sender, MouseEventArgs e)
        {
            rightSpinStopwatch.Restart();

            while (true)
            {
                if (!rightSpinStopwatch.IsRunning)
                {
                    break;
                }

                PublishRightSipnCommand(-0.5);

                await Task.Delay(100);
            }
        }

        private void right_spin_MouseUp(object sender, MouseEventArgs e)
        {
            rightSpinStopwatch.Stop();
            PublishRightSipnCommand(0.0);
        }

        private void PublishRightSipnCommand(double value)
        {
            var message = new
            {
                topic = "/request_move_spin",
                message = new
                {
                    data = value
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/request_move_spin", jsonMessage);
        }
        /// <summary>
        /// ////////////// yaml파일 저장부
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void save_yaml_Click(object sender, EventArgs e)
        {
            int areaValue = (int)area_numUpDown.Value;
            int growthValue = (int)growth_numUpDown.Value;

            var message = new
            {
                topic = "/save_yaml",
                message = new
                {
                    data = new int[] { areaValue, growthValue }
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/save_yaml", jsonMessage);
        }

        private void load_yaml_Click(object sender, EventArgs e)
        {
            var message = new
            {
                topic = "/load_yaml",
                message = new
                {
                    data = 1
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/load_yaml", jsonMessage);
        }

        private void confirmation_signal_button_Click(object sender, EventArgs e)
        {
            var message = new
            {
                topic = "/confirmation_signal",
                message = new
                {
                    data = 1
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/confirmation_signal", jsonMessage);
        }

        private void auto_drive_button_Click(object sender, EventArgs e)
        {
            var message = new
            {
                topic = "/auto_drive_signal",
                message = new
                {
                    data = 1
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/auto_drive_signal", jsonMessage);
        }

        private void photo_done_Click(object sender, EventArgs e)
        {
            var message = new
            {
                topic = "/photo_done",
                message = new
                {
                    data = 1
                }
            };
            string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(message);
            PublishTopicMessage("/photo_done", jsonMessage);
        }
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

    public class JointController
    {
        private Stopwatch jointStopwatch;
        private Label[] jointLabels;
        private Action<string, string> publishMessage;

        public JointController(Label[] jointLabels, Action<string, string> publishMessage)
        {
            this.jointLabels = jointLabels;
            this.jointStopwatch = new Stopwatch();
            this.publishMessage = publishMessage;
        }

        public void JointUpMouseDown(object? sender, MouseEventArgs e, int jointIndex)
        {
            jointStopwatch.Restart();
            UpdateJointValue(1, jointIndex);
        }

        public void JointDownMouseDown(object? sender, MouseEventArgs e, int jointIndex)
        {
            jointStopwatch.Restart();
            UpdateJointValue(-1, jointIndex);
        }

        public void JointMouseUp(object? sender, MouseEventArgs e)
        {
            jointStopwatch.Stop();
        }

        private async void UpdateJointValue(int direction, int jointIndex)
        {
            while (jointStopwatch.IsRunning)
            {
                double elapsedSeconds = jointStopwatch.Elapsed.TotalSeconds;

                double[] jointValues = new double[jointLabels.Length];
                for (int i = 0; i < jointLabels.Length; i++)
                {
                    jointValues[i] = double.Parse(jointLabels[i].Text.Replace($"joint{i + 1}_value: ", ""));
                }

                jointValues[jointIndex] += direction * elapsedSeconds;

                for (int i = 0; i < jointLabels.Length; i++)
                {
                    jointLabels[i].Text = $"joint{i + 1}_value: {jointValues[i]:F2}";
                }

                var jointRequestMessage = new
                {
                    topic = "/joint_request",
                    message = new
                    {
                        layout = new { dim = new object[] { }, data_offset = 0 },
                        data = jointValues
                    }
                };
                string jsonMessage = Newtonsoft.Json.JsonConvert.SerializeObject(jointRequestMessage);
                publishMessage("/joint_request", jsonMessage);

                await Task.Delay(100);
            }
        }
    }

    public class Int8
    {
        public sbyte data { get; set; }
    }
}