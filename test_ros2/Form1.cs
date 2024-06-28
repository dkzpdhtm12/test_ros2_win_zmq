using System;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json.Linq;

namespace test_ros2
{
    public partial class Form1 : Form
    {
        private readonly string localIP;
        private Publisher? publisher;
        private Subscriber? subscriber;

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
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error handling message: {ex.Message}");
            }
        }
        private void PublishJoyCommand(string topic, sbyte commandValue)
        {
            publisher?.Publish(topic, commandValue);
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