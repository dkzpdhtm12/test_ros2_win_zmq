namespace std_msgs.msg
{
    public class Int8 : Message
    {
        public sbyte data;

        public override string ToString()
        {
            return $"Int8(data={data})";
        }

        public override string MessageType => "std_msgs/msg/Int8";
    }
}