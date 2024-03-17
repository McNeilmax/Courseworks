import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

timer_T = 0.1
alpha = 0.5

class Process(Node):
    def __init__(self):
        super().__init__('process')
        self.time_subscriber = self.create_subscription(Float32, 'time', self.time_callback, 10)
        self.signal_subscriber = self.create_subscription(Float32, 'signal', self.signal_callback, 10)
        
        self.signal_processed_publisher = self.create_publisher(Float32, 'proc_signal', 10)
        
        timer_period = timer_T
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Signal processor node has been started!!!')
        
        self.processed_signal = Float32()
    
    def timer_callback(self):
        self.signal_processed_publisher.publish(self.processed_signal)

    def time_callback(self, msg):
        self.processed_signal.data = np.sin(msg.data - 1.1) / 2 + alpha

    def signal_callback(self, msg):
        self.get_logger().info('Original Signal= "%f", Processed Signal= "%f"' % (msg.data, self.processed_signal.data))

    


def main(args=None):
    rclpy.init(args=args)
    p_s = Process()
    rclpy.spin(p_s)
    p_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()