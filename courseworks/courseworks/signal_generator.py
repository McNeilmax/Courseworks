import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

timer_T = 0.1

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher = self.create_publisher(Float32, 'time', 10)
        timer_period = timer_T
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        time_msg = Float32()
        time_msg.data = self.t
        self.time_publisher.publish(time_msg)
        
        signal_msg = Float32()
        signal_msg.data = np.sin(time_msg.data)
        self.signal_publisher.publish(signal_msg)

        self.get_logger().info('Time: "%f", sin(t)= "%f"' % (time_msg.data, signal_msg.data))
        self.t += timer_T
    
def main(args=None):
    rclpy.init(args=args)
    s_g = SignalGenerator()
    rclpy.spin(s_g)
    s_g.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
