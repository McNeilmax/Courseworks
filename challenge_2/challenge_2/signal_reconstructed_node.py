import rclpy
from rclpy.node import Node
import numpy as np
from scipy import signal
from std_msgs.msg import String, Int32, Float32
from cstm_msgs.msg import SignalInfo

class SignalReconstructed(Node):
    def __init__(self):
        super().__init__('signal_reconstructed_node')
        #Create Subscribers 
        self.sg_sub = self.create_subscription(SignalInfo, 'signal_params', self.sf_subscriber_callback, 10)

        #Create Publishers
        self.sr_pub = self.create_publisher(Float32, 'signal_reconstructed', 10)

        #Callbacks for handling signals and publishing
        self.sreconstructed_handler_period = 0.001
        self.sreconstructed_handler = self.create_timer(self.sreconstructed_handler_period, self.sreconstructed_handler_callback)

        #Variables for reconstructing signal
        self.signal_var = Float32() 
        self.signal_params_var = SignalInfo()
        self.signal_params_var.type = 1
        self.signal_params_var.amplitude = 0.0
        self.signal_params_var.frequency = 0.0
        self.signal_params_var.offset = 0.0
        self.time_var = 0

        #Message for succesful node creation
        self.get_logger().info('signal_reconstructed_node Initialized')

    #Callbacks
    def sf_subscriber_callback(self, msg):
         self.signal_params_var = msg
#         msg.type = signal_params_var.type
#         msg.amplitude = signal_params_var.amplitude
#         msg.frequency = signal_params_var.frequency
#         msg.offset = signal_params_var.offset

        
    def sreconstructed_handler_callback(self)
        self.time_var += self.sreconstructed_handler_period
        #Creating function
        frequency2radiansPersecond = (self.time_var * self.signal_params_var.frequency * np.pi * 2)
        if(self.signal_params_var.type == 1):
            self.signal_var.data = np.sin(frequency2radiansPersecond + self.signal_params_var.offset) * self.signal_params_var.amplitude        
        elif(self.signal_params_var.type == 2):
            t = [frequency2radiansPersecond + self.signal_params_var.offset]
            self.signal_var.data = signal.square(t, duty=0.5)[0] * self.signal_params_var.amplitude
        elif(self.signal_params_var.type == 3):
            t = [frequency2radiansPersecond + self.signal_params_var.offset]
            self.signal_var.data = signal.sawtooth(t)[0] * self.signal_params_var.amplitude
        elif(self.signal_params_var.type == 4):
            t = [frequency2radiansPersecond + self.signal_params_var.offset]
            self.signal_var.data = (signal.square(t, duty=0.5)[0] * self.signal_params_var.amplitude) + (signal.sawtooth(t)[0] * self.signal_params_var.amplitude)
        elif(self.signal_params_var.type == 5):
            t = [frequency2radiansPersecond + self.signal_params_var.offset]
            self.signal_var.data = (signal.square(t, duty=0.5)[0] * self.signal_params_var.amplitude) - (signal.sawtooth(t)[0] * self.signal_params_var.amplitude)

        #Publishing signal
        self.sr_pub.publish(self.signal_var)



def main(args=None):
    rclpy.init(args=args)
    sr = SignalReconstructed()
    rclpy.spin(sr)
    sr.SignalReconstructed()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

