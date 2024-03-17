import rclpy
from rclpy.node import Node
import numpy as np
from scipy import signal
from std_msgs.msg import String, Int32, Float32
from cstm_msgs.msg import SignalInfo

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator_node')
        
        #Publishers
        self.signal_pub = self.create_publisher(Float32, 'signal', 10)
        self.signal_params_pub = self.create_publisher(SignalInfo, 'signal_params', 10)

        #Parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('type_of_signal', rclpy.Parameter.Type.INTEGER),
                    ('amplitude_param', rclpy.Parameter.Type.DOUBLE),
                    ('frequency_param', rclpy.Parameter.Type.DOUBLE),
                    ('offset_param', rclpy.Parameter.Type.DOUBLE),
                ]
        )

        #Callbacks for handling signals and publishing
        self.signal_handler_period = 0.01
        self.signal_handler = self.create_timer(self.signal_handler_period, self.signal_handler_callback)

        #Variables for signal
        self.signal_var = Float32() 
        self.signal_params_var = SignalInfo()
        self.time_var = 0

        #Message for succesful node creation
        self.get_logger().info('signal_generator_node Initialized')

    #Callbacks
    def signal_handler_callback(self):
        self.time_var += self.signal_handler_period
        #Saving params in variables
        type_of_signal_var = self.get_parameter('type_of_signal').get_parameter_value().integer_value
        amplitude_var = self.get_parameter('amplitude_param').get_parameter_value().double_value
        frequency_var = self.get_parameter('frequency_param').get_parameter_value().double_value
        offset_var = self.get_parameter('offset_param').get_parameter_value().double_value

        #Saving params in custom message 
        self.signal_params_var.type = type_of_signal_var
        self.signal_params_var.amplitude = amplitude_var
        self.signal_params_var.frequency = frequency_var
        self.signal_params_var.offset = offset_var
       
        #Creating function
        frequency2radiansPersecond = (self.time_var * frequency_var * np.pi * 2)
        if(type_of_signal_var == 1):
            self.signal_var.data = np.sin(frequency2radiansPersecond + offset_var) * amplitude_var        
        elif(type_of_signal_var == 2):
            t = [frequency2radiansPersecond + offset_var]
            self.signal_var.data = signal.square(t, duty=0.5)[0] * amplitude_var
        elif(type_of_signal_var == 3):
            t = [frequency2radiansPersecond + offset_var]
            self.signal_var.data = signal.sawtooth(t)[0] * amplitude_var
        elif(type_of_signal_var == 4):
            t = [frequency2radiansPersecond + offset_var]
            self.signal_var.data = (signal.square(t, duty=0.5)[0] * amplitude_var) + (signal.sawtooth(t)[0] * amplitude_var)
        elif(type_of_signal_var == 5):
            t = [frequency2radiansPersecond + offset_var]
            self.signal_var.data = (signal.square(t, duty=0.5)[0] * amplitude_var) - (signal.sawtooth(t)[0] * amplitude_var)

        
        #Publishing signal and params
        self.signal_pub.publish(self.signal_var)
        self.signal_params_pub.publish(self.signal_params_var)

def main(args=None):
    rclpy.init(args=args)
    sg = SignalGenerator()
    rclpy.spin(sg)
    sg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

