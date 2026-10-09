import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

#skeleton node
class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')
        #declaring paramters   
        self.declare_parameter('joint_name', 'joint')
        self.get_parameter('joint_name').value
        self.declare_parameter('spi', 0)
        self.get_parameter('spi').value
        self.declare_parameter('spi_speed', 0)
        self.get_parameter('spi_speed').value
        self.declare_parameter('publish_rate_hz', 0)
        self.get_parameter('publish_rate_hz').value
        self.declare_parameter('counts_per_rev', 0)
        self.get_parameter('counts_per_rev').value
        self.declare_parameter('gear_ratio', 0)
        self.get_parameter('gear_ratio').value
        self.declare_parameter('direction_sign', 1)
        self.get_parameter('direction_sign').value
        self.declare_parameter('zero_offset', 0)
        self.get_parameter('zero_offset').value
        self.declare_parameter('velocity_filter_alpha', 0)
        self.get_parameter('velocity_filter_alpha').value

        #set up internal state
        self.last_raw = 0
        self.turn_count = 0
        self.last_time  = 0
        self.last_position = 0
        self.filtered_velocity = 0
        #create publisher
        self.publisher = self.create_publisher(JointState,drivetrain/joint_state ,10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.update)
   
    def read_angle_raw(self):
        #reads the angle from the SPI value
        #returns the raw angle and if its valid or not
        count = 0
        return count + 1, True

    #creates node and includes timer
    def update(self):
        a, v = self.read_angle_raw()
        if(v == True):
            if(last_time == 0):
                last_time = currentTime
            #unwrap
            #compute position
            #compute velocity
            Join
        else:
            return
def main(args=None):
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':    
    main()


#add the unwrap + positon & velocity math
# add sero_offset callibration
# prove its reusable          