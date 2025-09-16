import rclpy
from rclpy.node import Node
import array

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import json
import serial

ser = serial.Serial("/dev/ttyUSB0",115200)


class AngleMonitor(Node):

    def __init__(self):
        super().__init__('angle_monitor')
        
        self.timer = self.create_timer(0.5, self.timer_callback)     # Timer callback, period in seconds, not frequency!

        self.get_logger().info("Angle monitor has been started, disabling servos...")

        data = json.dumps({'T':8,'P1':0})
        ser.write(data.encode())

        out = ser.readline().decode('utf-8')

        self.get_logger().info(out)

        

    def timer_callback(self):                                        # Timer callback function implementation
        data = json.dumps({'T':5})
        
        ser.write(data.encode())

        self.get_logger().debug(data)

        out = ser.readline().decode('utf-8')

        self.get_logger().debug(out)

        out = ser.readline().decode('utf-8')

        self.get_logger().info(out)
    
    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        else:
            getPos = int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 0.5)
            return getPos


def main(args=None):
    rclpy.init(args=args)

    angle_monitor = AngleMonitor()

    rclpy.spin(angle_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    angle_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
