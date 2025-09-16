import rclpy
from rclpy.node import Node
import array

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import json
import serial

ser = serial.Serial("/dev/ttyUSB0",115200)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('serial_ctrl')
        
        self.position = []
        # TODO: Should be a parameter
        self.joint_vel = 1000
        self.joint_acc = 30
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def getJointAngle(self, angleInput, calibOffset, direction=1):
        angle = calibOffset + (angleInput *180/3.1415926 * direction)

        return angle


    def listener_callback(self, msg):
        a = msg.position

        # TODO: Offsets has to come from calibration file
        joint1 = self.getJointAngle(a[0], 182.5488, -1)
        joint2 = self.getJointAngle(a[1], -4.21875, -1)
        joint3 = self.getJointAngle(a[2], -3.6914, -1)
        joint4 = self.getJointAngle(a[3], 1.66992, -1)
        joint5 = self.getJointAngle(a[4], 185.888, -1)

        data = json.dumps({'T':1,'P1':f"{joint1:.3f}",'P2':f"{joint2:.3f}",'P3':f"{joint3:.3f}",'P4':f"{joint4:.3f}",'P5':f"{joint5:.3f}",
                           'S1':self.joint_vel,'S2':self.joint_vel,'S3':self.joint_vel,'S4':self.joint_vel,'S5':self.joint_vel,
                           'A1':self.joint_acc,'A2':self.joint_acc,'A3':self.joint_acc,'A4':self.joint_acc,'A5':self.joint_acc})
        
        ser.write(data.encode())
        
        print(data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
