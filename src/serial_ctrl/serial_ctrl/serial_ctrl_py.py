import rclpy
from rclpy.node import Node
import array

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int8

import json
import serial

ser = serial.Serial("/dev/ttyUSB0",115200)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('serial_ctrl')
        
        self.position = []
        # TODO: Should be a parameter
        self.joint_vel = 1500
        self.gripper_vel = 4000 # Gripper should be faster due to a gear reduction
        self.gripper_acc = 200
        self.joint_acc = 100
        self.critical_vel = 800 # joint 3 has a critical vel due to gravity and inertia
        self.critical_acc = 30  # joint 3 has a critical acc due to gravity and inertia
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.speed_subscription = self.create_subscription(   
            Int8,
            'joint_speed',
            self.speed_listener_callback,
            10)


    def getJointAngle(self, angleInput, calibOffset, direction=1):
        angle = calibOffset + (angleInput *180/3.1415926 * direction)

        return angle

    def speed_listener_callback(self, msg):
        self.joint_vel = msg.data*20
        if self.joint_vel > 2000:
            self.joint_vel = 2000
        if self.joint_vel < 200:
            self.joint_vel = 200

        self.joint_acc = msg.data*2
        if self.joint_acc > 200:
            self.joint_acc = 200
        if self.joint_acc < 10:
            self.joint_acc = 10

    def listener_callback(self, msg):
        a = msg.position

        # TODO: Offsets has to come from calibration file
        joint1 = self.getJointAngle(a[0], 182.5488-4.35, -1)
        joint2 = self.getJointAngle(a[1], -4.21875-4, -1)
        joint3 = self.getJointAngle(a[2], -3.6914-9, -1)
        joint4 = self.getJointAngle(a[3], 1.66992-2, -1)
        joint5 = self.getJointAngle(a[4], 185.888, -1)

        if self.joint_vel > self.critical_vel:
            critical_joint_vel = self.critical_vel
        else:
            critical_joint_vel = self.joint_vel

        if self.joint_acc > self.critical_acc:
            critical_joint_acc = self.critical_acc
        else:
            critical_joint_acc = self.joint_acc

        data = json.dumps({'T':1,'P1':f"{joint1:.3f}",'P2':f"{joint2:.3f}",'P3':f"{joint3:.3f}",'P4':f"{joint4:.3f}",'P5':f"{joint5:.3f}",
                           'S1':self.joint_vel,'S2':self.joint_vel,'S3':critical_joint_vel,'S4':self.joint_vel,'S5':self.gripper_vel,
                           'A1':self.joint_acc,'A2':self.joint_acc,'A3':critical_joint_acc,'A4':self.joint_acc,'A5':self.gripper_acc})
        
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
