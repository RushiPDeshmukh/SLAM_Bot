import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np
from statistics import mean

class imu_test(Node):
    def __init__(self):
        super().__init__('imu_test')
        
        self.raw_data_subscriber = self.create_subscription(Imu,'rp2040_imu_info_topic',self.raw_data_callback,10)

        self.pose_publisher = self.create_publisher(Pose,"imu_sensor_pose",10)
        self.get_logger().info("Welcome to IMU Node:")
        self.accelero_data = []
        self.gyro_data = []
        self.calibrated = False
        self.acc_calibrated = False
        # Change offsets and calibrated to True
        self.gyro_offsets = []
        self.gyro_calibrated = False
        self.old_theta=0
        

    def raw_data_callback(self,msg):
        Ax=msg.linear_acceleration.x
        Ay=msg.linear_acceleration.y
        Az=msg.linear_acceleration.z
        Gx=msg.angular_velocity.x
        Gy=msg.angular_velocity.y
        Gz=msg.angular_velocity.z
        
        self.accelero_data.append([Ax,Ay,Az])
        self.gyro_data.append([Gx,Gy,Gz])
        #self.calibrated = self.acc_calibrated and self.gyro_calibrated
        self.calibrated = self.gyro_calibrated
        if(not self.calibrated):
            self.get_logger().info('Gyro not calibrated')

        elif(self.calibrated):
            x,y,theta = self.getOrientation()
            #call pose publisher callback
            output_msg = Pose()
            output_msg.x = x
            output_msg.y = y
            output_msg.z = theta 
            self.pose_publisher.publish(output_msg)
            self.get_logger().info('Publishing pose...')
            self.old_theta=theta

    def getOrientation(self):
        theta = self.old_theta
        return 0,0,0
    
def main():
    rclpy.init()
    test_node_object = imu_test()
    rclpy.spin(test_node_object)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()   