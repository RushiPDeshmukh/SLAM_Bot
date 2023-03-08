import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np


class imu_node(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.raw_data_subscriber = self.create_subscription(Imu,'rp2040_imu_info_topic',self.raw_data_callback,10)

        #TO_DO publish orienation - x,y,theta
        self.pose_publisher = self.create_publisher(Pose,"imu_sensor_pose",10)
        self.get_logger().info("Welcome to IMU Node:")
        self.accelero_data = []
        self.gyro_data = []
        self.calibrated = False
        self.acc_calibrated = False
        self.gyro_calibrated = False
        self.gyro_offsets = []

    def raw_data_callback(self,msg):
        Ax=msg.linear_acceleration.x
        Ay=msg.linear_acceleration.y
        Az=msg.linear_acceleration.z
        Gx=msg.angular_velocity.x
        Gy=msg.angular_velocity.y
        Gz=msg.angular_velocity.z

        self.accelero_data.append([Ax,Ay,Az])
        self.gyro_data.append([Gx,Gy,Gz])
        self.calibrated = self.acc_calibrated and self.gyro_calibrated
        if(not self.calibrated):
            
            if len(self.gyro_data)>=100:
                self.gyro_calibration()

        elif(self.calibrated):
            x,y,theta = self.getOrientation()
            #call pose publisher callback
            output_msg = Pose()
            output_msg.x = x
            output_msg.y = y
            output_msg.z = theta 
            self.pose_publisher.publish(output_msg)
            self.get_logger().info('Publishing pose...')
        
        else:
            self.get_logger().info("Collecting data for calibration...")


    def gyro_calibration(self):
        self.gyro_offsets[0]=sum(row[0] for row in self.gyro_data)/len(self.gyro_data)
        self.gyro_offsets[1]=sum(row[1] for row in self.gyro_data)/len(self.gyro_data)
        self.gyro_offsets[2]=sum(row[2] for row in self.gyro_data)/len(self.gyro_data)
        self.gyro_calibrated = True
        return None
    
    def getOrientation(self):

        return None