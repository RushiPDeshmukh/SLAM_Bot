import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np
from statistics import mean

class imu_node(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.raw_data_subscriber = self.create_subscription(Imu,'rp2040_imu_raw',self.raw_data_callback,10)

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
        # self.calibrated = self.gyro_calibrated
        if(not self.calibrated):
            self.get_logger().info("Collecting data for calibration...")

            if len(self.gyro_data)>=1000:
                self.gyro_calibration()
                self.acc_calibration()

        elif(self.calibrated):
            x,y,theta = self.getOrientation()
            #call pose publisher callback
            # output_msg = Pose()
            # output_msg.x = x
            # output_msg.y = y
            # output_msg.z = theta 
            # self.pose_publisher.publish(output_msg)
            # self.get_logger().info('Publishing pose...')
            self.destroy_node()

    def gyro_calibration(self):
        gyro_data_np = np.array(self.gyro_data).reshape(-1,3)
        self.gyro_offsets = gyro_data_np.mean(axis=0)
        self.gyro_min = gyro_data_np.min(axis=0)
        self.gyro_max = gyro_data_np.max(axis=0)
        self.get_logger().info(f'Gyro : zero offset error: {self.gyro_offsets}, min: {self.gyro_min}, max: {self.gyro_max}')
        self.gyro_calibrated = True
        return None
    
    def acc_calibration(self):
        acc_data_np = np.array(self.accelero_data).reshape(-1,3)
        self.acc_offsets = acc_data_np.mean(axis=0)
        self.acc_min = acc_data_np.min(axis=0)
        self.acc_max = acc_data_np.max(axis=0)
        self.get_logger().info(f'Accel : zero offset error: {self.acc_offsets}, min: {self.acc_min}, max: {self.acc_max}')
        self.acc_calibrated = True
        return None
    
    def getOrientation(self):
        
        return 0,0,0
    
def main():
    rclpy.init()
    imu_node_object = imu_node()
    rclpy.spin(imu_node_object)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()   