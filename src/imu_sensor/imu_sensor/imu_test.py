import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np
from statistics import mean

class imu_test(Node):
    def __init__(self):
        super().__init__('imu_test')
        
        self.raw_data_subscriber = self.create_subscription(Imu,'rp2040_imu_raw',self.raw_data_callback,10)

        self.pose_publisher = self.create_publisher(Pose,"imu_sensor_pose",10)
        self.imu_publisher = self.create_publisher(Imu,'imu',10)
        self.get_logger().info("IMU Calibration Test")
        self.accelero_data = []
        self.gyro_data = []
        self.calibrated = False
        self.acc_calibrated = False
        self.old_theta=0

        # Change offsets and calibrated to True
        self.gyro_calibrated = True
        
        #Calibration Parameters
        """ Gyro : zero offset error: [-0.2109375  -0.12591553 -0.22644043], min: [-0.36621094 -0.30517578 -1.83105469], max: [-0.12207031  0.          0.79345703]
            Accel : zero offset error: [-0.01117114  0.01199207  1.00737036], min: [-0.03344727  0.00366211  1.00488281], max: [-0.00585938  0.02893066  1.01000977]"""
        self.Ax_offset = -0.01117114
        self.Ay_offset = 0.01199207
        self.Az_offset = 0.00737036 # offset - 1 
        self.Gx_offset = -0.2109375
        self.Gy_offset = -0.12591553
        self.Gz_offset = -0.22644043
        self.Ax_scaling_factor = 1.0
        self.Ay_scaling_factor = 1.0
        self.Az_scaling_factor = 1.0
        self.Gx_scaling_factor = 1.0
        self.Gy_scaling_factor = 1.0
        self.Gz_scaling_factor = 1.0
        
        init_t = self.get_clock().now().to_msg()
        self.prev_time = init_t.sec + init_t.nanosec*1e-9     
        self.imu_data = Imu()  

    def raw_data_callback(self,msg):
        Ax,Ay,Az,Gx,Gy,Gz = self.apply_calibration(msg)
        
        t = self.get_clock().now().to_msg()
        self.now_time = t.sec + t.nanosec*1e-9
        self.dt = self.now_time-self.prev_time
        self.imu_data.header.stamp.sec= t.sec
        self.imu_data.header.stamp.sec= t.nanosec
        self.imu_data.angular_velocity.x=Gx
        self.imu_data.angular_velocity.y=Gy
        self.imu_data.angular_velocity.z=Gz
        self.imu_data.linear_acceleration.x=Ax
        self.imu_data.linear_acceleration.y=Ay
        self.imu_data.linear_acceleration.z=Az
        self.imu_publisher.publish(self.imu_data)
        #self.calibrated = self.acc_calibrated and self.gyro_calibrated
        self.calibrated = self.gyro_calibrated
        if(not self.calibrated):
            self.get_logger().info('Gyro not calibrated')

        elif(self.calibrated):
            x,y,theta = self.getOrientation()
            #call pose publisher callback
            # output_msg = Pose()
            # output_msg.x = x
            # output_msg.y = y
            # output_msg.z = theta 
            # self.pose_publisher.publish(output_msg)
            # self.get_logger().info('Publishing pose...')
            self.old_theta=theta

    def getOrientation(self):
        theta = self.old_theta + self.dt*self.imu_data.angular_velocity.z
        self.get_logger().info(f'Theta = {theta} -- Gyro {self.imu_data.angular_velocity.z}')
        return 0,0,theta
    
    def apply_calibration(self,msg):
        Ax= (msg.linear_acceleration.x - self.Ax_offset) * self.Ax_scaling_factor
        Ay= (msg.linear_acceleration.y - self.Ay_offset) * self.Ay_scaling_factor
        Az= (msg.linear_acceleration.z - self.Az_offset) * self.Az_scaling_factor
        Gx= (msg.angular_velocity.x - self.Gx_offset) * self.Gx_scaling_factor
        Gy= (msg.angular_velocity.y - self.Gy_offset) * self.Gy_scaling_factor
        Gz= (msg.angular_velocity.z - self.Gz_offset) * self.Gz_scaling_factor

        return Ax,Ay,Az,Gx,Gy,Gz


    
def main():
    rclpy.init()
    test_node_object = imu_test()
    rclpy.spin(test_node_object)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()   