import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion
from nav_msgs.msg import Odometry

import numpy as np
import sys
import os

# Get the directory of the current script
current_directory = os.path.dirname(os.path.abspath(__file__))

# Add the directory to sys.path
sys.path.append(current_directory)

from M5MotorController import M5Module4EncoderMotorController

class controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.__subcriber = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,1)
        self.__odom_publisher = self.create_publisher(Odometry,'/wheel_odom',10)
        self.__odom_timer = self.create_timer(0.02,self.odom_publisher_callback)

        #Car Parameters
        self.L = 0.106 ##### Dist from robot body center to wheel center in x direction (along longer body side)
        self.W = 0.094 #####
        self.wheel_radius = 0.0485
        self.wheel_dir_alignment = np.array([1,-1,-1,1]).reshape(4,1)
        self.kinematic_model = np.array([[1, -1, -(self.L + self.W)],[1, 1, (self.L + self.W)],[1,1,-(self.L+self.W)],[1,-1,(self.L+self.W)]])
        self.car_controller = M5Module4EncoderMotorController()
        self.car_controller.setMode(0,0x00) #Normal Mode
        self.car_controller.setMode(1,0x00)
        self.car_controller.setMode(2,0x00)
        self.car_controller.setMode(3,0x00)

        self.prev_odom = None
        self.prev_encoder_values = None

        self.motor_ppr = 2880

        self.car_controller.setEncoderValues([0,500000,500000,0])



    def cmd_vel_callback(self,msg):
        try:
            x = msg.linear.x
            y = msg.linear.y
            w = msg.angular.z
            cmd_vel = np.array([x,y,w]).reshape(3,1)
            self.get_logger().info("Received Cmd_vel")
            wheel_angular_velocities = (self.wheel_radius**-1*self.wheel_dir_alignment*self.kinematic_model@cmd_vel)
            
            PWMs = self.angularVelocities_to_PWM_convertor(wheel_angular_velocities)
            
            self.car_controller.setMotorSpeeds(PWMs)

        except KeyboardInterrupt:
            print("Ctrl+C pressed. Stopping the car and exiting gracefully.")

        except Exception as err:
            print(f"Exception: {err}. Stopping the car.")
            self.car_controller.setMotorSpeeds([0,0,0,0])

    def angularVelocities_to_PWM_convertor(self,wheel_angular_velocities):
            PWMs = 21.81500872600349*wheel_angular_velocities
            print(PWMs)
            PWMs = PWMs.reshape(1,4)[0].astype(int).tolist()
            print(PWMs)
            return PWMs
    
    def odom_publisher_callback(self):
        odom_msg = Odometry()
        now_time_=self.get_clock().now()
        self.get_logger().info(f'Clock : {now_time_} == {type(now_time_.nanoseconds)} == {type(now_time_.seconds_nanoseconds())}')
        try:
            current_encoder_value = self.car_controller.getEncoderValues()
            current_encoder_value[1] = 500000 - current_encoder_value[1]
            current_encoder_value[2] = 500000 - current_encoder_value[2]
        except Exception as e:
            self.get_logger().info(f'Error in getting encoder value : {e}')

        if self.prev_odom is None:
            self.prev_encoder_values = [0,0,0,0]
            self.prev_odom = Odometry()
            self.prev_odom.header.frame_id='base_footprint'
            self.prev_odom.header.stamp = self.get_clock().now().to_msg()
            self.prev_odom.pose.pose.position.x=0.
            self.prev_odom.pose.pose.position.y=0.
            self.prev_odom.pose.pose.orientation=self.get_quaternion_from_euler(0,0,0)
            self.prev_odom.twist.twist.linear.x=0. #v_x 
            self.prev_odom.twist.twist.linear.y=0. #v_y
            self.prev_odom.twist.twist.angular.z=0. #w_z

        (pos_x,pos_y,yaw,v_x,v_y,w_z)=self.calculate_odom(now_time_.nanoseconds,current_encoder_value,self.prev_encoder_values,self.prev_odom)
        
        odom_msg = Odometry()
        odom_msg.header.frame_id='base_footprint'
        odom_msg.header.stamp=now_time_.to_msg()
        odom_msg.pose.pose.position.x = pos_x
        odom_msg.pose.pose.position.y = pos_y
        odom_msg.pose.pose.orientation = self.get_quaternion_from_euler(0,0,yaw)
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
        odom_msg.twist.twist.angular.z = w_z
        self.__odom_publisher.publish(odom_msg)
        self.prev_odom = odom_msg
        self.prev_encoder_values = current_encoder_value
        
    def calculate_odom(self,time_,this_enc_values,prev_enc_values,prev_odom):
        # self.get_logger().info(f'Timestamp {prev_odom.header.stamp} , sec {prev_odom.header.stamp.sec} == {type(prev_odom.header.stamp.sec)}, ns {prev_odom.header.stamp.nanosec}=={type(prev_odom.header.stamp.nanosec)}')
        del_time = (time_-prev_odom.header.stamp.nanosec)*10e-9
        motor_angular_velocities = np.array([self.encoder_to_rad(this_enc_values[0]-prev_enc_values[0]),self.encoder_to_rad(this_enc_values[1]-prev_enc_values[1]),self.encoder_to_rad(this_enc_values[2]-prev_enc_values[2]),self.encoder_to_rad(this_enc_values[3]-prev_enc_values[3])])/del_time

        lin_x = (self.wheel_radius/4)*(motor_angular_velocities[0]+motor_angular_velocities[1]+motor_angular_velocities[2]+motor_angular_velocities[3])
        lin_y = (self.wheel_radius/4)*(-motor_angular_velocities[0]+motor_angular_velocities[1]+motor_angular_velocities[2]-motor_angular_velocities[3])
        ang_z = (self.wheel_radius/(4*(self.L+self.W)))*(-motor_angular_velocities[0]+motor_angular_velocities[1]-motor_angular_velocities[2]+motor_angular_velocities[3])

        pose_x = prev_odom.pose.pose.position.x + lin_x*del_time
        pose_y = prev_odom.pose.pose.position.y + lin_y*del_time
        new_yaw = self.get_euler_from_quaternion(prev_odom.pose.pose.orientation)[2] + ang_z*del_time

        return pose_x,pose_y,new_yaw,lin_x,lin_y,ang_z

    def get_quaternion_from_euler(self,roll,pitch,yaw):
        quat = Quaternion()
        quat.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        quat.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        quat.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        quat.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)
        return quat
    
    def get_euler_from_quaternion(self,quat):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        roll is rotation around the x-axis in radians (counterclockwise)
        pitch is rotation around the y-axis in radians (counterclockwise)
        yaw is rotation around the z-axis in radians (counterclockwise)
        """
        t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        roll_x = np.atan2(t0, t1)

        t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.asin(t2)

        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw_z = np.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def encoder_to_rad(self,encoder_):
        return (encoder_/self.motor_ppr)*2*np.pi

def main():
    rclpy.init()
    node = controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    except Exception as e:
        node.get_logger().error('An error occurred: %s' % str(e))
    finally:
        node.car_controller.setMotorSpeeds([0,0,0,0])
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
