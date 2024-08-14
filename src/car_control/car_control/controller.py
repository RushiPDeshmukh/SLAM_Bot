import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
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
        self.__odom_tf_broadcaster = TransformBroadcaster(self)
        self.__joint_state_publisher = self.create_publisher(JointState,'joint_states',10)
        #Car Parameters
        self.L = 0.106 ##### Dist from robot body center to wheel center in x direction (along longer body side)
        self.W = 0.094 #####
        self.wheel_radius = 0.0485
        self.wheel_dir_alignment = np.array([1,-1, 1, -1]).reshape(4,1)
        self.kinematic_model = np.array([[1, -1, -(self.L + self.W)],[1, 1, (self.L + self.W)],[1,1,-(self.L+self.W)],[1,-1,(self.L+self.W)]])
        self.car_controller = M5Module4EncoderMotorController()
        self.car_controller.setMode(0,0x00) #Normal Mode
        self.car_controller.setMode(1,0x00)
        self.car_controller.setMode(2,0x00)
        self.car_controller.setMode(3,0x00)

        self.prev_odom = None
        self.prev_encoder_values = None

        self.motor_ppr = 2880

        self.car_controller.setEncoderValues([0,500000,500000,0]) # RL , RR , FR, FL

        self.joint_state = JointState()
        self.joint_state.header.stamp=self.get_clock().now().to_msg()
        self.joint_state.header.frame_id='base_link'
        self.joint_state.name.append('drivewhl_fl_joint')
        self.joint_state.name.append('drivewhl_fr_joint')
        self.joint_state.name.append('drivewhl_rl_joint')
        self.joint_state.name.append('drivewhl_rr_joint')
        self.joint_state.position.append(0.0)
        self.joint_state.position.append(0.0)
        self.joint_state.position.append(0.0)
        self.joint_state.position.append(0.0)

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
            print(wheel_angular_velocities)
            PWMs = 21.81500872600349*wheel_angular_velocities
            PWMs = PWMs.reshape(1,4)[0].astype(int).tolist()
            fl,fr,rl,rr = PWMs[0],PWMs[1],PWMs[2],PWMs[3]
            PWMs[0],PWMs[1],PWMs[2],PWMs[3] = rl,rr,fr,fl
            print(PWMs)
            return PWMs
    
    def odom_publisher_callback(self):
        odom_msg = Odometry()
        now_time_=self.get_clock().now()
        
        try:
            current_encoder_value_ = self.car_controller.getEncoderValues() # RL , RR , FR, FL
            current_encoder_value_[1] = 500000 - current_encoder_value_[1]
            current_encoder_value_[2] = 500000 - current_encoder_value_[2]

            # convert [ RL , RR , FR, FL ] to [ FL , FR , RL , RR ]
            current_encoder_value = [current_encoder_value_[3],current_encoder_value_[2],current_encoder_value_[0],current_encoder_value_[1]]

        except Exception as e:
            self.get_logger().info(f'Error in getting encoder value : {e}')

        if self.prev_odom is None:
            self.prev_encoder_values = [0,0,0,0]
            self.prev_odom = Odometry()
            self.prev_odom.header.frame_id='odom'
            self.prev_odom.header.stamp = self.get_clock().now().to_msg()
            self.prev_odom.pose.pose.position.x=0.
            self.prev_odom.pose.pose.position.y=0.
            self.prev_odom.pose.pose.orientation=self.get_quaternion_from_euler(0,0,0)
            self.prev_odom.twist.twist.linear.x=0. #v_x 
            self.prev_odom.twist.twist.linear.y=0. #v_y
            self.prev_odom.twist.twist.angular.z=0. #w_z

        (pos_x,pos_y,yaw,v_x,v_y,w_z,wheel_w,delta_t)=self.calculate_odom(now_time_.nanoseconds,current_encoder_value,self.prev_encoder_values,self.prev_odom)
        
        odom_msg = Odometry()
        odom_msg.header.frame_id='odom'
        odom_msg.header.stamp=now_time_.to_msg()
        odom_msg.pose.pose.position.x = pos_x 
        odom_msg.pose.pose.position.y = pos_y
        odom_msg.pose.pose.orientation = self.get_quaternion_from_euler(0,0,yaw)
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
        odom_msg.twist.twist.angular.z = w_z
        self.__odom_publisher.publish(odom_msg)
        
        # Publish joint states
        self.joint_state.position[0] = wheel_w[0]*delta_t # front left
        self.joint_state.position[1] = wheel_w[1]*delta_t # front right        
        self.joint_state.position[2] = wheel_w[2]*delta_t # rear left
        self.joint_state.position[3] = wheel_w[3]*delta_t # rear right       
        self.__joint_state_publisher.publish(self.joint_state)
        
        # Publish tf for odom to base link
        transform_ = TransformStamped()
        transform_.header.stamp=self.get_clock().now().to_msg()
        transform_.header.frame_id='odom'
        transform_._child_frame_id='base_link'

        transform_.transform.translation.x = pos_x
        transform_.transform.translation.y = pos_y
        transform_.transform.rotation=self.get_quaternion_from_euler(0,0,yaw)         
        
        self.__odom_tf_broadcaster.sendTransform(transform_)

        # Update previous odom message
        self.prev_odom = odom_msg
        self.prev_encoder_values = current_encoder_value
        if current_encoder_value[1] > 40000 or current_encoder_value[3] > 40000 :
            self.car_controller.setEncoderValues([0,500000,500000,0])
            self.prev_encoder_values = [0,0,0,0]
            print("Resetting encoder values!!!!")
        
    def calculate_odom(self,time_,this_enc_values,prev_enc_values,prev_odom):
        del_time = (time_-prev_odom.header.stamp.nanosec)*10e-9
        motor_angular_velocities = np.array([self.encoder_to_rad(this_enc_values[0]-prev_enc_values[0]),self.encoder_to_rad(this_enc_values[1]-prev_enc_values[1]),self.encoder_to_rad(this_enc_values[2]-prev_enc_values[2]),self.encoder_to_rad(this_enc_values[3]-prev_enc_values[3])])/del_time

        lin_x = (self.wheel_radius/4)*(motor_angular_velocities[0]+motor_angular_velocities[1]+motor_angular_velocities[2]+motor_angular_velocities[3])
        lin_y = (self.wheel_radius/4)*(-motor_angular_velocities[0]+motor_angular_velocities[1]+motor_angular_velocities[2]-motor_angular_velocities[3])
        ang_z = (self.wheel_radius/(4*(self.L+self.W)))*(-motor_angular_velocities[0]+motor_angular_velocities[1]-motor_angular_velocities[2]+motor_angular_velocities[3])

        prev_yaw = self.get_euler_from_quaternion(prev_odom.pose.pose.orientation)[2]
        pose_x = prev_odom.pose.pose.position.x + del_time*(lin_x*np.cos(prev_yaw)-lin_y*np.sin(prev_yaw))
        pose_y = prev_odom.pose.pose.position.y + del_time*(lin_x*np.sin(prev_yaw)+lin_y*np.cos(prev_yaw))
        new_yaw = prev_yaw + ang_z*del_time

        return pose_x,pose_y,new_yaw,lin_x,lin_y,ang_z,motor_angular_velocities,del_time

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
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw_z = np.arctan2(t3, t4)

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


if __name__ == "__main__":
    main()
