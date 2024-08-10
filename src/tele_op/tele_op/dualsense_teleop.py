import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



class keyboard_teleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.twist_publisher = self.create_publisher(Twist,'cmd_vel',1)
        self.sub = self.create_subscription(Joy,"/joy",self.tele_op_map,1)
        self.z = 0.0
        self.reset = False
        self.update = True

    def tele_op_map(self,msg):
        joystick_values = msg.axes
        x = 0.0
        y = 0.0
        rot_z = 0.0
        
        if joystick_values[1] == 1: #front
            y = 0.0667 #m/s     
            self.update = True
        if joystick_values[1] == -1: #back
            y = -0.0667 #m/s 
            self.update = True  

        if joystick_values[0] == 1: #left
            x = -0.0667 #m/s  
            self.update = True   
        if joystick_values[0] == -1: #right
            x = 0.0667 #m/s   
            self.update = True
        if joystick_values[3] == 1: #turn left
            rot_z = -0.0667 #m/s
            self.update = True     
        if joystick_values[3] == -1: #turn right
            rot_z = 0.0667 #m/s
            self.update = True   

        if msg.buttons[4]:
            self.reset = True
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            self.twist_publisher.publish(cmd_vel)

        if msg.buttons[5]:
            self.reset = False
            

        if(not self.reset and self.update):
            cmd_vel = Twist()
            cmd_vel.linear.x = x
            cmd_vel.linear.y = y
            cmd_vel.angular.z = rot_z
            self.twist_publisher.publish(cmd_vel)
            self.update = True
            

def main():
    rclpy.init()
    node = keyboard_teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
