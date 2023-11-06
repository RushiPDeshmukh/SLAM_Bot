#include "Odom_control_node.h"
#include <Arduino_LSM6DSOX.h>
#include <micro_ros_arduino.h>

OdomControlNode::OdomControlNode() {
    // Constructor (if needed)
  this->allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(this->&support, 0, NULL, this->&allocator));
}

void OdomControlNode::setupMicroROS() {
    set_micro_ros_transports();
}

void OdomControlNode::setupIMU() {
    // Initialize IMU sensor
    if (!IMU.begin()) {
        Serial.println("Failed to initialize RP2040 IMU.");
        while (1) {
            delay(10);
        }
    }
    Serial.println("IMU Initialized !");
}

void OdomControlNode::setupNodeAndPublisher() {
    // Your code to set up the node and publisher
    RCCHECK(rclc_node_init_default(this->&node, "rp2040_imu_publisher_node", "", this->&support));
    RCCHECK(rclc_publisher_init_default(this->&publisher, this->&node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "rp2040_imu_info_topic"));
    // Define and initialize the subscriber for the "/command" topic.
    RCCHECK(rclc_subscription_init_default(
        this->&subscriber,
        this->&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel",
        this->command_callback
    ));
  
}

void OdomControlNode::setupTimerAndExecutor() {
    // Your code to set up the timer and executor
    RCCHECK(rclc_timer_init_default(this->&timer, this->&support, RCL_MS_TO_NS(this->timer_timeout), this->timer_callback));
    RCCHECK(rclc_executor_init(this->&executor, this->&support.context, 2, this->&allocator));  // Increased the number of subscriptions to 2.
    RCCHECK(rclc_executor_add_timer(this->&executor, this->&timer));
    RCCHECK(rclc_executor_add_subscription(this->&executor, this->&subscriber);  // Add the subscriber to the executor.
}

void OdomControlNode::command_callback(const void *msgin){
    const sensor_msgs__msg__Imu *command_msg = (const geometry_msgs__msg_Twist *)msgin;

  // Process the received message.
  // You can access data from the received message using command_msg->field_name.
}

void OdomControlNode::timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // Read accelerometer and gyroscope data from the IMU sensor.
    IMU.readAcceleration(this->Ax, this->Ay, this->Az);
    IMU.readGyroscope(this->Gx, this->Gy, this->Gz);
    
    // Update the linear acceleration and angular velocity fields in the IMU message.
    this->imu_msg.linear_acceleration.x = this->Ax;
    this->imu_msg.linear_acceleration.y = this->Ay;
    this->imu_msg.linear_acceleration.z = this->Az;
    this->imu_msg.angular_velocity.x = this->Gx;
    this->imu_msg.angular_velocity.y = this->Gy;
    this->imu_msg.angular_velocity.z = this->Gz;

    // Publish the updated IMU message.
    RCSOFTCHECK(rcl_publish(this->&publisher, this->&imu_msg, NULL));
  }
}
void OdomControlNode::spin(){
    RCSOFTCHECK(rclc_executor_spin_some(this->&executor, RCL_MS_TO_NS(100)));
}