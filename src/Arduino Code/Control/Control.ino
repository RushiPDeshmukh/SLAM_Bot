#include <micro_ros_arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include "driver.h"

sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_subscription_t subscriber;

float Ax, Ay, Az, Gx, Gy, Gz;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void command_callback(const void *msgin){
  const sensor_msgs__msg__Imu *command_msg = (const geometry_msgs__msg_Twist *)msgin;

  // Use received velocity
  // v_x , v_y , w_z --> lin.x , lin.y , ang.z
  car.runModel(float(command_msg.linear.x),float(command_msg.linear.y),float(command_msg.angular.z))

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // Read accelerometer and gyroscope data from the IMU sensor.
    IMU.readAcceleration(Ax, Ay, Az);
    IMU.readGyroscope(Gx, Gy, Gz);
    
    // Update the linear acceleration and angular velocity fields in the IMU message.
    imu_msg.linear_acceleration.x = Ax;
    imu_msg.linear_acceleration.y = Ay;
    imu_msg.linear_acceleration.z = Az;
    imu_msg.angular_velocity.x = Gx;
    imu_msg.angular_velocity.y = Gy;
    imu_msg.angular_velocity.z = Gz;

    // Publish the updated IMU message.
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
  }
}


void setup() {
  set_microros_transports();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize RP2040 IMU.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("IMU Initialized !");

                       //-------  | DIR | EN(~)
  Motor motor_rl(3,5); // Motor 1 | 3   | 5  ~  Rear Left 
  Motor motor_rr(4,6); // Motor 2 | 4   | 6  ~  Rear Right
  Motor motor_fl(7,9); // Motor 3 | 7   | 9  ~  Front Left
  Motor motor_fr(8,10);// Motor 4 | 8   | 10 ~  Front Right
  Robot car(15,15,motor_rl,motor_rr,motor_fl,motor_fr);
  Serial.println("Car with motors Initialized !");

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rp2040_imu_publisher_node", "", &support));
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "rp2040_imu_info_topic"));
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  // Increased the number of subscriptions to 2.
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Define and initialize the subscriber for the "/command" topic.
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel",
    command_callback
  ));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber);  // Add the subscriber to the executor.
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
