#ifndef ODOM_CONTROL_NODE_H
#define ODOM_CONTROL_NODE_H

#include <micro_ros_arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/twist.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

class OdomControlNode{
    private:
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
        const unsigned int timer_timeout = 1000;
    public:
        OdomControlNode();
        void setupMicroROS();
        void setupIMU();
        void setupNodeAndPublisher();
        void setupTimerAndExecutor();
        void command_callback(const void *msgin);
        void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
        void spin()

};

#endif
