#include <micro_ros_arduino.h>

#include <Arduino_LSM6DSOX.h>
#include <Wire.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float Ax,Ay,Az,Gx,Gy,Gz;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    //sensors_event_t a, g, temp;
    //mpu.getEvent(&a, &g, &temp);
    IMU.readAcceleration(Ax,Ay,Az);
    IMU.readGyroscope(Gx,Gy,Gz);
    imu_msg.linear_acceleration.x = Ax;
    imu_msg.linear_acceleration.y = Ay;
    imu_msg.linear_acceleration.z = Az;

    imu_msg.angular_velocity.x = Gx;
    imu_msg.angular_velocity.y = Gy;
    imu_msg.angular_velocity.z = Gz;

  }
}

void setup() {
  set_microros_transports();

  // Try to initialize!
  if (!IMU.begin()) {
    Serial.println("Failed to initialize RP2040 IMU.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("IMU Initialized !");
  // Serial.print("Accelerometer sample rate = ");
  // Serial.print(IMU.accelerationSampleRate());
  // Serial.println("Hz");
  // Serial.println();

  // Serial.print("Gyroscope sample rate = ");  
  // Serial.print(IMU.gyroscopeSampleRate());
  // Serial.println("Hz");
  // Serial.println();

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rp2040_imu_publisher_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "rp2040_imu_info_topic"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // imu_msg.data = "";
}

void loop() {
  delay(100);
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
