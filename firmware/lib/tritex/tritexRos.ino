//ros2 launch micro_ros_agent micro_ros_agent_launch.py serial --dev /dev/ttyUSB0

#include "sensorTritex.h"

#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/vector3.h>

#include <sensor_msgs/msg/range.h>  // libreria range

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>



#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); } \
  }

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)


rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rcl_node_t node;

///////subscribers
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor_sub;


/////publishers
rcl_publisher_t publisher, pubTritex;

sensor_msgs__msg__Range *range_msg;

rclc_executor_t executor_pub;

unsigned long long time_offset = 0;

bool micro_ros_init_successful;

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;



bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "microRos_Node", "", &support));


  //tritex
  RCCHECK(rclc_publisher_init_default(
    &pubTritex,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "range/raw"));

  // twist subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  // executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  //create executor publisher
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));


  range_msg = sensor_msgs__msg__Range__create();

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}



void setup() {

  setupSensorTritex();

  set_microros_transports();

  state = WAITING_AGENT;
}

void loop() {


  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {


        lectura();



        struct timespec time_stamp = getTime();

        range_msg->header.stamp.nanosec = time_stamp.tv_nsec;
        range_msg->header.stamp.sec = time_stamp.tv_sec;
        range_msg->header.frame_id.data = "frame_range";

        range_msg->radiation_type = 0;
        range_msg->field_of_view = 1;

        range_msg->max_range = 1;
        range_msg->min_range = 0.001;

        range_msg->range = fsub;

        struct timespec tv = { 0 };
        clock_gettime(0, &tv);



        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));

        rcl_publish(&pubTritex, range_msg, NULL);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime() {
  struct timespec tp = { 0 };
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

void rclErrorLoop() {
  while (true) {
    //
  }
}