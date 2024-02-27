#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include <Kinematic.h>
#include <Motor.h>
#include <MPU9250.h>

#define DirectionPin 4 // Define pin for motor direction
#define BaudRate 115200 // Define baud rate for serial commkgmmmunication

rcl_publisher_t wheel_publisher;
rcl_publisher_t imu_publisher;

rcl_subscription_t cmd_vel_subscription;

std_msgs__msg__Float64MultiArray wheel_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

struct timespec ts;

TaskHandle_t task0;
TaskHandle_t task1;

SemaphoreHandle_t imu_sem;
SemaphoreHandle_t wheel_sem;
SemaphoreHandle_t cmd_sem;

DiffDriveKinematic kin;
MPU9250 mpu;

double wheel_speed[2] = {0.0};
double gyro[3] = {0.0};
double accel[3] = {0.0};
double quat[4] = {0.0};

extern int clock_gettime(clockid_t unused, struct timespec *tp);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    clock_gettime(CLOCK_REALTIME, &ts);
    imu_msg.header.stamp.sec = ts.tv_sec;
    imu_msg.header.stamp.nanosec = ts.tv_nsec;
    if (xSemaphoreTake(imu_sem, 0))
    {
      imu_msg.linear_acceleration.x = accel[0];
      imu_msg.linear_acceleration.y = accel[1];
      imu_msg.linear_acceleration.z = accel[2];

      imu_msg.angular_velocity.x = gyro[0];
      imu_msg.angular_velocity.y = gyro[1];
      imu_msg.angular_velocity.z = gyro[2];

      // imu_msg.orientation.x = quat[0];
      // imu_msg.orientation.y = quat[1];
      // imu_msg.orientation.z = quat[2];
      // imu_msg.orientation.w = quat[3];
    }

    if (xSemaphoreTake(wheel_sem, 0))
    {
      wheel_msg.data.data[0] = wheel_speed[0];
      wheel_msg.data.data[1] = wheel_speed[1];
    }

    RCSOFTCHECK(rcl_publish(&wheel_publisher, &wheel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist *cmd_vel_msg = (const geometry_msgs__msg__Twist *)msgin;

  DiffDrive_IK_Compute(&kin, cmd_vel_msg->linear.x, cmd_vel_msg->angular.z);

  xSemaphoreGive(cmd_sem);
}

void loop0(void* pvParameters)
{

  Motor.begin(BaudRate, DirectionPin, &Serial2);
  int dir[2] = {0};
  double speed[2] = {0.0};
  float gain = 60.0 / (2.0 * M_PI);

  double deg2rad = M_PI / 180.0;
  double g2m_s2 = -9.81;

  Wire.begin();
  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(1000);
      }
  }

  uint32_t timestamp = millis() + 10;

  while (1)
  {
    if (millis() >= timestamp)
    {
      timestamp += 20;

      if (xSemaphoreTake(cmd_sem, 0))
      {
        speed[0] = kin.left_wheel;
        speed[1] = kin.right_wheel;
      }

      if (speed[0] >= 0) dir[0] = 0;
      else dir[0] = 1;

      if (speed[1] >= 0) dir[1] = 1;
      else dir[1] = 0;

      Motor.turnWheel(1, LEFT, round((0.916 * dir[0] * 1024) + (gain * fabs(speed[0]))));
      Motor.turnWheel(2, LEFT, round((0.916 * dir[1] * 1024) + (gain * fabs(speed[1]))));

      int motor_speed;
      for (int i = 0; i < 10; i++) {
        motor_speed = Motor.readSpeed(1);
        if (motor_speed >= 0 && motor_speed <= 2047) break;
      }
      if (motor_speed >= 1024) wheel_speed[0] = 0.916 * (1024 - motor_speed) / gain;
      else if (motor_speed >= 0) wheel_speed[0] = 0.916 * motor_speed / gain;
      // if (motor_speed >= 1024) wheel_speed[0] = (0.6 * 0.916 * (1024 - motor_speed) / gain) + (0.4 * wheel_speed[0]);
      // else if (motor_speed >= 0) wheel_speed[0] = (0.6 * 0.916 * motor_speed / gain) + (0.4 * wheel_speed[0]);
      // Serial.print(motor_speed);

      for (int i = 0; i < 10; i++) {
        motor_speed = Motor.readSpeed(2);
        if (motor_speed >= 0 && motor_speed <= 2047) break;
      }
      if (motor_speed >= 1024) wheel_speed[1] = 0.916 * (motor_speed - 1024) / gain;
      else if (motor_speed >= 0) wheel_speed[1] = -0.916 * motor_speed / gain;
      // Serial.print(' ');
      // Serial.println(motor_speed);

      xSemaphoreGive(wheel_sem);
    }

    if (mpu.update()) 
    {
      accel[0] = mpu.getAccX() * g2m_s2;
      accel[1] = mpu.getAccY() * g2m_s2;
      accel[2] = mpu.getAccZ() * g2m_s2;

      gyro[0] = mpu.getGyroX() * deg2rad;
      gyro[1] = mpu.getGyroY() * deg2rad;
      gyro[2] = mpu.getGyroZ() * deg2rad;

      // quat[0] = mpu.getQuaternionX();
      // quat[1] = mpu.getQuaternionY();
      // quat[2] = mpu.getQuaternionZ();
      // quat[3] = mpu.getQuaternionW();

      xSemaphoreGive(imu_sem);
    }
  }
}

void loop1(void* pvParameters)
{

  ///////////////////
  IPAddress agent_ip(192,168,139,55);
  size_t agent_port = 8888;

  char ssid[] = "AndroidAP8A99";
  char psk[]= "omuo0856";
  Serial.println("Initial Robot");
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  // char ssid[] = "Galaxy Note20 Ultra 5Gf04d";
  // char psk[]= "0936742513";
  // Serial.println("Initial Robot");
  // set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  // delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &wheel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "wheel_vel"));

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_raw"));

  // create subscription
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));

  // create timer,
  const unsigned int timer_timeout = 33;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &cmd_vel_subscription, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));

  // create variable
  wheel_msg.data.capacity = 10;
  wheel_msg.data.data = (double_t*) malloc(wheel_msg.data.capacity * sizeof(double_t));
  wheel_msg.data.size = 2;

  while (1)
  {
    delay(1);
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(1)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1)));
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);

  DiffDrive_IK_Init(&kin, 67.5 / (2.0 * 1000.0), 162.5 / (1000.0));

  imu_sem = xSemaphoreCreateBinary();
  wheel_sem = xSemaphoreCreateBinary();
  cmd_sem = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(loop0, "Task0", 10000, NULL, tskIDLE_PRIORITY, &task0, 1);
  delay(500);
  xTaskCreatePinnedToCore(loop1, "Task1", 10000, NULL, tskIDLE_PRIORITY, &task1, 0);
  delay(500);

}

void loop() {
  delay(100);
}
