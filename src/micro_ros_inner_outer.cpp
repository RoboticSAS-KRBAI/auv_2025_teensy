// #include <Arduino.h>
// #include <Servo.h>
// #include <REG.h>
// #include <wit_c_sdk.h>
// #include <stdint.h>
// #include <math.h>
// #undef TEMP
// #include <Wire.h>
// #include "MS5837.h"
// #define ACC_UPDATE 0x01

// /////////////////////
// /// For Micro ROS ///
// /////////////////////
// #include <micro_ros_platformio.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <rmw_microros/rmw_microros.h>
// #include <ModbusMaster.h>

// void run_control_loop(); 

// // Initialize ModbusMaster HWT3100
// #define DE_RE 20

// ModbusMaster nodemod;

// void preTransmission() {
//   digitalWrite(DE_RE, HIGH);
//   delayMicroseconds(10);     
// }

// void postTransmission() {
//   delayMicroseconds(10);
//   digitalWrite(DE_RE, LOW);
// }

// // Message headers
// #include <std_msgs/msg/string.h>
// #include <std_msgs/msg/float32.h>
// #include <rosidl_runtime_c/string_functions.h>
// #include <auv_interfaces/msg/multi_pid.h>
// #include <auv_interfaces/msg/set_point.h>
// #include <auv_interfaces/msg/actuator.h>
// #include <auv_interfaces/msg/error.h>
// #include <auv_interfaces/msg/sensor.h>
// #include <auv_interfaces/msg/velocity_sensor.h>
// #include <auv_interfaces/msg/object_difference.h>

// /*
//  * Helper functions to help reconnect
// */
// #define EXECUTE_EVERY_N_MS(MS, X)  do { \
//     static volatile int64_t init = -1; \
//     if (init == -1) { init = uxr_millis();} \
//     if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
//   } while (0)

// enum states {
//   WAITING_AGENT,
//   AGENT_AVAILABLE,
//   AGENT_CONNECTED,
//   AGENT_DISCONNECTED
// } state;

// // Declare rcl object
// rclc_support_t support;
// rcl_init_options_t init_options;
// rcl_node_t node;
// rcl_timer_t timer;
// rclc_executor_t executor;
// rcl_allocator_t allocator;

// // Declare Publishers
// rcl_publisher_t pub_pwm, pub_error, pub_sensor, pub_set_point, pub_pid, pub_status, pub_boost, pub_velocity_sensor;

// // Declare Subscribers
// rcl_subscription_t sub_status, sub_boost, sub_pid, sub_set_point, sub_object_difference;

// // Declare Messages
// auv_interfaces__msg__Actuator pwm_msg;
// auv_interfaces__msg__SetPoint set_point_msg;
// auv_interfaces__msg__MultiPID pid_msg;
// auv_interfaces__msg__Error error_msg;
// auv_interfaces__msg__VelocitySensor velocity_msg;
// auv_interfaces__msg__ObjectDifference object_difference_msg;
// auv_interfaces__msg__Sensor sensor_msg;
// std_msgs__msg__String status_msg;
// std_msgs__msg__Float32 boost_msg;


// // =====================================================
// // DEADBAND PER-THRUSTER (Hasil pengukuran lab)
// // =====================================================
// // Index:       0     1     2     3     4     5     6     7     8     9
// // Nama:       T1    T2    T3    T4    T5    T6    T7    T8    T9   T10
// // Fungsi:    SSY   SSY   SSY   SSY   DPR   DPR   DPR   DPR  Boost Boost
// //
// // Forward = PWM minimal agar thruster mulai berputar ke depan (CW)
// // Reverse = PWM maksimal agar thruster mulai berputar ke belakang (CCW)
// //
// // Thruster 1 (idx 0): belum diukur → pakai nilai aman (terlebar)
// // Thruster 5 (idx 4): diukur 1476 tapi tidak stabil, pakai 1467 agar nyala terus
// // Thruster 9-10 (idx 8-9): boost, belum diukur → pakai nilai aman
// // =====================================================

// const int THRUSTER_FORWARD_START[10] = {
//   1527,  // T1  (idx 0) — belum diukur, pakai nilai tertinggi (aman) //1521
//   1527,  // T2  (idx 1) — diukur                                     //1518
//   1528,  // T3  (idx 2) — diukur                                     //1518
//   1528,  // T4  (idx 3) — diukur                                     //1519
//   1528,  // T5  (idx 4) — diukur                                     //1519
//   1528,  // T6  (idx 5) — diukur                                     //1519
//   1528,  // T7  (idx 6) — diukur                                     //1521
//   1528,  // T8  (idx 7) — diukur                                     //1519
//   1521,  // T9  (idx 8) — belum diukur, pakai nilai tertinggi (aman)  //ga pake thruster boost
//   1521   // T10 (idx 9) — belum diukur, pakai nilai tertinggi (aman)  //ga pake thruster boost
// };

// const int THRUSTER_REVERSE_START[10] = {
//   1467,  // T1  (idx 0) — belum diukur, pakai nilai terendah (aman) //1467
//   1467,  // T2  (idx 1) — diukur                                    //1476
//   1468,  // T3  (idx 2) — diukur                                    //1477
//   1468,  // T4  (idx 3) — diukur                                    //1477
//   1467,  // T5  (idx 4) — diukur 1476, TAPI tidak stabil! Pakai 1467 agar nyala terus
//   1468,  // T6  (idx 5) — diukur                                    //1477
//   1470,  // T7  (idx 6) — diukur                                    //1479
//   1467,  // T8  (idx 7) — diukur                                    //1476
//   1467,  // T9  (idx 8) — belum diukur, pakai nilai terendah (aman)  //ga pake thruster boost
//   1467   // T10 (idx 9) — belum diukur, pakai nilai terendah (aman)  //ga pake thruster boost
// };

// const int PWM_NEUTRAL = 1500;
// const int PWM_MIN = 1250;
// const int PWM_MAX = 1750;
// const float LOOP_DT = 0.01; // 100 Hz

// // =====================================================
// // MOVING AVERAGE CLASS (Generic, reusable)
// // =====================================================
// // Menggunakan incremental sum untuk efisiensi di embedded system.
// // Tidak perlu loop setiap kali update — O(1) per sample.
// // =====================================================
// template<int WINDOW_SIZE>
// class MovingAverage {
// private:
//   float buffer[WINDOW_SIZE];
//   int idx;
//   bool full;
//   float sum;

// public:
//   MovingAverage() : idx(0), full(false), sum(0) {
//     for (int i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0;
//   }

//   float update(float new_value) {
//     sum -= buffer[idx];       // kurangi nilai lama
//     buffer[idx] = new_value;  // masukkan nilai baru
//     sum += new_value;         // tambah nilai baru
//     idx = (idx + 1) % WINDOW_SIZE;

//     if (!full && idx == 0) full = true;

//     int count = full ? WINDOW_SIZE : idx;
//     if (count == 0) return new_value;
//     return sum / count;
//   }

//   void reset() {
//     for (int i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0;
//     idx = 0;
//     full = false;
//     sum = 0;
//   }
// };

// // =====================================================
// // MOVING AVERAGE INSTANCES
// // =====================================================
// // SENSOR_MA_WINDOW = 10 → 100ms averaging @ 100Hz
// // YAW_RATE_MA_WINDOW = 20 → 200ms averaging untuk inner loop
// // Bisa di-tune: kecilkan jika terlalu lambat, besarkan jika masih noisy
// // =====================================================
// #define SENSOR_MA_WINDOW 10
// #define YAW_RATE_MA_WINDOW 25

// MovingAverage<SENSOR_MA_WINDOW> ma_yaw;
// MovingAverage<SENSOR_MA_WINDOW> ma_pitch;
// MovingAverage<SENSOR_MA_WINDOW> ma_roll;
// MovingAverage<SENSOR_MA_WINDOW> ma_depth;
// MovingAverage<YAW_RATE_MA_WINDOW> ma_yaw_rate;

// // Sensor and control variables
// float yaw = 0.0, last_yaw = 0.0, delta_yaw = 0.0;
// float pitch = 0.0, last_pitch = 0.0, delta_pitch = 0.0;
// float roll = 0.0, last_roll = 0.0, delta_roll = 0.0;
// float depth = 0.0, last_depth = 0.0, delta_depth = 0.0;

// float set_point_yaw = 0, set_point_pitch = 0, set_point_roll = 0, set_point_depth = 0;
// float error_yaw = 0, error_pitch = 0, error_roll = 0, error_depth = 0;
// bool is_stable_roll = true, is_stable_pitch = true, is_stable_yaw = true, is_stable_depth = true;
// float thrust_dpr[4], thrust_ssy[4];
// float t_yaw = 0, camera_yaw = 0;
// int yawIndex = 0;
// float constrain_boost = 150.0;
// float pwm_thruster[10] = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0};

// // Yaw rate (untuk inner loop cascaded PID)
// float yaw_rate_filtered = 0.0;
// unsigned long last_compass_time = 0;

// // PID coefficients (dari ROS subscriber)
// float kp_yaw = 0, ki_yaw = 0, kd_yaw = 0;
// float kp_pitch = 1000, ki_pitch = 0, kd_pitch = 0;
// float kp_roll = 0, ki_roll = 0, kd_roll = 0;
// float kp_depth = 0, ki_depth = 0, kd_depth = 0;
// float kp_camera = 0, ki_camera = 0, kd_camera = 0;

// // =====================================================
// // CASCADED PID: Inner loop gains (hardcoded awal, tune manual)
// // =====================================================
// // Ini gains untuk inner loop (velocity PID).
// // Gains outer loop pakai kp_yaw/ki_yaw/kd_yaw dari ROS subscriber.
// // Tune inner loop ini setelah outer loop stabil.
// // =====================================================
// float kp_yaw_inner = 0.5, ki_yaw_inner = 0.1, kd_yaw_inner = 0.01;
// float MAX_YAW_RATE = 90.0; // Max desired yaw rate (deg/s) dari outer loop

// String class_name = "None";
// int camera_error = 0;
// String status = "stop";

// byte pin_thruster[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
// Servo thruster[10];
// Servo camera;
// MS5837 sensor;

// // Sensor communication variables
// static volatile char s_cDataUpdate = 0;
// const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
// float sqrt2 = sqrt(2);

// // =====================================================
// // PID Controller Class (untuk outer loop & DPR/pitch/roll/depth)
// // =====================================================
// class PID {
//   public:
//     float kp, ki, kd;
//     float integral;
//     float last_error;

//     PID(float kp, float ki, float kd)
//       : kp(kp), ki(ki), kd(kd), integral(0), last_error(0) {}

//     void updateGains(float new_kp, float new_ki, float new_kd) {
//       kp = new_kp;
//       ki = new_ki;
//       kd = new_kd;
//     }

//     float calculate(float error) {
//       // Integral
//       integral += error * LOOP_DT;

//       // Batasi integral (anti windup)
//       integral = constrain(integral, -10.0f, 10.0f);

//       float derivative = (error - last_error) / LOOP_DT;
//       last_error = error;

//       float p = kp * error;
//       float i = ki * integral;
//       float d = kd * derivative;

//       return p + i + d;
//     }
// };

// // =====================================================
// // PID Velocity Class (untuk inner loop cascaded yaw)
// // =====================================================
// // Inner loop PID: membandingkan desired yaw rate (dari outer loop)
// // dengan actual yaw rate (dari sensor, di-filter MA)
// // Output: torque/control signal ke SSY controller
// // =====================================================
// class PIDVelocity {
//   public:
//     float kp, ki, kd;
//     float integral;
//     float last_error;

//     PIDVelocity(float kp, float ki, float kd)
//       : kp(kp), ki(ki), kd(kd), integral(0), last_error(0) {}

//     void updateGains(float new_kp, float new_ki, float new_kd) {
//       kp = new_kp;
//       ki = new_ki;
//       kd = new_kd;
//     }

//     float calculate(float target_rate, float measured_rate) {
//       float error = target_rate - measured_rate;

//       // Integral
//       integral += error * LOOP_DT;
//       integral = constrain(integral, -50.0f, 50.0f); // anti windup

//       float derivative = (error - last_error) / LOOP_DT;
//       last_error = error;

//       return kp * error + ki * integral + kd * derivative;
//     }
// };

// // SSY Controller Class
// class SSYController {
//   public:
//     float d;

//     SSYController(float distance) {
//       d = distance;
//     }

//     void control(float Fx, float Fy, float tau, float thrust[4]) {
//       float M[4][4] = {
//         { -0.5f / sqrt2, 0.5f / sqrt2, d / 4.0f, 0.25f },
//         { 0.5f / sqrt2, 0.5f / sqrt2, -d / 4.0f, -0.25f },
//         { 0.5f / sqrt2, 0.5f / sqrt2, -d / 4.0f, 0.25f },
//         { -0.5f / sqrt2, 0.5f / sqrt2, d / 4.0f, -0.25f }
//       };

//       float F[4] = { Fx, Fy, 0, tau };

//       for (int i = 0; i < 4; i++) {
//         thrust[i] = 0;
//         for (int j = 0; j < 4; j++) {
//           thrust[i] += M[i][j] * F[j];
//         }
//       }
//     }
// };

// // DPR Controller Class
// class DPRController {
//   public:
//     float Lx, Ly;

//     DPRController(float lx, float ly) : Lx(lx), Ly(ly) {}

//     void control(float control_depth, float control_pitch, float control_roll, float thrust[4]) {
//       float A[3][4] = {
//         { Ly, -Ly, Ly, -Ly },
//         { Lx, Lx, -Lx, -Lx },
//         { 1, 1, 1, 1 }
//       };

//       float desired_control[3] = { control_depth, control_pitch, control_roll };

//       for (int i = 0; i < 4; i++) {
//         thrust[i] = 0;
//         for (int j = 0; j < 3; j++) {
//           thrust[i] += A[j][i] * desired_control[j];
//         }
//       }
//     }
// };

// // Controller instances
// SSYController ssyController(1.0);
// DPRController dprController(0.5, 0.5);

// // =====================================================
// // GLOBAL PID CONTROLLERS
// // =====================================================
// // HARUS global supaya integral & last_error tersimpan antar loop
// //
// // Yaw menggunakan cascaded PID:
// //   pid_yaw_outer = outer loop (heading error → desired yaw rate)
// //   pid_yaw_inner = inner loop (yaw rate error → torque)
// // =====================================================
// PID pid_yaw_outer(0, 0, 0);        // outer loop — gains dari ROS
// PIDVelocity pid_yaw_inner(0.5, 0.1, 0.01); // inner loop — gains hardcoded awal
// PID pid_roll(0, 0, 0);
// PID pid_pitch(0, 0, 0);
// PID pid_depth(0, 0, 0);
// PID pid_camera(0, 0, 0);

// // Helper functions
// float calculate_heading_error(float current, float target) {
//   float error = target - current;
//   if (error > 180) error -= 360;
//   else if (error < -180) error += 360;
//   return error;
// }

// // =====================================================
// // DEADBAND PER-THRUSTER FUNCTION
// // =====================================================
// int applyDeadband(int thruster_idx, float control_scaled)
// {
//   int pwm;

//   int fwd = THRUSTER_FORWARD_START[thruster_idx];
//   int rev = THRUSTER_REVERSE_START[thruster_idx];

//   if (control_scaled > 0)
//   {
//     pwm = fwd + (int)control_scaled;
//   }
//   else if (control_scaled < 0)
//   {
//     pwm = rev + (int)control_scaled;
//   }
//   else
//   {
//     pwm = PWM_NEUTRAL;
//   }

//   return constrain(pwm, PWM_MIN, PWM_MAX);
// }

// bool generate_is_stable(float thresh, float error) {
//   return (-thresh <= error) && (error <= thresh);
// }

// // Callback functions
// bool receive_status = false;  
// void status_callback(const void *msgin) {
//   const std_msgs__msg__String *status_msg = (const std_msgs__msg__String *)msgin;
  
//   String received_status = String(status_msg->data.data);
//   status = received_status;
//   receive_status = true;
// }

// bool receive_boost = false;
// void boost_callback(const void *msgin) {
//   const std_msgs__msg__Float32 *boost_msg = (const std_msgs__msg__Float32 *)msgin;

//   constrain_boost = boost_msg->data;
//   receive_boost = true;
// }

// bool receive_set_point = false;
// void set_point_callback(const void *msgin) {
//   const auv_interfaces__msg__SetPoint *set_point_msg = (const auv_interfaces__msg__SetPoint *)msgin;

//   set_point_yaw = set_point_msg->yaw;
//   set_point_pitch = set_point_msg->pitch;
//   set_point_roll = set_point_msg->roll;
//   set_point_depth = set_point_msg->depth;
//   receive_set_point = true;
// }

// bool recieve_pid = false;
// void pid_callback(const void *msgin) {
//   const auv_interfaces__msg__MultiPID *pid_msg = (const auv_interfaces__msg__MultiPID *)msgin;
  
//   kp_yaw = pid_msg->pid_yaw.kp;
//   ki_yaw = pid_msg->pid_yaw.ki;
//   kd_yaw = pid_msg->pid_yaw.kd;

//   kp_pitch = pid_msg->pid_pitch.kp;
//   ki_pitch = pid_msg->pid_pitch.ki;
//   kd_pitch = pid_msg->pid_pitch.kd;

//   kp_roll = pid_msg->pid_roll.kp;
//   ki_roll = pid_msg->pid_roll.ki;
//   kd_roll = pid_msg->pid_roll.kd;

//   kp_depth = pid_msg->pid_depth.kp;
//   ki_depth = pid_msg->pid_depth.ki;
//   kd_depth = pid_msg->pid_depth.kd;

//   kp_camera = pid_msg->pid_camera.kp;
//   ki_camera = pid_msg->pid_camera.ki;
//   kd_camera = pid_msg->pid_camera.kd;

//   recieve_pid = true;
// }

// void object_difference_callback(const void *msgin) {
//   const auv_interfaces__msg__ObjectDifference *object_difference_msg = (const auv_interfaces__msg__ObjectDifference *)msgin;
  
//   class_name = object_difference_msg->object_type.data;
//   camera_error = object_difference_msg->x_difference;
// }

// void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//   (void) last_call_time;
//   if (timer != NULL)
//   {
//     run_control_loop();

//     if (receive_status)
//     {
//       rcl_publish(&pub_status, &status_msg, NULL);
//       receive_status = false;
//     }

//     if (receive_set_point)
//     {
//       set_point_msg.depth = set_point_depth;
//       set_point_msg.roll = set_point_roll;
//       set_point_msg.pitch = set_point_pitch;
//       set_point_msg.yaw = set_point_yaw;

//       rcl_publish(&pub_set_point, &set_point_msg, NULL);
//       receive_set_point = false;
//     }

//     if (receive_boost)
//     {
//       boost_msg.data = constrain_boost;
//       rcl_publish(&pub_boost, &boost_msg, NULL);
//       receive_boost = false;
//     }

//     if (recieve_pid)
//     {
//       pid_msg.pid_yaw.kp = kp_yaw;
//       pid_msg.pid_yaw.ki = ki_yaw;
//       pid_msg.pid_yaw.kd = kd_yaw;

//       pid_msg.pid_pitch.kp = kp_pitch;
//       pid_msg.pid_pitch.ki = ki_pitch;
//       pid_msg.pid_pitch.kd = kd_pitch;

//       pid_msg.pid_roll.kp = kp_roll;
//       pid_msg.pid_roll.ki = ki_roll;
//       pid_msg.pid_roll.kd = kd_roll;

//       pid_msg.pid_depth.kp = kp_depth;
//       pid_msg.pid_depth.ki = ki_depth;
//       pid_msg.pid_depth.kd = kd_depth;

//       pid_msg.pid_camera.kp = kp_camera;
//       pid_msg.pid_camera.ki = ki_camera;
//       pid_msg.pid_camera.kd = kd_camera;

//       rcl_publish(&pub_pid, &pid_msg, NULL);
//       recieve_pid = false;
//     }

//     rcl_publish(&pub_pwm, &pwm_msg, NULL);
//     rcl_publish(&pub_error, &error_msg, NULL);
//     rcl_publish(&pub_sensor, &sensor_msg, NULL);
//   }
// }

// // Micro-ROS functions
// bool create_entities()
// {
//   const char *node_name = "teensy_node";
//   const char *ns = "";
//   const int domain_id = 0;

//   allocator = rcl_get_default_allocator();
//   init_options = rcl_get_zero_initialized_init_options();
//   rcl_init_options_init(&init_options, allocator);
//   rcl_init_options_set_domain_id(&init_options, domain_id);
//   rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
//   rclc_node_init_default(&node, node_name, ns, &support);

//   // Initialize publishers
//   rclc_publisher_init(
//       &pub_pwm,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Actuator),
//       "actuator_pwm", &rmw_qos_profile_default);

//   rclc_publisher_init(
//       &pub_error,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Error),
//       "error_msg", &rmw_qos_profile_default);

//   rclc_publisher_init(
//       &pub_sensor,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Sensor),
//       "sensor_msg", &rmw_qos_profile_default);

//   rclc_publisher_init(
//       &pub_set_point,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint),
//       "set_point_msg", &rmw_qos_profile_default);

//   rclc_publisher_init(
//       &pub_pid,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID),
//       "pid_msg", &rmw_qos_profile_default);

//   rclc_publisher_init(
//       &pub_status,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//       "status_msg", &rmw_qos_profile_default);

//   rclc_publisher_init(
//       &pub_boost,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//       "boost_msg", &rmw_qos_profile_default);

//   // Initialize subscribers
//   rclc_subscription_init(
//       &sub_status,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//       "status", &rmw_qos_profile_default);

//   rclc_subscription_init(
//       &sub_boost,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//       "boost", &rmw_qos_profile_default);

//   rclc_subscription_init(
//       &sub_pid,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID),
//       "pid", &rmw_qos_profile_default);

//   rclc_subscription_init(
//       &sub_set_point,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint),
//       "set_point", &rmw_qos_profile_default);

//   rclc_subscription_init(
//       &sub_object_difference,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, ObjectDifference),
//       "object_difference", &rmw_qos_profile_default);

//   const unsigned int timer_timeout = 10;
//   rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

//   unsigned int num_handles = 6;
//   executor = rclc_executor_get_zero_initialized_executor();
//   rclc_executor_init(&executor, &support.context, num_handles, &allocator);
//   rclc_executor_add_subscription(&executor, &sub_status, &status_msg, &status_callback, ON_NEW_DATA);
//   rclc_executor_add_subscription(&executor, &sub_boost, &boost_msg, &boost_callback, ON_NEW_DATA);
//   rclc_executor_add_subscription(&executor, &sub_pid, &pid_msg, &pid_callback, ON_NEW_DATA);
//   rclc_executor_add_subscription(&executor, &sub_set_point, &set_point_msg, &set_point_callback, ON_NEW_DATA);
//   rclc_executor_add_subscription(&executor, &sub_object_difference, &object_difference_msg, &object_difference_callback, ON_NEW_DATA);
//   rclc_executor_add_timer(&executor, &timer);

//   return true;
// }

// void destroy_entities() {
//   rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
//   (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

//   rcl_timer_fini(&timer);
//   rclc_executor_fini(&executor);
//   rcl_init_options_fini(&init_options);
//   rcl_node_fini(&node);
//   rclc_support_fini(&support);

//   rcl_publisher_fini(&pub_pwm, &node);
//   rcl_publisher_fini(&pub_error, &node);
//   rcl_publisher_fini(&pub_sensor, &node);
//   rcl_publisher_fini(&pub_set_point, &node);
//   rcl_publisher_fini(&pub_pid, &node);
//   rcl_publisher_fini(&pub_status, &node);
//   rcl_publisher_fini(&pub_boost, &node);

//   rcl_subscription_fini(&sub_status, &node);
//   rcl_subscription_fini(&sub_boost, &node);
//   rcl_subscription_fini(&sub_pid, &node);
//   rcl_subscription_fini(&sub_set_point, &node);
//   rcl_subscription_fini(&sub_object_difference, &node);
// }

// // Sensor communication functions for HWT905
// static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
//   Serial6.write(p_data, uiSize);
//   Serial6.flush();
// }

// static void Delayms(uint16_t ucMs) {
//   delay(ucMs);
// }

// static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
//   for (uint32_t i = 0; i < uiRegNum; i++) {
//     if (uiReg == AZ) {
//       s_cDataUpdate |= ACC_UPDATE;
//     }
//     uiReg++;
//   }
// }

// static void AutoScanSensor(void) {
//   int iRetry;
//   for (size_t i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
//     Serial6.begin(115200);
//     Serial6.flush();
//     iRetry = 2;
//     s_cDataUpdate = 0;
//     do {
//       WitReadReg(AX, 3);
//       delay(200);
//       while (Serial6.available()) {
//         WitSerialDataIn(Serial6.read());
//       }
//       if (s_cDataUpdate != 0) {
//         Serial.print(c_uiBaud[i]);
//         Serial.print(" baud find sensor\r\n\r\n");
//         return;
//       }
//       iRetry--;
//     } while (iRetry);
//   }
//   Serial.print("Cannot find sensor\r\n");
// }

// void setup() {
//   // Initialize serial communication
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);

//   // Initialize sensor communication
//   pinMode(DE_RE, OUTPUT);
//   digitalWrite(DE_RE, LOW);

//   Serial7.begin(115200, SERIAL_8N1);

//   nodemod.begin(0x50, Serial7);

//   nodemod.preTransmission(preTransmission);
//   nodemod.postTransmission(postTransmission);

//   Serial6.begin(115200); // Pitch & Roll
//   WitInit(WIT_PROTOCOL_NORMAL, 0x50);
//   WitSerialWriteRegister(SensorUartSend);
//   WitRegisterCallBack(SensorDataUpdata);
//   WitDelayMsRegister(Delayms);
//   AutoScanSensor();

//   for (int i = 0; i < 10; i++) {
//     if (!thruster[i].attach(pin_thruster[i])) {
//       Serial.printf("Failed to attach thruster %d\n", i);
//     }
//     thruster[i].writeMicroseconds(1500);
//   }

//   // camera initialize servo
//   if (!camera.attach(17)) {
//     Serial.println("Failed to attach camera servo");
//   }

//   camera.write(100);

//   // Initialize MS5837 depth sensor
//   Wire.begin();
//   int attempts = 0;
//   while (!sensor.init() && attempts < 5) {
//     Serial.println("Init failed! Retrying...");
//     delay(1000);
//     attempts++;
//   }
//   if (attempts >= 5) {
//     Serial.println("Failed to initialize depth sensor after 5 attempts");
//   } else {
//     sensor.setModel(MS5837::MS5837_30BA);
//     sensor.setFluidDensity(997);
//     Serial.println("Depth sensor initialized successfully");
//   }

//   // Initialize messages
//   auv_interfaces__msg__Actuator__init(&pwm_msg);
//   auv_interfaces__msg__SetPoint__init(&set_point_msg);
//   auv_interfaces__msg__MultiPID__init(&pid_msg);
//   auv_interfaces__msg__Error__init(&error_msg);
//   auv_interfaces__msg__ObjectDifference__init(&object_difference_msg);
//   auv_interfaces__msg__Sensor__init(&sensor_msg);
//   std_msgs__msg__String__init(&status_msg);
//   std_msgs__msg__Float32__init(&boost_msg);

//   status_msg.data.data = malloc(50);
//   status_msg.data.capacity = 50;
//   status_msg.data.size = 0;

//   // Initialize state
//   state = WAITING_AGENT;
//   Serial.println("Setup completed");
// }

// // =====================================================
// // CONTROL LOOP
// // =====================================================
// // Perubahan utama dari main_deadband_per_thruster.cpp:
// //   1. Semua sensor di-filter Moving Average
// //   2. Yaw menggunakan Cascaded PID (inner-outer loop)
// //   3. Yaw rate MA dipakai sebagai feedback inner loop
// // =====================================================
// void run_control_loop()
// {
//   // =====================================================
//   // 1. BACA PITCH & ROLL (HWT905 via Serial6) + MA
//   // =====================================================
//   while (Serial6.available())
//   {
//     WitSerialDataIn(Serial6.read());
//   }

//   if (s_cDataUpdate & ACC_UPDATE)
//   {
//     float raw_pitch = sReg[AY] / 32768.0f * 16.0f;
//     float raw_roll  = sReg[AX] / 32768.0f * 16.0f;

//     pitch = ma_pitch.update(raw_pitch);
//     roll  = ma_roll.update(raw_roll);

//     s_cDataUpdate &= ~ACC_UPDATE;
//   }

//   // =====================================================
//   // 2. BACA KOMPAS / YAW (HWT3100 via Modbus) + MA
//   // =====================================================
//   uint8_t result = nodemod.readHoldingRegisters(0xDE, 1);

//   if (result == nodemod.ku8MBSuccess)
//   {
//     int16_t raw = nodemod.getResponseBuffer(0);

//     last_yaw = yaw;

//     // RAW → derajat, normalisasi 0-360
//     float raw_yaw = raw / 10.0f;
//     raw_yaw = -raw_yaw;
//     if (raw_yaw < 0) raw_yaw += 360.0f;
//     if (raw_yaw >= 360.0) raw_yaw -= 360.0f;

//     // Moving Average pada yaw
//     yaw = ma_yaw.update(raw_yaw);

//     // =====================================================
//     // HITUNG YAW RATE (untuk inner loop cascaded PID)
//     // =====================================================
//     // Yaw rate = perubahan heading / delta time
//     // Di-filter MA untuk mengurangi noise
//     // =====================================================
//     unsigned long now = millis();
//     float dt = (now - last_compass_time) / 1000.0f;

//     if (dt > 0.001 && last_compass_time > 0)
//     {
//       float delta = calculate_heading_error(last_yaw, yaw);
//       float raw_yaw_rate = delta / dt;

//       // Reject spike (noise dari sensor)
//       if (fabs(raw_yaw_rate) < 400.0f)
//       {
//         yaw_rate_filtered = ma_yaw_rate.update(raw_yaw_rate);
//       }
//     }

//     last_compass_time = now;
//   }

//   // =====================================================
//   // 3. BACA DEPTH (MS5837) + MA
//   // =====================================================
//   sensor.read();
//   float raw_depth = sensor.depth();
//   depth = ma_depth.update(raw_depth);

//   // =====================================================
//   // 4. HITUNG ERROR
//   // =====================================================
//   error_yaw   = calculate_heading_error(yaw, set_point_yaw);
//   error_pitch = set_point_pitch - pitch;
//   error_roll  = set_point_roll - roll;
//   error_depth = set_point_depth - depth;

//   // Check stability
//   is_stable_roll  = generate_is_stable(0, error_roll);
//   is_stable_pitch = generate_is_stable(0, error_pitch);
//   is_stable_yaw   = generate_is_stable(0, error_yaw);
//   is_stable_depth = generate_is_stable(0, error_depth);

//   // Zero error if stable
//   error_roll  = is_stable_roll  ? 0 : error_roll;
//   error_pitch = is_stable_pitch ? 0 : error_pitch;
//   error_yaw   = is_stable_yaw   ? 0 : error_yaw;
//   error_depth = is_stable_depth ? 0 : error_depth;

//   // =====================================================
//   // 5. UPDATE PID GAINS
//   // =====================================================
//   // Outer loop yaw: gains dari ROS subscriber
//   pid_yaw_outer.updateGains(kp_yaw, ki_yaw, kd_yaw);
//   // Inner loop yaw: gains hardcoded (bisa diubah di defines di atas)
//   pid_yaw_inner.updateGains(kp_yaw_inner, ki_yaw_inner, kd_yaw_inner);

//   pid_roll.updateGains(kp_roll, ki_roll, kd_roll);
//   pid_pitch.updateGains(kp_pitch, ki_pitch, kd_pitch);
//   pid_depth.updateGains(kp_depth, ki_depth, kd_depth);
//   pid_camera.updateGains(kp_camera, ki_camera, kd_camera);

//   // =====================================================
//   // 6. CASCADED PID UNTUK YAW
//   // =====================================================
//   // OUTER LOOP: heading error → desired yaw rate (deg/s)
//   //   "Seberapa cepat saya harus berputar untuk mencapai setpoint?"
//   //
//   // INNER LOOP: yaw rate error → control signal (torque)
//   //   "Seberapa kuat thruster harus mendorong untuk mencapai kecepatan putar yang diinginkan?"
//   // =====================================================

//   // Outer loop: heading error → desired yaw rate
//   float desired_yaw_rate = pid_yaw_outer.calculate(error_yaw);
//   desired_yaw_rate = constrain(desired_yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE);

//   // Inner loop: desired yaw rate vs actual yaw rate → torque
//   float tau_yaw = pid_yaw_inner.calculate(desired_yaw_rate, yaw_rate_filtered);

//   // Scale supaya range cocok ke SSY controller
//   t_yaw = tau_yaw * 0.01f;

//   camera_yaw = -3 + ((pid_camera.calculate(camera_error) - (-500)) / (500 - (-500))) * (3 - (-3));

//   bool yaw_locked = false;
//   if (fabs(error_yaw) < 2.0)
//   {
//     yaw_locked = true;
//   }

//   // =====================================================
//   // 7. CONTROL LOGIC BASED ON STATUS
//   // =====================================================
//   if (status == "stop")
//   {
//     ssyController.control(0, 0, 0, thrust_ssy);
//     dprController.control(0, 0, 0, thrust_dpr);
//   }
//   else if (status == "all")
//   {
//     ssyController.control(0, 1, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "all_boost")
//   {
//     ssyController.control(0, 2, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "backward")
//   {
//     ssyController.control(0, -2, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "all_slow")
//   {
//     ssyController.control(0, .5, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "last")
//   {
//     ssyController.control(0, 2, 0, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "last_slow")
//   {
//     ssyController.control(0, 1, 0, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "pitch")
//   {
//     ssyController.control(0, 0, 0, thrust_ssy);
//     dprController.control(pid_depth.calculate(0), pid_pitch.calculate(error_pitch), pid_roll.calculate(0), thrust_dpr);
//   }
//   else if (status == "roll")
//   {
//     ssyController.control(0, 0, 0, thrust_ssy);
//     dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "yaw")
//   {
//     ssyController.control(0, 0, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(0), thrust_dpr);
//   }
//   else if (status == "depth")
//   {
//     ssyController.control(0, 0, 0, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(0), pid_roll.calculate(0), thrust_dpr);
//   }
//   else if (status == "dpr")
//   {
//     ssyController.control(0, 0, 0, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "dpr_ssy")
//   {
//     ssyController.control(0, 0, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "camera")
//   {
//     ssyController.control(0, 1, camera_yaw, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "camera_sway")
//   {
//     ssyController.control((camera_yaw * 0.5), 0, (camera_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "camera_yaw")
//   {
//     ssyController.control(0, 0, (camera_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "camera_sway_forward")
//   {
//     ssyController.control(-(camera_yaw * 0.5), 1, (t_yaw * 0.5), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "sway_right_forward")
//   {
//     ssyController.control(-3, 1, (t_yaw + 2), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "sway_left_forward")
//   {
//     ssyController.control(3, 2, (t_yaw - 2), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "sway_right")
//   {
//     ssyController.control(-3, 0, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "sway_left")
//   {
//     ssyController.control(3, 0.4, (t_yaw), thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "yaw_right")
//   {
//     ssyController.control(0, 0, -0.3, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }
//   else if (status == "yaw_left")
//   {
//     ssyController.control(0, 0, 0.3, thrust_ssy);
//     dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(-error_roll), thrust_dpr);
//   }

//   // =====================================================
//   // 8. CALCULATE PWM WITH PER-THRUSTER DEADBAND
//   // =====================================================
//   // SSY thrusters (idx 0-3)
//   pwm_thruster[0] = applyDeadband(0, -(thrust_ssy[1] * 500.0));
//   pwm_thruster[1] = applyDeadband(1, -(thrust_ssy[0] * 500.0));
//   pwm_thruster[2] = applyDeadband(2, -(thrust_ssy[3] * 500.0));
//   pwm_thruster[3] = applyDeadband(3, -(thrust_ssy[2] * 500.0));

//   // DPR thrusters (idx 4-7)
//   pwm_thruster[4] = applyDeadband(4, -thrust_dpr[2]);
//   pwm_thruster[5] = applyDeadband(5, -thrust_dpr[1]);
//   pwm_thruster[6] = applyDeadband(6,  thrust_dpr[0]);
//   pwm_thruster[7] = applyDeadband(7,  thrust_dpr[3]);

//   // Boost thrusters (idx 8-9) — belum aktif
//   // pwm_thruster[8] = applyDeadband(8, -(thrust_ssy[1] * constrain_boost));
//   // pwm_thruster[9] = applyDeadband(9, -(thrust_ssy[0] * constrain_boost));

//   // Special case for all_boost status
//   if (status == "all_boost")
//   {
//     pwm_thruster[0] = 1500.0;
//     pwm_thruster[1] = 1500.0;
//     pwm_thruster[2] = 1500.0;
//     pwm_thruster[3] = 1500.0;
//   }

//   // Apply PWM to thrusters
//   for (int i = 0; i < 10; i++)
//   {
//     thruster[i].writeMicroseconds(pwm_thruster[i]);
//   }

//   // =====================================================
//   // 9. DEBUG OUTPUT
//   // =====================================================
//   Serial.println("----------------------");
//   Serial.print("sensor yaw = ");
//   Serial.println(yaw);
//   Serial.print("error yaw = ");
//   Serial.println(error_yaw);
//   Serial.print("yaw rate (filtered) = ");
//   Serial.println(yaw_rate_filtered);
//   Serial.print("desired yaw rate = ");
//   Serial.println(desired_yaw_rate);

//   Serial.print("sensor roll = ");
//   Serial.println(roll);
//   Serial.print("error roll = ");
//   Serial.println(error_roll);

//   Serial.print("sensor pitch = ");
//   Serial.println(pitch);
//   Serial.print("error pitch = ");
//   Serial.println(error_pitch);

//   Serial.print("sensor depth = ");
//   Serial.println(depth);
//   Serial.print("error depth = ");
//   Serial.println(error_depth);

//   Serial.print("pwm_thruster_1 = ");
//   Serial.println(pwm_thruster[0]);
//   Serial.print("pwm_thruster_2 = ");
//   Serial.println(pwm_thruster[1]);
//   Serial.print("pwm_thruster_3 = ");
//   Serial.println(pwm_thruster[2]);
//   Serial.print("pwm_thruster_4 = ");
//   Serial.println(pwm_thruster[3]);
//   Serial.print("pwm_thruster_5 = ");
//   Serial.println(pwm_thruster[4]);
//   Serial.print("pwm_thruster_6 = ");
//   Serial.println(pwm_thruster[5]);
//   Serial.print("pwm_thruster_7 = ");
//   Serial.println(pwm_thruster[6]);
//   Serial.print("pwm_thruster_8 = ");
//   Serial.println(pwm_thruster[7]);
//   Serial.print("pwm_thruster_9 = ");
//   Serial.println(pwm_thruster[8]);
//   Serial.print("pwm_thruster_10 = ");
//   Serial.println(pwm_thruster[9]);

//   // =====================================================
//   // 10. UPDATE ROS MESSAGES
//   // =====================================================
//   pwm_msg.thruster_1 = pwm_thruster[0];
//   pwm_msg.thruster_2 = pwm_thruster[1];
//   pwm_msg.thruster_3 = pwm_thruster[2];
//   pwm_msg.thruster_4 = pwm_thruster[3];
//   pwm_msg.thruster_5 = pwm_thruster[4];
//   pwm_msg.thruster_6 = pwm_thruster[5];
//   pwm_msg.thruster_7 = pwm_thruster[6];
//   pwm_msg.thruster_8 = pwm_thruster[7];
//   pwm_msg.thruster_9 = pwm_thruster[8];
//   pwm_msg.thruster_10 = pwm_thruster[9];

//   // Update sensor message (nilai yang sudah di-filter MA)
//   sensor_msg.depth = depth;
//   sensor_msg.roll = roll;
//   sensor_msg.pitch = pitch;
//   sensor_msg.yaw = yaw;

//   // Update error message
//   error_msg.depth = error_depth;
//   error_msg.roll = error_roll;
//   error_msg.pitch = error_pitch;
//   error_msg.yaw = error_yaw;
//   error_msg.camera = float(camera_error);
// }

// void loop() {
//   switch (state) {
//     case WAITING_AGENT:
//       EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
//       break;
//     case AGENT_AVAILABLE:
//       state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//       if (state == WAITING_AGENT) {
//         destroy_entities();
//       };
//       break;
//     case AGENT_CONNECTED:
//       EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
//       if (state == AGENT_CONNECTED) {
//         Serial.println("Executor running...");
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
//       }
//       break;
//     case AGENT_DISCONNECTED:
//       destroy_entities();
//       state = WAITING_AGENT;
//       break;
//     default:
//       break;
//   }
// }