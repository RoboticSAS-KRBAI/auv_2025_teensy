//   #include <Servo.h>
//   #include <REG.h>
//   #include <wit_c_sdk.h>
//   #include <stdint.h>
//   #include <math.h>
//   #undef TEMP
//   #include <Wire.h>
//   #include "MS5837.h"
//   #define ACC_UPDATE 0x01
//   /////////////////////
//   /// For Micro ROS ///
//   /////////////////////
//   #include <Arduino.h>
//   #include <micro_ros_platformio.h>
//   #include <stdio.h>
//   #include <rcl/rcl.h>
//   #include <rcl/error_handling.h>
//   #include <rclc/rclc.h>
//   #include <rclc/executor.h>
//   #include <rmw_microros/rmw_microros.h>

//   // Message headers
//   #include <std_msgs/msg/string.h>
//   #include <std_msgs/msg/float32.h>
//   #include <rosidl_runtime_c/string_functions.h>
//   #include <auv_interfaces/msg/multi_pid.h>
//   #include <auv_interfaces/msg/set_point.h>
//   #include <auv_interfaces/msg/actuator.h>
//   #include <auv_interfaces/msg/error.h>
//   #include <auv_interfaces/msg/sensor.h>
//   #include <auv_interfaces/msg/object_difference.h>

//   /*
//   * Helper functions to help reconnect
//   */
//   #define EXECUTE_EVERY_N_MS(MS, X)  do { \
//       static volatile int64_t init = -1; \
//       if (init == -1) { init = uxr_millis();} \
//       if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
//     } while (0)

//   enum states {
//     WAITING_AGENT,
//     AGENT_AVAILABLE,
//     AGENT_CONNECTED,
//     AGENT_DISCONNECTED
//   } state;

//   // Declare rcl object
//   rclc_support_t support;
//   rcl_init_options_t init_options;
//   rcl_node_t node;
//   rcl_timer_t timer;
//   rclc_executor_t executor;
//   rcl_allocator_t allocator;

//   // Declare Publishers
//   rcl_publisher_t pub_pwm, pub_error, pub_sensor, pub_set_point, pub_pid, pub_status, pub_boost;

//   // Declare Subscribers
//   rcl_subscription_t sub_status, sub_boost, sub_pid, sub_set_point, sub_object_difference;

//   // Declare Messages
//   auv_interfaces__msg__Actuator pwm_msg;
//   auv_interfaces__msg__SetPoint set_point_msg;
//   auv_interfaces__msg__MultiPID pid_msg;
//   auv_interfaces__msg__Error error_msg;
//   auv_interfaces__msg__ObjectDifference object_difference_msg;
//   auv_interfaces__msg__Sensor sensor_msg;
//   std_msgs__msg__String status_msg;
//   std_msgs__msg__Float32 boost_msg;

//   // Sensor and control variables
//   float yaw = 0.0, last_yaw = 0.0, delta_yaw = 0.0;
//   float pitch = 0.0, last_pitch = 0.0, delta_pitch = 0.0;
//   float roll = 0.0, last_roll = 0.0, delta_roll = 0.0;
//   float depth = 0.0, last_depth = 0.0, delta_depth = 0.0;
//   float set_point_yaw = 0, set_point_pitch = 0, set_point_roll = 0, set_point_depth = 0;
//   float error_yaw = 0, error_pitch = 0, error_roll = 0, error_depth = 0;
//   bool is_stable_roll = true, is_stable_pitch = true, is_stable_yaw = true, is_stable_depth = true;
//   float thrust_dpr[4], thrust_ssy[4];
//   float t_yaw = 0, camera_yaw = 0;
//   int yawIndex = 0;
//   float min_pwm = 1000.0, max_pwm = 2000.0; //max_pwm = 2000.0
//   float constrain_boost = 150.0;
//   float pwm_thruster[10] = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0};
//   // bool control_loop_flag = false;

//   // Tambahkan variabel untuk filter yaw
//   float filtered_yaw = 0.0;
//   float prev_raw_yaw = 0.0;
//   bool firstRun_yaw = true;
//   const float max_delta_yaw = 30.0f; // Maksimum perubahan yaw yang diperbolehkan (derajat)
//   const float alpha_yaw = 0.7f;      // Faktor smoothing (0.1 = sangat smooth, 0.9 = sedikit smooth)

//   // PID coefficients
//   float kp_yaw = 0, ki_yaw = 0, kd_yaw = 0;
//   float kp_pitch = 0, ki_pitch = 0, kd_pitch = 0;
//   float kp_roll = 0, ki_roll = 0, kd_roll = 0;
//   float kp_depth = 0, ki_depth = 0, kd_depth = 0;
//   float kp_camera = 0, ki_camera = 0, kd_camera = 0;

//   String class_name = "None";
//   int camera_error = 0;
//   String status = "stop";

//   byte pin_thruster[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
//   Servo thruster[10];
//   MS5837 sensor;

//   // Sensor communication variables
//   static volatile char s_cDataUpdate = 0;
//   const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
//   float sqrt2 = sqrt(2);

//   // PID Controller Class
//   class PID {
//     public:
//       PID(float kp, float ki, float kd)
//         : kp(kp), ki(ki), kd(kd), integral(0), last_error(0) {}

//       float calculate(float error) {
//         float d_error = (error - last_error) / 10;

//         float proportional = kp * error;
//         integral += ki * error;
//         float derivative = kd * d_error;

//         last_error = error;

//         return proportional + integral + derivative;
//       }

//     private:
//       float kp, ki, kd;
//       float integral;
//       float last_error;
//   };

//   // SSY Controller Class
//   class SSYController {
//     public:
//       float d;

//       // SSYController(float distance) : d(distance) {}
//       SSYController(float distance) {
//         d = distance;
//       }

//       void control(float Fx, float Fy, float tau, float thrust[4]) {
//         float M[4][4] = {
//           { -0.5f / sqrt2, 0.5f / sqrt2, d / 4.0f, 0.25f },
//           { 0.5f / sqrt2, 0.5f / sqrt2, -d / 4.0f, -0.25f },
//           { 0.5f / sqrt2, 0.5f / sqrt2, -d / 4.0f, 0.25f },
//           { -0.5f / sqrt2, 0.5f / sqrt2, d / 4.0f, -0.25f }
//         };

//         float F[4] = { Fx, Fy, 0, tau };

//         for (int i = 0; i < 4; i++) {
//           thrust[i] = 0;
//           for (int j = 0; j < 4; j++) {
//             thrust[i] += M[i][j] * F[j];
//           }
//         }
//       }
//   };

//   // DPR Controller Class
//   class DPRController {
//     public:
//       float Lx, Ly;

//       DPRController(float lx, float ly) : Lx(lx), Ly(ly) {}

//       void control(float control_depth, float control_pitch, float control_roll, float thrust[4]) {
//         float A[3][4] = {
//           { Ly, -Ly, Ly, -Ly },
//           { Lx, Lx, -Lx, -Lx },
//           { 1, 1, 1, 1 }
//         };

//         float desired_control[3] = { control_depth, control_pitch, control_roll };

//         for (int i = 0; i < 4; i++) {
//           thrust[i] = 0;
//           for (int j = 0; j < 3; j++) {
//             thrust[i] += A[j][i] * desired_control[j];
//           }
//         }
//       }
//   };

//   // Controller instances
//   SSYController ssyController(1.0);
//   DPRController dprController(0.5, 0.5);

//   // Filter functions untuk yaw
//   float normalize_yaw(float yaw) {
//     while (yaw > 180) yaw -= 360;
//     while (yaw < -180) yaw += 360;
//     return yaw;
//   }

//   float yaw_difference(float a, float b) {
//     float diff = a - b;
//     while (diff > 180.0) diff -= 360.0;
//     while (diff < -180.0) diff += 360.0;
//     return diff;
// }

//   float filter_yaw(float raw_yaw) {
//     raw_yaw = normalize_yaw(raw_yaw);

//     if (firstRun_yaw) {
//       filtered_yaw = raw_yaw;
//       prev_raw_yaw = raw_yaw;
//       firstRun_yaw = false;
//       return filtered_yaw;
//     }

//     // Outlier rejection - deteksi dan reject spike
//     float yaw_diff = fabs(yaw_difference(raw_yaw, prev_raw_yaw));
//     if (yaw_diff > max_delta_yaw) {
//       // Jika perubahan terlalu besar, gunakan nilai sebelumnya
//       Serial.print("Yaw spike detected: ");
//       Serial.print(yaw_diff);
//       Serial.println(" degrees - using previous value");
//       raw_yaw = prev_raw_yaw;
//     }

//     // Exponential smoothing filter
//     filtered_yaw = alpha_yaw * filtered_yaw + (1 - alpha_yaw) * raw_yaw;
//     prev_raw_yaw = raw_yaw;

//     return filtered_yaw;
//   }

//   // Helper functions
//   float calculate_heading_error(float current, float target) {
//     float diff = (target - current) * M_PI / 180.0;  // ke radian
//     float err = atan2(sin(diff), cos(diff));        // -pi..pi
//     return err * 180.0 / M_PI;                      // balik ke derajat
//   }

//   bool generate_is_stable(float thresh, float error) {
//     return (-thresh <= error) && (error <= thresh);
//   }

//   // Callback functions
//   bool receive_status = false;  
//   void status_callback(const void *msgin) {
//     const std_msgs__msg__String *status_msg = (const std_msgs__msg__String *)msgin;
    
//     // receive message
//     String received_status = String(status_msg->data.data);
//     status = received_status;  // Menyimpan data status yang diterima
//     receive_status = true;
//   }

//   bool receive_boost = false;
//   void boost_callback(const void *msgin) {
//     const std_msgs__msg__Float32 *boost_msg = (const std_msgs__msg__Float32 *)msgin;

//     constrain_boost = boost_msg->data;
//     receive_boost = true;
//   }

//   bool receive_set_point = false;
//   void set_point_callback(const void *msgin) {
//     const auv_interfaces__msg__SetPoint *set_point_msg = (const auv_interfaces__msg__SetPoint *)msgin;

//     set_point_yaw = set_point_msg->yaw;
//     set_point_pitch = set_point_msg->pitch;
//     set_point_roll = set_point_msg->roll;
//     set_point_depth = set_point_msg->depth;
//     receive_set_point = true;
//   }

//   bool recieve_pid = false;
//   void pid_callback(const void *msgin) {
//     const auv_interfaces__msg__MultiPID *pid_msg = (const auv_interfaces__msg__MultiPID *)msgin;
    
//     kp_yaw = pid_msg->pid_yaw.kp;
//     ki_yaw = pid_msg->pid_yaw.ki;
//     kd_yaw = pid_msg->pid_yaw.kd;

//     kp_pitch = pid_msg->pid_pitch.kp;
//     ki_pitch = pid_msg->pid_pitch.ki;
//     kd_pitch = pid_msg->pid_pitch.kd;

//     kp_roll = pid_msg->pid_roll.kp;
//     ki_roll = pid_msg->pid_roll.ki;
//     kd_roll = pid_msg->pid_roll.kd;

//     kp_depth = pid_msg->pid_depth.kp;
//     ki_depth = pid_msg->pid_depth.ki;
//     kd_depth = pid_msg->pid_depth.kd;

//     kp_camera = pid_msg->pid_camera.kp;
//     ki_camera = pid_msg->pid_camera.ki;
//     kd_camera = pid_msg->pid_camera.kd;

//     recieve_pid = true;
//   }

//   void object_difference_callback(const void *msgin) {
//     const auv_interfaces__msg__ObjectDifference *object_difference_msg = (const auv_interfaces__msg__ObjectDifference *)msgin;
    
//     class_name = object_difference_msg->object_type.data;
//     camera_error = object_difference_msg->x_difference;
//   }

//   void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//     (void) last_call_time;
//     if (timer != NULL) {
//       // control_loop_flag = true;
//       // timer callback code here

//       /*
//         TODO : Publish anything inside here
        
//         For example, we are going to echo back
//         the int16array_sub data to int16array_pub data,
//         so we could see the data reflect each other.
//         And also keep incrementing the int16_pub
//       */
//     if(receive_status) {
//       // Publish
//       rcl_publish(&pub_status, &status_msg, NULL);

//       receive_status = false;
//     }

//     if (receive_set_point) {
//       set_point_msg.depth = set_point_depth;
//       set_point_msg.roll = set_point_roll;
//       set_point_msg.pitch = set_point_pitch;
//       set_point_msg.yaw = set_point_yaw;

//       // publish
//       rcl_publish(&pub_set_point, &set_point_msg, NULL);

//       receive_set_point = false;
//     }

//     if (receive_boost) {
//       boost_msg.data = constrain_boost;

//       // publish
//       rcl_publish(&pub_boost, &boost_msg, NULL);

//       receive_boost = false;
//     }

//     if (recieve_pid) {
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

//       // publish
//       rcl_publish(&pub_pid, &pid_msg, NULL);

//       recieve_pid = false;
//     }

//       rcl_publish(&pub_pwm, &pwm_msg, NULL);
//       rcl_publish(&pub_error, &error_msg, NULL);
//       rcl_publish(&pub_sensor, &sensor_msg, NULL);

//     }
//   }

//   // Micro-ROS functions
//   bool create_entities() {
//     const char * node_name = "teensy_node";
//     const char * ns = "";
//     const int domain_id = 0;

//     // Initialize node

//     allocator = rcl_get_default_allocator();
//     init_options = rcl_get_zero_initialized_init_options();
//     rcl_init_options_init(&init_options, allocator);
//     rcl_init_options_set_domain_id(&init_options, domain_id);
//     rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
//     rclc_node_init_default(&node, node_name, ns, &support);

//     // Initialize publishers
//       rclc_publisher_init(
//           &pub_pwm,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Actuator),
//           "actuator_pwm", &rmw_qos_profile_default);

//       rclc_publisher_init(
//           &pub_error,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Error),
//           "error_msg", &rmw_qos_profile_default);

//       rclc_publisher_init(
//           &pub_sensor,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Sensor),
//           "sensor_msg", &rmw_qos_profile_default);

//       rclc_publisher_init(
//           &pub_set_point,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint),
//           "set_point_msg", &rmw_qos_profile_default);

//       rclc_publisher_init(
//           &pub_pid,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID),
//           "pid_msg", &rmw_qos_profile_default);

//       rclc_publisher_init(
//           &pub_status,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//           "status_msg", &rmw_qos_profile_default);

//       rclc_publisher_init(
//           &pub_boost,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//           "boost_msg", &rmw_qos_profile_default);

//       // Initialize subscribers
//       rclc_subscription_init(
//           &sub_status,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//           "status", &rmw_qos_profile_default);

//       rclc_subscription_init(
//           &sub_boost,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//           "boost", &rmw_qos_profile_default);

//       rclc_subscription_init(
//           &sub_pid,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID),
//           "pid", &rmw_qos_profile_default);

//       rclc_subscription_init(
//           &sub_set_point,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint),
//           "set_point", &rmw_qos_profile_default);

//       rclc_subscription_init(
//           &sub_object_difference,
//           &node,
//           ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, ObjectDifference),
//           "object_difference", &rmw_qos_profile_default);

//       /*
//     * Init timer_callback
//     * TODO : change timer_timeout
//     * 50ms : 20Hz
//     * 20ms : 50Hz
//     * 10ms : 100Hz
//     */
//     const unsigned int timer_timeout = 50;
//     rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

//     /*
//     * Init Executor
//     * TODO : make sure the num_handles is correct
//     * num_handles = total_of_subscriber + timer
//     * publisher is not counted
//     * 
//     * TODO : make sure the name of sub msg and callback are correct
//     */
//     unsigned int num_handles = 6;
//     executor = rclc_executor_get_zero_initialized_executor();
//     rclc_executor_init(&executor, &support.context, num_handles, &allocator);
//     rclc_executor_add_subscription(&executor, &sub_status, &status_msg, &status_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &sub_boost, &boost_msg, &boost_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &sub_pid, &pid_msg, &pid_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &sub_set_point, &set_point_msg, &set_point_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &sub_object_difference, &object_difference_msg, &object_difference_callback, ON_NEW_DATA);
//     rclc_executor_add_timer(&executor, &timer);

//     return true;
//   }

//   void destroy_entities() {
//     rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
//     (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

//     // Clean up all the created objects
//     rcl_timer_fini(&timer);
//     rclc_executor_fini(&executor);
//     rcl_init_options_fini(&init_options);
//     rcl_node_fini(&node);
//     rclc_support_fini(&support);

//     // Destroy publishers
//     rcl_publisher_fini(&pub_pwm, &node);
//     rcl_publisher_fini(&pub_error, &node);
//     rcl_publisher_fini(&pub_sensor, &node);
//     rcl_publisher_fini(&pub_set_point, &node);
//     rcl_publisher_fini(&pub_pid, &node);
//     rcl_publisher_fini(&pub_status, &node);
//     rcl_publisher_fini(&pub_boost, &node);

//     // Destroy subscribers
//     rcl_subscription_fini(&sub_status, &node);
//     rcl_subscription_fini(&sub_boost, &node);
//     rcl_subscription_fini(&sub_pid, &node);
//     rcl_subscription_fini(&sub_set_point, &node);
//     rcl_subscription_fini(&sub_object_difference, &node);
//   }

//   // Sensor communication functions
//   static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
//     Serial8.write(p_data, uiSize);
//     Serial8.flush();
//   }

//   static void Delayms(uint16_t ucMs) {
//     delay(ucMs);
//   }

//   static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
//     for (uint32_t i = 0; i < uiRegNum; i++) {
//       if (uiReg == AZ) {
//         s_cDataUpdate |= ACC_UPDATE;
//       }
//       uiReg++;
//     }
//   }

//   static void AutoScanSensor(void) {
//     int iRetry;
//     for (size_t i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
//       Serial8.begin(c_uiBaud[i]);
//       Serial8.flush();
//       iRetry = 2;
//       s_cDataUpdate = 0;
//       do {
//         WitReadReg(AX, 3);
//         delay(200);
//         while (Serial8.available()) {
//           WitSerialDataIn(Serial8.read());
//         }
//         if (s_cDataUpdate != 0) {
//           Serial.print(c_uiBaud[i]);
//           Serial.print(" baud find sensor\r\n\r\n");
//           return;
//         }
//         iRetry--;
//       } while (iRetry);
//     }
//     Serial.print("Cannot find sensor\r\n");
//   }

//   void setup() {
//     // Initialize serial communication
//     Serial.begin(115200);
//     set_microros_serial_transports(Serial);

//     // Initialize sensor communication
//     Serial5.begin(115200); // Yaw
//     Serial8.begin(115200); // Pitch & Roll
//     WitInit(WIT_PROTOCOL_NORMAL, 0x50);
//     WitSerialWriteRegister(SensorUartSend);
//     WitRegisterCallBack(SensorDataUpdata);
//     WitDelayMsRegister(Delayms);
//     AutoScanSensor();

//     for (int i = 0; i < 10; i++) {
//       if (!thruster[i].attach(pin_thruster[i])) {
//         Serial.printf("Failed to attach thruster %d\n", i);
//       }
//       thruster[i].writeMicroseconds(1500); // Neutral position
//     }

//     // Initialize MS5837 depth sensor
//     Wire.begin();
//     int attempts = 0;
//     while (!sensor.init() && attempts < 5) {
//       Serial.println("Init failed! Retrying...");
//       delay(1000);
//       attempts++;
//     }
//     if (attempts >= 5) {
//       Serial.println("Failed to initialize depth sensor after 5 attempts");
//     } else {
//       sensor.setModel(MS5837::MS5837_30BA);
//       sensor.setFluidDensity(997);
//       Serial.println("Depth sensor initialized successfully");
//     }

//     // Initialize messages
//     auv_interfaces__msg__Actuator__init(&pwm_msg);
//     auv_interfaces__msg__SetPoint__init(&set_point_msg);
//     auv_interfaces__msg__MultiPID__init(&pid_msg);
//     auv_interfaces__msg__Error__init(&error_msg);
//     auv_interfaces__msg__ObjectDifference__init(&object_difference_msg);
//     auv_interfaces__msg__Sensor__init(&sensor_msg);
//     std_msgs__msg__String__init(&status_msg);
//     std_msgs__msg__Float32__init(&boost_msg);

//     status_msg.data.data = malloc(50);
//     status_msg.data.capacity = 50;
//     status_msg.data.size = 0;
//     // Initialize state
//     state = WAITING_AGENT;
//     Serial.println("Setup completed");
//   }

//   void run_control_loop() {
//   // Sensor data processing
//       delta_depth = abs(last_depth - depth);
//       delta_roll = abs(last_roll - roll);
//       delta_pitch = abs(last_pitch - pitch);
//       delta_yaw = abs(last_yaw - yaw);

//       // Read yaw from Serial5
//       if (Serial5.available()) {
//           String data_str = Serial5.readStringUntil('\n');
//           if (data_str.indexOf("Yaw:") != -1) {
//             yawIndex = data_str.indexOf("Yaw:") + 4;
//             float raw_yaw = data_str.substring(yawIndex).toFloat();
              
//             // Terapkan filter pada yaw
//             yaw = filter_yaw(raw_yaw);
//           }
//       }

//       // Read pitch and roll from Serial8
//       while (Serial8.available()) {
//           WitSerialDataIn(Serial8.read());
//       }

//       if (s_cDataUpdate & ACC_UPDATE) {
//           pitch = sReg[AX] / 32768.0f * 16.0f;
//           roll = sReg[AY] / 32768.0f * 16.0f;
//           s_cDataUpdate &= ~ACC_UPDATE;
//       }

//       // Update depth from sensor
//       sensor.read();
//       depth = sensor.depth();

//       // Filter sensor data
//       if (abs(depth) > 10) depth = last_depth;
//       if (delta_depth > 1) depth = last_depth;
//       if (delta_roll > 0.4) roll = last_roll;
//       if (delta_pitch > 0.3) pitch = last_pitch;

//       // Filter yaw dengan threshold yang lebih ketat
//       if (delta_yaw > 5.0) { // Reduced from mungkin sebelumnya lebih besar
//           yaw = last_yaw;
//       }

//       // Calculate errors
//       error_yaw = calculate_heading_error(yaw, set_point_yaw);

//       // Error calculation untuk pitch, roll, depth (tetap sama)
//       error_pitch = set_point_pitch - pitch;
//       error_roll = set_point_roll - roll;
//       error_depth = set_point_depth - depth;

//       // Check stability
//       is_stable_roll = generate_is_stable(0, error_roll);
//       is_stable_pitch = generate_is_stable(0, error_pitch);
//       is_stable_yaw = generate_is_stable(0, error_yaw);
//       is_stable_depth = generate_is_stable(0.05, error_depth);

//       // Zero error if stable
//       error_roll = is_stable_roll ? 0 : error_roll;
//       error_pitch = is_stable_pitch ? 0 : error_pitch;
//       error_yaw = is_stable_yaw ? 0 : error_yaw;
//       error_depth = is_stable_depth ? 0 : error_depth;

//       // Create PID controllers
//       PID pid_yaw(kp_yaw, ki_yaw, kd_yaw);
//       PID pid_roll(kp_roll, ki_roll, kd_roll);
//       PID pid_pitch(kp_pitch, ki_pitch, kd_pitch);
//       PID pid_depth(kp_depth, ki_depth, kd_depth);
//       PID pid_camera(kp_camera, ki_camera, kd_camera);

//       // Calculate control outputs
//       t_yaw = -3 + ((pid_yaw.calculate(error_yaw) - (-500)) / (500 - (-500))) * (3 - (-3));
//       camera_yaw = -3 + ((pid_camera.calculate(camera_error) - (-500)) / (500 - (-500))) * (3 - (-3));

//       // Serial.print("run_control_loop - status: ");
//       // Serial.println(status);

//       // Control logic based on status
//       if (status == "stop") {
//           ssyController.control(0, 0, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(0), thrust_dpr);
//       } 
//       else if (status == "all") {
//           ssyController.control(0, 2, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "all_boost") {
//           ssyController.control(0, 2, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "backward") {
//           ssyController.control(0, -2, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "all_slow") {
//           ssyController.control(0, 1, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "last") {
//           ssyController.control(0, 2, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "last_slow") {
//           ssyController.control(0, 1, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "pitch") {
//           ssyController.control(0, 0, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(0), pid_pitch.calculate(error_pitch), pid_roll.calculate(0), thrust_dpr);
//       }
//       else if (status == "roll") {
//           ssyController.control(0, 0, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "yaw") {
//           ssyController.control(0, 0, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(0), thrust_dpr);
//       }
//       else if (status == "depth") {
//           ssyController.control(0, 0, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(0), pid_roll.calculate(0), thrust_dpr);
//       }
//       else if (status == "dpr") {
//           ssyController.control(0, 0, 0, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "dpr_ssy") {
//           ssyController.control(0, 0, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "camera") {
//           ssyController.control(0, 1, camera_yaw, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "camera_sway") {
//           ssyController.control((camera_yaw * 0.5), 0, (camera_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "camera_yaw") {
//           ssyController.control(0, 0, (camera_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "camera_sway_forward") {
//           ssyController.control(-(camera_yaw * 0.5), 1, -(t_yaw * 0.5), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "sway_right_forward") {
//           ssyController.control(-3, 1, -(t_yaw + 2), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "sway_left_forward") {
//           ssyController.control(3, 2, -(t_yaw - 2), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "sway_right") {
//           ssyController.control(-3, 0, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "sway_left") {
//           ssyController.control(3, 0.4, -(t_yaw), thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "yaw_right") {
//           ssyController.control(0, 0, 0.3, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }
//       else if (status == "yaw_left") {
//           ssyController.control(0, 0, -0.3, thrust_ssy);
//           dprController.control(pid_depth.calculate(error_depth), pid_pitch.calculate(error_pitch), pid_roll.calculate(error_roll), thrust_dpr);
//       }

//       // Calculate PWM values
//       pwm_thruster[0] = constrain(1500 - (thrust_ssy[1] * 500.0), min_pwm, max_pwm);
//       pwm_thruster[1] = constrain(1500 - (thrust_ssy[0] * 500.0), min_pwm, max_pwm);
//       pwm_thruster[2] = constrain(1500 - (thrust_ssy[3] * 500.0), min_pwm, max_pwm);
//       pwm_thruster[3] = constrain(1500 - (thrust_ssy[2] * 500.0), min_pwm, max_pwm);
//       pwm_thruster[4] = constrain(1500 + thrust_dpr[2], min_pwm, max_pwm);
//       pwm_thruster[5] = constrain(1500 - thrust_dpr[1], min_pwm, max_pwm);
//       pwm_thruster[6] = constrain(1500 + thrust_dpr[0], min_pwm, max_pwm);
//       pwm_thruster[7] = constrain(1500 - thrust_dpr[3], min_pwm, max_pwm);
//       pwm_thruster[8] = constrain(1500 - (thrust_ssy[1] * constrain_boost), (1500.0 - constrain_boost), (1500.0 + constrain_boost));
//       pwm_thruster[9] = constrain(1500 - (thrust_ssy[0] * constrain_boost), (1500.0 - constrain_boost), (1500.0 + constrain_boost));

//       // Special case for all_boost status
//       if (status == "all_boost") {
//           pwm_thruster[0] = 1500.0;
//           pwm_thruster[1] = 1500.0;
//           pwm_thruster[2] = 1500.0;
//           pwm_thruster[3] = 1500.0;
//       }

//       // Apply PWM to thrusters
//       for (int i = 0; i < 10; i++) {
//           thruster[i].writeMicroseconds(pwm_thruster[i]);
//       }

//     Serial.println("----------------------");
//     Serial.print("sensor yaw = "); Serial.println(yaw);
//     Serial.print("error yaw = "); Serial.println(error_yaw);

//     Serial.print("sensor roll = "); Serial.println(roll);
//     Serial.print("error roll = "); Serial.println(error_roll);

//     Serial.print("sensor pitch = "); Serial.println(pitch);
//     Serial.print("error pitch = "); Serial.println(error_pitch);

//     Serial.print("sensor depth = "); Serial.println(depth);
//     Serial.print("error depth = "); Serial.println(error_depth);

//     Serial.print("pwm_thruster_1 = "); Serial.println(pwm_thruster[1]);
//     Serial.print("pwm_thruster_2 = "); Serial.println(pwm_thruster[2]);
//     Serial.print("pwm_thruster_3 = "); Serial.println(pwm_thruster[3]);
//     Serial.print("pwm_thruster_4 = "); Serial.println(pwm_thruster[4]);
//     Serial.print("pwm_thruster_5 = "); Serial.println(pwm_thruster[5]);
//     Serial.print("pwm_thruster_6 = "); Serial.println(pwm_thruster[6]);
//     Serial.print("pwm_thruster_7 = "); Serial.println(pwm_thruster[7]);
//     Serial.print("pwm_thruster_8 = "); Serial.println(pwm_thruster[8]);
//     Serial.print("pwm_thruster_9 = "); Serial.println(pwm_thruster[9]);
//     Serial.print("pwm_thruster_10 = "); Serial.println(pwm_thruster[10]);

//       // Update message data
//       pwm_msg.thruster_1 = pwm_thruster[0];
//       pwm_msg.thruster_2 = pwm_thruster[1];
//       pwm_msg.thruster_3 = pwm_thruster[2];
//       pwm_msg.thruster_4 = pwm_thruster[3];
//       pwm_msg.thruster_5 = pwm_thruster[4];
//       pwm_msg.thruster_6 = pwm_thruster[5];
//       pwm_msg.thruster_7 = pwm_thruster[6];
//       pwm_msg.thruster_8 = pwm_thruster[7];
//       pwm_msg.thruster_9 = pwm_thruster[8];
//       pwm_msg.thruster_10 = pwm_thruster[9];

//       // Update sensor message
//       sensor_msg.depth = depth;
//       sensor_msg.roll = roll;
//       sensor_msg.pitch = pitch;
//       sensor_msg.yaw = yaw;

//       last_depth = depth;
//       last_roll = roll;
//       last_pitch = pitch;
//       last_yaw = yaw;

//       // Update error message
//       error_msg.depth = error_depth;
//       error_msg.roll = error_roll;
//       error_msg.pitch = error_pitch;
//       error_msg.yaw = error_yaw;
//       error_msg.camera = float(camera_error);
      
//       delay(5);
//   }

//   void loop() {
//     switch (state) {
//       case WAITING_AGENT:
//         EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
//         break;
//       case AGENT_AVAILABLE:
//         state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//         if (state == WAITING_AGENT) {
//           destroy_entities();
//         };
//         break;
//       case AGENT_CONNECTED:
//         EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
//         if (state == AGENT_CONNECTED) {
//           Serial.println("Executor running...");
//           rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//           run_control_loop();
//         }
//         break;
//       case AGENT_DISCONNECTED:
//         destroy_entities();
//         state = WAITING_AGENT;
//         break;
//       default:
//         break;
//     }

//     // if (state == AGENT_CONNECTED) {
//     //   if (control_loop_flag) {
//     //       run_control_loop();
//     //       control_loop_flag = false;
//     //   }
//     // }
//   }