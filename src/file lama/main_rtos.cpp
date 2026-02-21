// #include <Arduino.h>
// #include <Servo.h>
// #include <REG.h>
// #include <wit_c_sdk.h>
// #include <stdint.h>
// #include <math.h>
// #undef TEMP
// #include <Wire.h>
// #include "MS5837.h"
// #include <task.h>
// #include <semphr.h>

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

// // ============== SHARED DATA STRUCTURE ==============
// typedef struct {
//   // Sensor data
//   float yaw, pitch, roll, depth;
//   float last_yaw, last_pitch, last_roll, last_depth;
//   float delta_yaw, delta_pitch, delta_roll, delta_depth;
  
//   // Setpoints
//   float set_point_yaw, set_point_pitch, set_point_roll, set_point_depth;
  
//   // Errors
//   float error_yaw, error_pitch, error_roll, error_depth;
  
//   // PID Coefficients
//   float kp_yaw, ki_yaw, kd_yaw;
//   float kp_pitch, ki_pitch, kd_pitch;
//   float kp_roll, ki_roll, kd_roll;
//   float kp_depth, ki_depth, kd_depth;
//   float kp_camera, ki_camera, kd_camera;
  
//   // Control outputs
//   float t_yaw, camera_yaw;
//   float thrust_dpr[4], thrust_ssy[4];
//   float pwm_thruster[10];
  
//   // Status
//   String status;
//   float constrain_boost;
//   int camera_error;
//   String class_name;
  
//   // Flags
//   bool receive_status, receive_boost, receive_set_point, receive_pid;
  
// } SensorControlData_t;

// // Global shared data
// SensorControlData_t sharedData;
// SemaphoreHandle_t dataMutex = NULL;

// // ============== MODBUS & SENSORS ==============
// ModbusMaster nodemod;
// #define DE_RE 20

// void preTransmission() {
//   digitalWrite(DE_RE, HIGH);
//   delayMicroseconds(50);
// }

// void postTransmission() {
//   delayMicroseconds(50);
//   digitalWrite(DE_RE, LOW);
// }

// // Initialize ModbusMaster HWT3100
// static volatile char s_cDataUpdate = 0;
// const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
// float sqrt2 = sqrt(2);
// MS5837 sensor;

// byte pin_thruster[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
// Servo thruster[10];
// Servo camera;

// // ============== PID CONTROLLER ==============
// class PID {
//   public:
//     PID(float kp = 0, float ki = 0, float kd = 0)
//       : kp(kp), ki(ki), kd(kd), integral(0), last_error(0) {}

//     float calculate(float error) {
//       float d_error = (error - last_error);
//       float proportional = kp * error;
//       integral += ki * error;
//       float derivative = kd * d_error;
//       last_error = error;
//       return proportional + integral + derivative;
//     }

//   private:
//     float kp, ki, kd;
//     float integral;
//     float last_error;
// };

// // ============== CONTROLLER CLASSES ==============
// class SSYController {
//   public:
//     float d;
//     SSYController(float distance) : d(distance) {}

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

// SSYController ssyController(1.0);
// DPRController dprController(0.5, 0.5);

// // ============== HELPER FUNCTIONS ==============
// float calculate_heading_error(float current, float target) {
//   target = fmod(target, 360);
//   float error = target - current;
//   if (error > 180) error -= 360;
//   else if (error < -180) error += 360;
//   return error;
// }

// bool generate_is_stable(float thresh, float error) {
//   return (-thresh <= error) && (error <= thresh);
// }

// // ============== SENSOR READING TASK ==============
// void sensorReadingTask(void *pvParameters) {
//   TickType_t xLastWakeTime = xTaskGetTickCount();
  
//   for (;;) {
//     // Read pitch and roll from Serial6 (non-blocking)
//     while (Serial6.available()) {
//       WitSerialDataIn(Serial6.read());
//     }

//     if (s_cDataUpdate & ACC_UPDATE) {
//       float new_pitch = sReg[AY] / 32768.0f * 16.0f;
//       float new_roll = sReg[AX] / 32768.0f * 16.0f;

//       xSemaphoreTake(dataMutex, portMAX_DELAY);
//       sharedData.last_pitch = sharedData.pitch;
//       sharedData.last_roll = sharedData.roll;
//       sharedData.pitch = new_pitch;
//       sharedData.roll = new_roll;
//       sharedData.delta_pitch = abs(sharedData.last_pitch - sharedData.pitch);
//       sharedData.delta_roll = abs(sharedData.last_roll - sharedData.roll);
//       xSemaphoreGive(dataMutex);

//       s_cDataUpdate &= ~ACC_UPDATE;
//     }

//     // Read compass (Modbus) - with reduced frequency
//     uint8_t result = nodemod.readHoldingRegisters(0xDE, 1);

//     if (result == nodemod.ku8MBSuccess) {
//       int16_t raw = nodemod.getResponseBuffer(0);
//       float new_yaw = raw / 10.0f;
//       new_yaw = -new_yaw;

//       if (new_yaw < 0) new_yaw += 360.0f;
//       if (new_yaw >= 360.0) new_yaw -= 360.0f;

//       xSemaphoreTake(dataMutex, portMAX_DELAY);
//       sharedData.last_yaw = sharedData.yaw;
//       sharedData.yaw = new_yaw;
//       sharedData.delta_yaw = abs(sharedData.last_yaw - sharedData.yaw);
//       xSemaphoreGive(dataMutex);
//     }

//     // Read depth sensor
//     sensor.read();
//     float new_depth = sensor.depth();

//     xSemaphoreTake(dataMutex, portMAX_DELAY);
//     sharedData.last_depth = sharedData.depth;
//     sharedData.depth = new_depth;
//     sharedData.delta_depth = abs(sharedData.last_depth - sharedData.depth);
//     xSemaphoreGive(dataMutex);

//     // Task runs at 50Hz (20ms)
//     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
//   }
// }

// // ============== PID CALCULATION TASK ==============
// PID pid_yaw(0, 0, 0);
// PID pid_pitch(0, 0, 0);
// PID pid_roll(0, 0, 0);
// PID pid_depth(0, 0, 0);
// PID pid_camera(0, 0, 0);

// void pidCalculationTask(void *pvParameters) {
//   TickType_t xLastWakeTime = xTaskGetTickCount();
  
//   for (;;) {
//     xSemaphoreTake(dataMutex, portMAX_DELAY);
    
//     // Update PID if parameters changed
//     static float last_kp = -999;
//     if (sharedData.kp_yaw != last_kp) {
//       pid_yaw = PID(sharedData.kp_yaw, sharedData.ki_yaw, sharedData.kd_yaw);
//       pid_pitch = PID(sharedData.kp_pitch, sharedData.ki_pitch, sharedData.kd_pitch);
//       pid_roll = PID(sharedData.kp_roll, sharedData.ki_roll, sharedData.kd_roll);
//       pid_depth = PID(sharedData.kp_depth, sharedData.ki_depth, sharedData.kd_depth);
//       pid_camera = PID(sharedData.kp_camera, sharedData.ki_camera, sharedData.kd_camera);
//       last_kp = sharedData.kp_yaw;
//     }

//     // Calculate errors
//     sharedData.error_yaw = calculate_heading_error(sharedData.yaw, sharedData.set_point_yaw);
//     sharedData.error_pitch = sharedData.set_point_pitch - sharedData.pitch;
//     sharedData.error_roll = sharedData.set_point_roll - sharedData.roll;
//     sharedData.error_depth = sharedData.set_point_depth - sharedData.depth;

//     // Check stability
//     bool is_stable_roll = generate_is_stable(0, sharedData.error_roll);
//     bool is_stable_pitch = generate_is_stable(0, sharedData.error_pitch);
//     bool is_stable_yaw = generate_is_stable(0, sharedData.error_yaw);
//     bool is_stable_depth = generate_is_stable(0.05, sharedData.error_depth);

//     // Zero error if stable
//     sharedData.error_roll = is_stable_roll ? 0 : sharedData.error_roll;
//     sharedData.error_pitch = is_stable_pitch ? 0 : sharedData.error_pitch;
//     sharedData.error_yaw = is_stable_yaw ? 0 : sharedData.error_yaw;
//     sharedData.error_depth = is_stable_depth ? 0 : sharedData.error_depth;

//     // Calculate control outputs
//     sharedData.t_yaw = -3 + ((pid_yaw.calculate(sharedData.error_yaw) - (-500)) / (500 - (-500))) * (3 - (-3));
//     sharedData.camera_yaw = -3 + ((pid_camera.calculate(sharedData.camera_error) - (-500)) / (500 - (-500))) * (3 - (-3));

//     xSemaphoreGive(dataMutex);

//     // Task runs at 100Hz (10ms)
//     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
//   }
// }

// // ============== THRUSTER EXECUTION TASK ==============
// void thrusterExecutionTask(void *pvParameters) {
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   float min_pwm = 1000.0, max_pwm = 2000.0;
  
//   for (;;) {
//     xSemaphoreTake(dataMutex, portMAX_DELAY);

//     // Control logic based on status
//     if (sharedData.status == "stop") {
//       ssyController.control(0, 0, 0, sharedData.thrust_ssy);
//       dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(0), sharedData.thrust_dpr);
//     }
//     else if (sharedData.status == "all") {
//       ssyController.control(0, 1, sharedData.t_yaw, sharedData.thrust_ssy);
//       dprController.control(pid_depth.calculate(sharedData.error_depth), pid_pitch.calculate(sharedData.error_pitch), pid_roll.calculate(-sharedData.error_roll), sharedData.thrust_dpr);
//     }
//     else if (sharedData.status == "all_boost") {
//       ssyController.control(0, 2, sharedData.t_yaw, sharedData.thrust_ssy);
//       dprController.control(pid_depth.calculate(sharedData.error_depth), pid_pitch.calculate(sharedData.error_pitch), pid_roll.calculate(-sharedData.error_roll), sharedData.thrust_dpr);
//     }
//     else if (sharedData.status == "backward") {
//       ssyController.control(0, -2, sharedData.t_yaw, sharedData.thrust_ssy);
//       dprController.control(pid_depth.calculate(sharedData.error_depth), pid_pitch.calculate(sharedData.error_pitch), pid_roll.calculate(-sharedData.error_roll), sharedData.thrust_dpr);
//     }
//     else if (sharedData.status == "dpr") {
//       ssyController.control(0, 0, 0, sharedData.thrust_ssy);
//       dprController.control(pid_depth.calculate(sharedData.error_depth), pid_pitch.calculate(sharedData.error_pitch), pid_roll.calculate(-sharedData.error_roll), sharedData.thrust_dpr);
//     }
//     else {
//       // Default behavior
//       ssyController.control(0, 0, 0, sharedData.thrust_ssy);
//       dprController.control(pid_depth.calculate(0), pid_pitch.calculate(0), pid_roll.calculate(0), sharedData.thrust_dpr);
//     }

//     // Calculate PWM values
//     sharedData.pwm_thruster[0] = constrain(1500 - (sharedData.thrust_ssy[1] * 500.0), min_pwm, max_pwm);
//     sharedData.pwm_thruster[1] = constrain(1500 - (sharedData.thrust_ssy[0] * 500.0), min_pwm, max_pwm);
//     sharedData.pwm_thruster[2] = constrain(1500 - (sharedData.thrust_ssy[3] * 500.0), min_pwm, max_pwm);
//     sharedData.pwm_thruster[3] = constrain(1500 - (sharedData.thrust_ssy[2] * 500.0), min_pwm, max_pwm);
//     sharedData.pwm_thruster[4] = constrain(1500 - sharedData.thrust_dpr[2], min_pwm, max_pwm);
//     sharedData.pwm_thruster[5] = constrain(1500 - sharedData.thrust_dpr[1], min_pwm, max_pwm);
//     sharedData.pwm_thruster[6] = constrain(1500 + sharedData.thrust_dpr[0], min_pwm, max_pwm);
//     sharedData.pwm_thruster[7] = constrain(1500 + sharedData.thrust_dpr[3], min_pwm, max_pwm);
//     sharedData.pwm_thruster[8] = constrain(1500 - (sharedData.thrust_ssy[1] * sharedData.constrain_boost), (1500.0 - sharedData.constrain_boost), (1500.0 + sharedData.constrain_boost));
//     sharedData.pwm_thruster[9] = constrain(1500 - (sharedData.thrust_ssy[0] * sharedData.constrain_boost), (1500.0 - sharedData.constrain_boost), (1500.0 + sharedData.constrain_boost));

//     xSemaphoreGive(dataMutex);

//     // Apply PWM to thrusters
//     for (int i = 0; i < 10; i++) {
//       thruster[i].writeMicroseconds(sharedData.pwm_thruster[i]);
//     }

//     // Task runs at 100Hz (10ms)
//     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
//   }
// }

// // ============== ROS COMMUNICATION TASK ==============
// // ROS global variables
// rclc_support_t support;
// rcl_init_options_t init_options;
// rcl_node_t node;
// rcl_timer_t timer;
// rclc_executor_t executor;
// rcl_allocator_t allocator;

// rcl_publisher_t pub_pwm, pub_error, pub_sensor, pub_set_point, pub_pid, pub_status, pub_boost;
// rcl_subscription_t sub_status, sub_boost, sub_pid, sub_set_point, sub_object_difference;

// auv_interfaces__msg__Actuator pwm_msg;
// auv_interfaces__msg__SetPoint set_point_msg;
// auv_interfaces__msg__MultiPID pid_msg;
// auv_interfaces__msg__Error error_msg;
// auv_interfaces__msg__Sensor sensor_msg;
// std_msgs__msg__String status_msg;
// std_msgs__msg__Float32 boost_msg;

// enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state = WAITING_AGENT;

// // ROS Callbacks
// void status_callback(const void *msgin) {
//   const std_msgs__msg__String *status_msg = (const std_msgs__msg__String *)msgin;
//   String received_status = String(status_msg->data.data);
//   xSemaphoreTake(dataMutex, portMAX_DELAY);
//   sharedData.status = received_status;
//   sharedData.receive_status = true;
//   xSemaphoreGive(dataMutex);
// }

// void boost_callback(const void *msgin) {
//   const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
//   xSemaphoreTake(dataMutex, portMAX_DELAY);
//   sharedData.constrain_boost = msg->data;
//   sharedData.receive_boost = true;
//   xSemaphoreGive(dataMutex);
// }

// void set_point_callback(const void *msgin) {
//   const auv_interfaces__msg__SetPoint *msg = (const auv_interfaces__msg__SetPoint *)msgin;
//   xSemaphoreTake(dataMutex, portMAX_DELAY);
//   sharedData.set_point_yaw = msg->yaw;
//   sharedData.set_point_pitch = msg->pitch;
//   sharedData.set_point_roll = msg->roll;
//   sharedData.set_point_depth = msg->depth;
//   sharedData.receive_set_point = true;
//   xSemaphoreGive(dataMutex);
// }

// void pid_callback(const void *msgin) {
//   const auv_interfaces__msg__MultiPID *msg = (const auv_interfaces__msg__MultiPID *)msgin;
//   xSemaphoreTake(dataMutex, portMAX_DELAY);
//   sharedData.kp_yaw = msg->pid_yaw.kp;
//   sharedData.ki_yaw = msg->pid_yaw.ki;
//   sharedData.kd_yaw = msg->pid_yaw.kd;
//   sharedData.kp_pitch = msg->pid_pitch.kp;
//   sharedData.ki_pitch = msg->pid_pitch.ki;
//   sharedData.kd_pitch = msg->pid_pitch.kd;
//   sharedData.kp_roll = msg->pid_roll.kp;
//   sharedData.ki_roll = msg->pid_roll.ki;
//   sharedData.kd_roll = msg->pid_roll.kd;
//   sharedData.kp_depth = msg->pid_depth.kp;
//   sharedData.ki_depth = msg->pid_depth.ki;
//   sharedData.kd_depth = msg->pid_depth.kd;
//   sharedData.kp_camera = msg->pid_camera.kp;
//   sharedData.ki_camera = msg->pid_camera.ki;
//   sharedData.kd_camera = msg->pid_camera.kd;
//   sharedData.receive_pid = true;
//   xSemaphoreGive(dataMutex);
// }

// void object_difference_callback(const void *msgin) {
//   const auv_interfaces__msg__ObjectDifference *msg = (const auv_interfaces__msg__ObjectDifference *)msgin;
//   xSemaphoreTake(dataMutex, portMAX_DELAY);
//   sharedData.class_name = String(msg->object_type.data);
//   sharedData.camera_error = msg->x_difference;
//   xSemaphoreGive(dataMutex);
// }

// void rosCommTask(void *pvParameters) {
//   for (;;) {
//     // Publish sensor and control data
//     xSemaphoreTake(dataMutex, portMAX_DELAY);

//     // Update messages with current data
//     pwm_msg.thruster_1 = sharedData.pwm_thruster[0];
//     pwm_msg.thruster_2 = sharedData.pwm_thruster[1];
//     pwm_msg.thruster_3 = sharedData.pwm_thruster[2];
//     pwm_msg.thruster_4 = sharedData.pwm_thruster[3];
//     pwm_msg.thruster_5 = sharedData.pwm_thruster[4];
//     pwm_msg.thruster_6 = sharedData.pwm_thruster[5];
//     pwm_msg.thruster_7 = sharedData.pwm_thruster[6];
//     pwm_msg.thruster_8 = sharedData.pwm_thruster[7];
//     pwm_msg.thruster_9 = sharedData.pwm_thruster[8];
//     pwm_msg.thruster_10 = sharedData.pwm_thruster[9];

//     sensor_msg.yaw = sharedData.yaw;
//     sensor_msg.pitch = sharedData.pitch;
//     sensor_msg.roll = sharedData.roll;
//     sensor_msg.depth = sharedData.depth;

//     error_msg.yaw = sharedData.error_yaw;
//     error_msg.pitch = sharedData.error_pitch;
//     error_msg.roll = sharedData.error_roll;
//     error_msg.depth = sharedData.error_depth;
//     error_msg.camera = (float)sharedData.camera_error;

//     xSemaphoreGive(dataMutex);

//     // Publish data (non-blocking)
//     if (state == AGENT_CONNECTED) {
//       (void)rcl_publish(&pub_pwm, &pwm_msg, NULL);
//       (void)rcl_publish(&pub_error, &error_msg, NULL);
//       (void)rcl_publish(&pub_sensor, &sensor_msg, NULL);
//     }

//     // Task runs at 20Hz (50ms)
//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }

// // ============== SETUP & LOOP ==============
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
//     Serial6.begin(c_uiBaud[i]);
//     Serial6.flush();
//     iRetry = 2;
//     s_cDataUpdate = 0;
//     do {
//       WitReadReg(AX, 3);
//       delay(1);
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

// bool create_entities() {
//   const char *node_name = "teensy_node";
//   const char *ns = "";
//   const int domain_id = 0;

//   allocator = rcl_get_default_allocator();
//   init_options = rcl_get_zero_initialized_init_options();
//   (void)rcl_init_options_init(&init_options, allocator);
//   (void)rcl_init_options_set_domain_id(&init_options, domain_id);
//   rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
//   rclc_node_init_default(&node, node_name, ns, &support);

//   rclc_publisher_init(&pub_pwm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Actuator), "actuator_pwm", &rmw_qos_profile_default);
//   rclc_publisher_init(&pub_error, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Error), "error_msg", &rmw_qos_profile_default);
//   rclc_publisher_init(&pub_sensor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Sensor), "sensor_msg", &rmw_qos_profile_default);
//   rclc_publisher_init(&pub_set_point, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint), "set_point_msg", &rmw_qos_profile_default);
//   rclc_publisher_init(&pub_pid, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID), "pid_msg", &rmw_qos_profile_default);
//   rclc_publisher_init(&pub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status_msg", &rmw_qos_profile_default);
//   rclc_publisher_init(&pub_boost, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "boost_msg", &rmw_qos_profile_default);

//   rclc_subscription_init(&sub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status", &rmw_qos_profile_default);
//   rclc_subscription_init(&sub_boost, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "boost", &rmw_qos_profile_default);
//   rclc_subscription_init(&sub_pid, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID), "pid", &rmw_qos_profile_default);
//   rclc_subscription_init(&sub_set_point, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint), "set_point", &rmw_qos_profile_default);
//   rclc_subscription_init(&sub_object_difference, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, ObjectDifference), "object_difference", &rmw_qos_profile_default);

//   return true;
// }

// void destroy_entities() {
//   rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
//   (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

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

// void setup() {
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);

//   pinMode(DE_RE, OUTPUT);
//   digitalWrite(DE_RE, LOW);

//   Serial7.begin(9600, SERIAL_8N1);
//   nodemod.begin(0x00, Serial7);
//   nodemod.preTransmission(preTransmission);
//   nodemod.postTransmission(postTransmission);

//   Serial6.begin(115200);
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

//   if (!camera.attach(17)) {
//     Serial.println("Failed to attach camera servo");
//   }
//   camera.write(100);

//   Wire.begin();
//   int attempts = 0;
//   while (!sensor.init() && attempts < 5) {
//     Serial.println("Init failed! Retrying...");
//     delay(1);
//     attempts++;
//   }
//   if (attempts >= 5) {
//     Serial.println("Failed to initialize depth sensor");
//   } else {
//     sensor.setModel(MS5837::MS5837_30BA);
//     sensor.setFluidDensity(997);
//     Serial.println("Depth sensor initialized");
//   }

//   // Initialize messages
//   auv_interfaces__msg__Actuator__init(&pwm_msg);
//   auv_interfaces__msg__SetPoint__init(&set_point_msg);
//   auv_interfaces__msg__MultiPID__init(&pid_msg);
//   auv_interfaces__msg__Error__init(&error_msg);
//   auv_interfaces__msg__Sensor__init(&sensor_msg);
//   std_msgs__msg__String__init(&status_msg);
//   std_msgs__msg__Float32__init(&boost_msg);

//   status_msg.data.data = (char*)malloc(50);
//   status_msg.data.capacity = 50;
//   status_msg.data.size = 0;

//   // Initialize mutex
//   dataMutex = xSemaphoreCreateMutex();

//   // Initialize shared data
//   memset(&sharedData, 0, sizeof(SensorControlData_t));
//   sharedData.status = "stop";

//   // Create RTOS tasks
//   xTaskCreate(sensorReadingTask, "SensorRead", 2048, NULL, 2, NULL);    // Priority 2
//   xTaskCreate(pidCalculationTask, "PIDCalc", 2048, NULL, 2, NULL);      // Priority 2
//   xTaskCreate(thrusterExecutionTask, "ThrusterExec", 2048, NULL, 2, NULL); // Priority 2
//   xTaskCreate(rosCommTask, "ROSComm", 4096, NULL, 1, NULL);             // Priority 1

//   Serial.println("RTOS Tasks created successfully");
// }

// void loop() {
//   // Main loop just manages ROS agent connection
//   static unsigned long lastCheck = 0;

//   switch (state) {
//     case WAITING_AGENT:
//       if (millis() - lastCheck > 500) {
//         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
//         lastCheck = millis();
//       }
//       break;

//     case AGENT_AVAILABLE:
//       state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//       if (state == WAITING_AGENT) {
//         destroy_entities();
//       }
//       break;

//     case AGENT_CONNECTED:
//       if (millis() - lastCheck > 200) {
//         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
//         lastCheck = millis();

//         // Process ROS subscriptions
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
//       }
//       break;

//     case AGENT_DISCONNECTED:
//       destroy_entities();
//       state = WAITING_AGENT;
//       break;

//     default:
//       break;
//   }

//   vTaskDelay(1);  // Yield to scheduler
// }
