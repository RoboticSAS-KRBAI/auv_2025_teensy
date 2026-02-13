#include <Servo.h>
#include <REG.h>
#include <wit_c_sdk.h>
#include <stdint.h>
#include <math.h>
#undef TEMP
#include <Wire.h>
#include "MS5837.h"
#define ACC_UPDATE 0x01
/////////////////////
/// For Micro ROS ///
/////////////////////
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// Variables untuk sensor
float gyro_yaw_raw = 0.0;
float compass_yaw_raw = 0.0;
float gyro_rate = 0.0;
float dt = 0.01;
unsigned long last_time = 0;

#include <ModbusMaster.h>

#define DE_RE 20

ModbusMaster nodemod;

void preTransmission() {
  digitalWrite(DE_RE, HIGH);
  delayMicroseconds(50);
}

void postTransmission() {
  delayMicroseconds(50);
  digitalWrite(DE_RE, LOW);
}

// Message headers
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <rosidl_runtime_c/string_functions.h>
#include <auv_interfaces/msg/multi_pid.h>
#include <auv_interfaces/msg/set_point.h>
#include <auv_interfaces/msg/actuator.h>
#include <auv_interfaces/msg/error.h>
#include <auv_interfaces/msg/sensor.h>
#include <auv_interfaces/msg/object_difference.h>

/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Declare rcl object
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Declare Publishers
rcl_publisher_t pub_pwm, pub_error, pub_sensor, pub_set_point, pub_pid, pub_status, pub_boost;

// Declare Subscribers
rcl_subscription_t sub_status, sub_boost, sub_pid, sub_set_point, sub_object_difference;

// Declare Messages
auv_interfaces__msg__Actuator pwm_msg;
auv_interfaces__msg__SetPoint set_point_msg;
auv_interfaces__msg__MultiPID pid_msg;
auv_interfaces__msg__Error error_msg;
auv_interfaces__msg__ObjectDifference object_difference_msg;
auv_interfaces__msg__Sensor sensor_msg;
std_msgs__msg__String status_msg;
std_msgs__msg__Float32 boost_msg;

// Sensor and control variables
float yaw = 0.0, last_yaw = 0.0, delta_yaw = 0.0;
float pitch = 0.0, last_pitch = 0.0, delta_pitch = 0.0;
float roll = 0.0, last_roll = 0.0, delta_roll = 0.0;
float depth = 0.0, last_depth = 0.0, delta_depth = 0.0;
float set_point_yaw = 0, set_point_pitch = 0, set_point_roll = 0, set_point_depth = 0;
float error_yaw = 0, error_pitch = 0, error_roll = 0, error_depth = 0;
bool is_stable_roll = true, is_stable_pitch = true, is_stable_yaw = true, is_stable_depth = true;
float thrust_dpr[4], thrust_ssy[4];
float t_yaw = 0, camera_yaw = 0;
int yawIndex = 0;
float min_pwm = 1000.0, max_pwm = 2000.0;
float constrain_boost = 150.0;
float pwm_thruster[10] = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0};

// PID coefficients (diterima dari ROS publisher)
float kp_yaw = 0, ki_yaw = 0, kd_yaw = 0;
float kp_pitch = 0, ki_pitch = 0, kd_pitch = 0;
float kp_roll = 0, ki_roll = 0, kd_roll = 0;
float kp_depth = 0, ki_depth = 0, kd_depth = 0;
float kp_camera = 0, ki_camera = 0, kd_camera = 0;

String class_name = "None";
int camera_error = 0;
String status = "stop";

byte pin_thruster[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
Servo thruster[10];
MS5837 sensor;

// Sensor communication variables
static volatile char s_cDataUpdate = 0;
const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
float sqrt2 = sqrt(2);

// ============================================================
// =================== PID Controller Class ====================
// ============================================================
// PERBAIKAN: Class PID yang PERSISTENT (tidak dibuat ulang tiap loop)
class PID {
  public:
    float kp, ki, kd;
    float integral;
    float last_error;
    float output_min, output_max;  // Anti-windup limits

    PID() : kp(0), ki(0), kd(0), integral(0), last_error(0),
            output_min(-1e6), output_max(1e6) {}

    PID(float _kp, float _ki, float _kd)
      : kp(_kp), ki(_ki), kd(_kd), integral(0), last_error(0),
        output_min(-1e6), output_max(1e6) {}

    void setGains(float _kp, float _ki, float _kd) {
      kp = _kp; ki = _ki; kd = _kd;
    }

    void setLimits(float _min, float _max) {
      output_min = _min; output_max = _max;
    }

    void reset() {
      integral = 0;
      last_error = 0;
    }

    float calculate(float error, float dt_sec) {
      // Proportional
      float proportional = kp * error;

      // Integral dengan anti-windup
      integral += ki * error * dt_sec;

      // Derivative
      float derivative = 0;
      if (dt_sec > 0) {
        derivative = kd * (error - last_error) / dt_sec;
      }
      last_error = error;

      float output = proportional + integral + derivative;

      // Anti-windup: clamp output dan stop integral accumulation
      if (output > output_max) {
        output = output_max;
        integral -= ki * error * dt_sec;  // Undo integral
      } else if (output < output_min) {
        output = output_min;
        integral -= ki * error * dt_sec;  // Undo integral
      }

      return output;
    }
};

// ============================================================
// ============= TRAPEZOIDAL RATE PROFILE PLANNER =============
// ============================================================
// Ini yang dosen Anda sarankan! Menghasilkan r_ref(t) yang halus.
class TrapezoidalPlanner {
  public:
    // === PARAMETER TUNING ===
    // r_max: kecepatan putar maksimal (deg/s) - mulai kecil, naikkan pelan-pelan
    // a_max: akselerasi putar maksimal (deg/s²) - menentukan seberapa cepat percepatan
    float r_max;       // Max yaw rate (deg/s)
    float a_max;       // Max yaw acceleration (deg/s²)

    // State internal
    float t_a;         // Waktu akselerasi (s)
    float t_c;         // Waktu cruise (s)
    float t_total;     // Waktu total manuver (s)
    float r_peak;      // Rate puncak yang dicapai (deg/s)
    float direction;   // +1 atau -1
    float delta_psi;   // Total rotasi yang diminta (deg, absolut)

    bool active;       // Apakah planner sedang berjalan
    float t_elapsed;   // Waktu yang sudah berlalu sejak mulai (s)

    // Untuk tracking posisi (heading reference)
    float heading_start;  // Heading saat planner dimulai
    float heading_target; // Heading target

    TrapezoidalPlanner()
      : r_max(30.0), a_max(15.0),  // <<< DEFAULT: tune ini!
        t_a(0), t_c(0), t_total(0), r_peak(0),
        direction(1), delta_psi(0), active(false), t_elapsed(0),
        heading_start(0), heading_target(0) {}

    // Panggil ini untuk memulai manuver baru
    void startManeuver(float current_heading, float target_heading) {
      heading_start = current_heading;
      heading_target = target_heading;

      // Hitung error heading dengan wrap -180..+180
      float delta = wrapTo180(target_heading - current_heading);
      direction = (delta >= 0) ? 1.0f : -1.0f;
      delta_psi = fabs(delta);

      // Jika error sangat kecil, tidak perlu manuver
      if (delta_psi < 0.5f) {  // kurang dari 0.5 derajat
        active = false;
        return;
      }

      // Hitung profil
      float delta_psi_a = (r_max * r_max) / (2.0f * a_max);

      if (delta_psi > 2.0f * delta_psi_a) {
        // === TRAPEZOID PENUH (ada fase cruise) ===
        r_peak = r_max;
        t_a = r_max / a_max;
        t_c = (delta_psi - 2.0f * delta_psi_a) / r_max;
      } else {
        // === SEGITIGA (rotasi kecil, tidak sempat cruise) ===
        r_peak = sqrtf(a_max * delta_psi);
        t_a = r_peak / a_max;
        t_c = 0;
      }
      t_total = 2.0f * t_a + t_c;

      t_elapsed = 0;
      active = true;

      Serial.println("=== TRAPEZOIDAL PLANNER STARTED ===");
      Serial.print("  Delta heading: "); Serial.print(delta); Serial.println(" deg");
      Serial.print("  r_peak: "); Serial.print(r_peak); Serial.println(" deg/s");
      Serial.print("  t_accel: "); Serial.print(t_a); Serial.println(" s");
      Serial.print("  t_cruise: "); Serial.print(t_c); Serial.println(" s");
      Serial.print("  t_total: "); Serial.print(t_total); Serial.println(" s");
    }

    // Panggil ini setiap loop untuk mendapatkan rate reference
    // Returns: yaw rate reference dalam deg/s
    float update(float dt_sec) {
      if (!active) return 0.0f;

      t_elapsed += dt_sec;
      float r_ref = 0.0f;

      if (t_elapsed < t_a) {
        // Fase 1: AKSELERASI (rate naik linear)
        r_ref = a_max * t_elapsed;
      }
      else if (t_elapsed < t_a + t_c) {
        // Fase 2: CRUISE (rate konstan di r_peak)
        r_ref = r_peak;
      }
      else if (t_elapsed < t_total) {
        // Fase 3: DESELERASI (rate turun linear)
        r_ref = r_peak - a_max * (t_elapsed - t_a - t_c);
        if (r_ref < 0) r_ref = 0;
      }
      else {
        // Fase 4: SELESAI
        r_ref = 0;
        active = false;
        Serial.println("=== TRAPEZOIDAL PLANNER FINISHED ===");
      }

      return r_ref * direction;  // Terapkan arah (+/-)
    }

    bool isActive() { return active; }

    // Hitung heading reference (integral dari rate) - untuk monitoring
    float getHeadingRef() {
      if (!active) return heading_target;

      float psi_ref = heading_start;
      float t = t_elapsed;

      if (t <= t_a) {
        // Sudut = 0.5 * a_max * t²
        psi_ref += direction * 0.5f * a_max * t * t;
      }
      else if (t <= t_a + t_c) {
        // Sudut saat akselerasi + sudut saat cruise
        float psi_accel = 0.5f * a_max * t_a * t_a;
        float psi_cruise = r_peak * (t - t_a);
        psi_ref += direction * (psi_accel + psi_cruise);
      }
      else if (t <= t_total) {
        float t_decel = t - t_a - t_c;
        float psi_accel = 0.5f * a_max * t_a * t_a;
        float psi_cruise = r_peak * t_c;
        float psi_decel = r_peak * t_decel - 0.5f * a_max * t_decel * t_decel;
        psi_ref += direction * (psi_accel + psi_cruise + psi_decel);
      }
      else {
        psi_ref = heading_target;
      }

      return psi_ref;
    }

    // ===================================================
    // INI YANG DIMAKSUD DOSEN! (garis MERAH di diagram)
    // ===================================================
    // Mengembalikan akselerasi sudut saat ini (deg/s²)
    // Positif = percepat, Nol = cruise, NEGATIF = NGEREM!
    // Ini dipakai sebagai FEEDFORWARD ke thruster.
    float getAcceleration() {
      if (!active) return 0.0f;

      if (t_elapsed < t_a) {
        return a_max * direction;           // Fase 1: PERCEPAT (merah positif)
      }
      else if (t_elapsed < t_a + t_c) {
        return 0.0f;                        // Fase 2: CRUISE (merah nol)
      }
      else if (t_elapsed < t_total) {
        return -a_max * direction;          // Fase 3: NGEREM! (merah negatif)
      }
      return 0.0f;                          // Selesai
    }

  private:
    float wrapTo180(float angle) {
      // BUG FIX: Pakai fmod agar aman dari infinite loop jika angle sangat besar
      angle = fmod(angle, 360.0f);
      if (angle > 180.0f)  angle -= 360.0f;
      if (angle < -180.0f) angle += 360.0f;
      return angle;
    }
};

// ============================================================
// V4: readYawRate() DIHAPUS
// V4 tidak butuh yaw rate sama sekali.
// Pengereman ditangani oleh feedforward dari acceleration profile.
// ============================================================

// ============================================================
// =================== GLOBAL INSTANCES ========================
// ============================================================

// PERBAIKAN: PID instances dibuat GLOBAL, bukan di dalam loop!
PID pid_ctrl_pitch;
PID pid_ctrl_roll;
PID pid_ctrl_depth;
PID pid_ctrl_camera;

// Yaw sekarang pakai: Trapezoidal Planner + Feedforward + P correction
TrapezoidalPlanner yaw_planner;

// V4: pid_rate_yaw TIDAK DIPAKAI (rate loop dihapus)
PID pid_rate_yaw;  // Tetap ada supaya compile tidak error

// V4: kp_yaw_outer TIDAK DIPAKAI
// Sekarang:
//   kp_yaw dari publisher = P gain koreksi heading (kecil, 0.03-0.05)
//   kd_yaw dari publisher = Kff feedforward gain (0.1-0.3)

// Track apakah setpoint berubah
float prev_set_point_yaw = -999;

// Smoothing filter states (untuk S-curve effect)
float smoothed_accel = 0.0f;   // Filtered acceleration
float smoothed_output = 0.0f;  // Filtered final output

// ============================================================
// =================== SSY & DPR Controllers ===================
// ============================================================

class SSYController {
  public:
    float d;
    SSYController(float distance) { d = distance; }

    void control(float Fx, float Fy, float tau, float thrust[4]) {
      float M[4][4] = {
        { -0.5f / sqrt2, 0.5f / sqrt2, d / 4.0f, 0.25f },
        { 0.5f / sqrt2, 0.5f / sqrt2, -d / 4.0f, -0.25f },
        { 0.5f / sqrt2, 0.5f / sqrt2, -d / 4.0f, 0.25f },
        { -0.5f / sqrt2, 0.5f / sqrt2, d / 4.0f, -0.25f }
      };

      float F[4] = { Fx, Fy, 0, tau };
      for (int i = 0; i < 4; i++) {
        thrust[i] = 0;
        for (int j = 0; j < 4; j++) {
          thrust[i] += M[i][j] * F[j];
        }
      }
    }
};

class DPRController {
  public:
    float Lx, Ly;
    DPRController(float lx, float ly) : Lx(lx), Ly(ly) {}

    void control(float control_depth, float control_pitch, float control_roll, float thrust[4]) {
      float A[3][4] = {
        { Ly, -Ly, Ly, -Ly },
        { Lx, Lx, -Lx, -Lx },
        { 1, 1, 1, 1 }
      };
      float desired_control[3] = { control_depth, control_pitch, control_roll };
      for (int i = 0; i < 4; i++) {
        thrust[i] = 0;
        for (int j = 0; j < 3; j++) {
          thrust[i] += A[j][i] * desired_control[j];
        }
      }
    }
};

SSYController ssyController(1.0);
DPRController dprController(0.5, 0.5);

// ============================================================
// =================== Helper Functions ========================
// ============================================================

float calculate_heading_error(float current, float target) {
  target = fmod(target, 360);
  float error = target - current;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  return error;
}

bool generate_is_stable(float thresh, float error) {
  return (-thresh <= error) && (error <= thresh);
}

// ============================================================
// =================== Sensor Communication ====================
// ============================================================

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial6.write(p_data, uiSize);
  Serial6.flush();
}

static void Delayms(uint16_t ucMs) { delay(ucMs); }

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  for (uint32_t i = 0; i < uiRegNum; i++) {
    if (uiReg == AZ) { s_cDataUpdate |= ACC_UPDATE; }
    uiReg++;
  }
}

static void AutoScanSensor(void) {
  int iRetry;
  for (size_t i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    Serial6.begin(c_uiBaud[i]);
    Serial6.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial6.available()) { WitSerialDataIn(Serial6.read()); }
      if (s_cDataUpdate != 0) {
        Serial.print(c_uiBaud[i]);
        Serial.print(" baud find sensor\r\n\r\n");
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  Serial.print("Cannot find sensor\r\n");
}

// ============================================================
// =================== ROS Callbacks ===========================
// ============================================================

bool receive_status = false;
void status_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String received_status = String(msg->data.data);
  status = received_status;
  receive_status = true;
}

bool receive_boost = false;
void boost_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  constrain_boost = msg->data;
  receive_boost = true;
}

bool receive_set_point = false;
void set_point_callback(const void *msgin) {
  const auv_interfaces__msg__SetPoint *msg = (const auv_interfaces__msg__SetPoint *)msgin;
  set_point_yaw = msg->yaw;
  set_point_pitch = msg->pitch;
  set_point_roll = msg->roll;
  set_point_depth = msg->depth;
  receive_set_point = true;
}

bool recieve_pid = false;
void pid_callback(const void *msgin) {
  const auv_interfaces__msg__MultiPID *msg = (const auv_interfaces__msg__MultiPID *)msgin;

  kp_yaw = msg->pid_yaw.kp;   ki_yaw = msg->pid_yaw.ki;   kd_yaw = msg->pid_yaw.kd;
  kp_pitch = msg->pid_pitch.kp; ki_pitch = msg->pid_pitch.ki; kd_pitch = msg->pid_pitch.kd;
  kp_roll = msg->pid_roll.kp;  ki_roll = msg->pid_roll.ki;  kd_roll = msg->pid_roll.kd;
  kp_depth = msg->pid_depth.kp; ki_depth = msg->pid_depth.ki; kd_depth = msg->pid_depth.kd;
  kp_camera = msg->pid_camera.kp; ki_camera = msg->pid_camera.ki; kd_camera = msg->pid_camera.kd;

  // Update PID gains untuk pitch, roll, depth
  pid_ctrl_pitch.setGains(kp_pitch, ki_pitch, kd_pitch);
  pid_ctrl_roll.setGains(kp_roll, ki_roll, kd_roll);
  pid_ctrl_depth.setGains(kp_depth, ki_depth, kd_depth);
  pid_ctrl_camera.setGains(kp_camera, ki_camera, kd_camera);

  // V4: kp_yaw = P gain heading koreksi (kecil)
  //     kd_yaw = Feedforward gain (repurposed, ini yang penting!)
  //     ki_yaw = tidak dipakai

  recieve_pid = true;
}

void object_difference_callback(const void *msgin) {
  const auv_interfaces__msg__ObjectDifference *msg = (const auv_interfaces__msg__ObjectDifference *)msgin;
  class_name = msg->object_type.data;
  camera_error = msg->x_difference;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    if(receive_status) {
      rcl_publish(&pub_status, &status_msg, NULL);
      receive_status = false;
    }
    if (receive_set_point) {
      set_point_msg.depth = set_point_depth;
      set_point_msg.roll = set_point_roll;
      set_point_msg.pitch = set_point_pitch;
      set_point_msg.yaw = set_point_yaw;
      rcl_publish(&pub_set_point, &set_point_msg, NULL);
      receive_set_point = false;
    }
    if (receive_boost) {
      boost_msg.data = constrain_boost;
      rcl_publish(&pub_boost, &boost_msg, NULL);
      receive_boost = false;
    }
    if (recieve_pid) {
      pid_msg.pid_yaw.kp = kp_yaw;     pid_msg.pid_yaw.ki = ki_yaw;     pid_msg.pid_yaw.kd = kd_yaw;
      pid_msg.pid_pitch.kp = kp_pitch;  pid_msg.pid_pitch.ki = ki_pitch;  pid_msg.pid_pitch.kd = kd_pitch;
      pid_msg.pid_roll.kp = kp_roll;    pid_msg.pid_roll.ki = ki_roll;    pid_msg.pid_roll.kd = kd_roll;
      pid_msg.pid_depth.kp = kp_depth;  pid_msg.pid_depth.ki = ki_depth;  pid_msg.pid_depth.kd = kd_depth;
      pid_msg.pid_camera.kp = kp_camera; pid_msg.pid_camera.ki = ki_camera; pid_msg.pid_camera.kd = kd_camera;
      rcl_publish(&pub_pid, &pid_msg, NULL);
      recieve_pid = false;
    }

    rcl_publish(&pub_pwm, &pwm_msg, NULL);
    rcl_publish(&pub_error, &error_msg, NULL);
    rcl_publish(&pub_sensor, &sensor_msg, NULL);
  }
}

// ============================================================
// =================== Micro-ROS Functions =====================
// ============================================================

bool create_entities() {
  const char * node_name = "teensy_node";
  const char * ns = "";
  const int domain_id = 0;

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  // Publishers
  rclc_publisher_init(&pub_pwm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Actuator), "actuator_pwm", &rmw_qos_profile_default);
  rclc_publisher_init(&pub_error, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Error), "error_msg", &rmw_qos_profile_default);
  rclc_publisher_init(&pub_sensor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Sensor), "sensor_msg", &rmw_qos_profile_default);
  rclc_publisher_init(&pub_set_point, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint), "set_point_msg", &rmw_qos_profile_default);
  rclc_publisher_init(&pub_pid, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID), "pid_msg", &rmw_qos_profile_default);
  rclc_publisher_init(&pub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status_msg", &rmw_qos_profile_default);
  rclc_publisher_init(&pub_boost, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "boost_msg", &rmw_qos_profile_default);

  // Subscribers
  rclc_subscription_init(&sub_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status", &rmw_qos_profile_default);
  rclc_subscription_init(&sub_boost, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "boost", &rmw_qos_profile_default);
  rclc_subscription_init(&sub_pid, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, MultiPID), "pid", &rmw_qos_profile_default);
  rclc_subscription_init(&sub_set_point, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, SetPoint), "set_point", &rmw_qos_profile_default);
  rclc_subscription_init(&sub_object_difference, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, ObjectDifference), "object_difference", &rmw_qos_profile_default);

  const unsigned int timer_timeout = 50;  // 20Hz
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  unsigned int num_handles = 6;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &sub_status, &status_msg, &status_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_boost, &boost_msg, &boost_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_pid, &pid_msg, &pid_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_set_point, &set_point_msg, &set_point_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_object_difference, &object_difference_msg, &object_difference_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  rcl_publisher_fini(&pub_pwm, &node);
  rcl_publisher_fini(&pub_error, &node);
  rcl_publisher_fini(&pub_sensor, &node);
  rcl_publisher_fini(&pub_set_point, &node);
  rcl_publisher_fini(&pub_pid, &node);
  rcl_publisher_fini(&pub_status, &node);
  rcl_publisher_fini(&pub_boost, &node);

  rcl_subscription_fini(&sub_status, &node);
  rcl_subscription_fini(&sub_boost, &node);
  rcl_subscription_fini(&sub_pid, &node);
  rcl_subscription_fini(&sub_set_point, &node);
  rcl_subscription_fini(&sub_object_difference, &node);
}

// ============================================================
// =================== SETUP ===================================
// ============================================================

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);

  Serial7.begin(9600, SERIAL_8N1);
  nodemod.begin(0x00, Serial7);
  nodemod.preTransmission(preTransmission);
  nodemod.postTransmission(postTransmission);

  Serial6.begin(115200);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();

  for (int i = 0; i < 10; i++) {
    if (!thruster[i].attach(pin_thruster[i])) {
      Serial.printf("Failed to attach thruster %d\n", i);
    }
    thruster[i].writeMicroseconds(1500);
  }

  Wire.begin();
  int attempts = 0;
  while (!sensor.init() && attempts < 5) {
    Serial.println("Init failed! Retrying...");
    delay(1000);
    attempts++;
  }
  if (attempts >= 5) {
    Serial.println("Failed to initialize depth sensor after 5 attempts");
  } else {
    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(997);
    Serial.println("Depth sensor initialized successfully");
  }

  // Initialize messages
  auv_interfaces__msg__Actuator__init(&pwm_msg);
  auv_interfaces__msg__SetPoint__init(&set_point_msg);
  auv_interfaces__msg__MultiPID__init(&pid_msg);
  auv_interfaces__msg__Error__init(&error_msg);
  auv_interfaces__msg__ObjectDifference__init(&object_difference_msg);
  auv_interfaces__msg__Sensor__init(&sensor_msg);
  std_msgs__msg__String__init(&status_msg);
  std_msgs__msg__Float32__init(&boost_msg);

  status_msg.data.data = malloc(50);
  status_msg.data.capacity = 50;
  status_msg.data.size = 0;

  // === Setup Trapezoidal Planner Parameters ===
  // Ini menentukan profil gerakan (3 garis di diagram dosen):
  yaw_planner.r_max = 10.0;   // Max 10 deg/s — kecepatan putar maks (garis hitam)
  yaw_planner.a_max = 5.0;    // Max 5 deg/s² — akselerasi maks (garis merah)

  // V4: Rate loop tidak dipakai. Feedforward dari acceleration profile.
  // kp_yaw = P gain koreksi heading (kecil, 0.03-0.05)
  // kd_yaw = Feedforward gain Kff (0.1-0.3)

  // Setup PID limits untuk axis lain
  pid_ctrl_depth.setLimits(-500, 500);
  pid_ctrl_pitch.setLimits(-500, 500);
  pid_ctrl_roll.setLimits(-500, 500);
  pid_ctrl_camera.setLimits(-3.0, 3.0);

  state = WAITING_AGENT;
  Serial.println("Setup completed - V4-SMOOTH: Feedforward + P + Smoothing (S-curve effect)");
  last_time = millis();
}

// ============================================================
// ================== MAIN CONTROL LOOP ========================
// ============================================================

void run_control_loop() {
  // 1. Hitung delta time
  unsigned long current_time = millis();
  dt = (current_time - last_time) / 1000.0f;
  if (dt <= 0) dt = 0.01f;  // Safety
  last_time = current_time;

  // 2. Simpan sensor sebelumnya
  last_depth = depth;
  last_roll = roll;
  last_pitch = pitch;
  last_yaw = yaw;

  // 3. Baca sensor YAW (heading) dari HWT3100 via Modbus
  uint8_t result = nodemod.readHoldingRegisters(0xDE, 1);
  if (result == nodemod.ku8MBSuccess) {
    int16_t raw = nodemod.getResponseBuffer(0);
    yaw = raw / 10.0f;
    yaw = -yaw;
    if (yaw < 0)    yaw += 360.0f;
    if (yaw >= 360.0) yaw -= 360.0f;
  } else {
    Serial.print("Modbus Error: 0x");
    Serial.println(result, HEX);
  }

  // 4. Baca PITCH & ROLL dari HWT905 via Serial6
  while (Serial6.available()) {
    WitSerialDataIn(Serial6.read());
  }
  if (s_cDataUpdate & ACC_UPDATE) {
    pitch = sReg[AY] / 32768.0f * 16.0f;
    roll = sReg[AX] / 32768.0f * 16.0f;
    s_cDataUpdate &= ~ACC_UPDATE;
  }

  // 5. Baca DEPTH
  sensor.read();
  depth = sensor.depth();

  // 6. Filter sensor data
  delta_depth = abs(last_depth - depth);
  delta_roll = abs(last_roll - roll);
  delta_pitch = abs(last_pitch - pitch);
  delta_yaw = abs(last_yaw - yaw);

  if (abs(depth) > 10) depth = last_depth;
  if (delta_depth > 1) depth = last_depth;
  if (delta_roll > 0.4) roll = last_roll;
  if (delta_pitch > 0.3) pitch = last_pitch;

  // 7. V4: Yaw rate TIDAK DIPAKAI (pengereman dari feedforward)

  // 8. Hitung errors untuk pitch, roll, depth (sama seperti sebelumnya)
  error_pitch = set_point_pitch - pitch;
  error_roll = set_point_roll - roll;
  error_depth = set_point_depth - depth;

  // Heading error (untuk monitoring, bukan untuk kontrol langsung)
  error_yaw = calculate_heading_error(yaw, set_point_yaw);

  // 9. Stability check untuk pitch, roll, depth
  is_stable_roll = generate_is_stable(0, error_roll);
  is_stable_pitch = generate_is_stable(0, error_pitch);
  is_stable_depth = generate_is_stable(0.05, error_depth);

  error_roll = is_stable_roll ? 0 : error_roll;
  error_pitch = is_stable_pitch ? 0 : error_pitch;
  error_depth = is_stable_depth ? 0 : error_depth;

  // ============================================================
  // 10. YAW CONTROL V4-SMOOTH: FEEDFORWARD + P + SMOOTHING
  // ============================================================
  //
  // Upgrade dari V4: tambah SMOOTHING supaya gerakan lebih halus.
  //
  // Trapezoidal punya akselerasi yang LONCAT (kotak):
  //    ┌──────┐
  //    │      │
  // ───┘      └───
  //
  // Dengan smoothing filter, jadi seperti S-curve (halus):
  //       ╭──────╮
  //      ╱        ╲
  // ────╯          ╰───
  //
  // Hasilnya: thruster naik gradual, tidak "nendang"
  //

  // Deteksi setpoint baru → mulai trapezoidal planner
  if (fabs(set_point_yaw - prev_set_point_yaw) > 0.5f) {
    yaw_planner.startManeuver(yaw, set_point_yaw);
    prev_set_point_yaw = set_point_yaw;
    smoothed_accel = 0.0f;  // Reset filter saat manuver baru
    smoothed_output = 0.0f;
  }

  // Update planner (advance waktu)
  yaw_planner.update(dt);

  // Dapatkan sinyal dari planner
  float heading_ref = yaw_planner.getHeadingRef();   // Garis BIRU
  float accel_ref   = yaw_planner.getAcceleration();  // Garis MERAH (kotak/tajam)

  // === SMOOTHING FILTER pada akselerasi ===
  // Ini mengubah akselerasi dari bentuk kotak (tajam) jadi bentuk S-curve (halus)
  // alpha_accel kecil = lebih smooth, tapi lebih lambat merespons
  float alpha_accel = 0.15f;  // 0.1 = sangat smooth, 0.3 = agak smooth
  smoothed_accel = alpha_accel * accel_ref + (1.0f - alpha_accel) * smoothed_accel;

  // === FEEDFORWARD dari akselerasi yang sudah di-smooth ===
  float ff_term = kd_yaw * smoothed_accel;

  // === P CORRECTION (koreksi error heading) ===
  float heading_err = calculate_heading_error(yaw, heading_ref);

  // Deadband: error sangat kecil → tidak perlu koreksi
  if (fabs(heading_err) < 1.0f) {
    heading_err = 0.0f;
  }

  float p_term = kp_yaw * heading_err;

  // === GABUNG: Feedforward + P correction ===
  float raw_output = ff_term + p_term;

  // === SMOOTHING FILTER pada output akhir ===
  // Mencegah t_yaw berubah terlalu cepat → thruster bergerak gradual
  float alpha_output = 0.2f;  // 0.1 = sangat smooth, 0.3 = agak smooth
  smoothed_output = alpha_output * raw_output + (1.0f - alpha_output) * smoothed_output;

  t_yaw = smoothed_output;

  // Clamp output ke range SSYController
  if (t_yaw > 3.0f) t_yaw = 3.0f;
  if (t_yaw < -3.0f) t_yaw = -3.0f;

  // Camera yaw tetap pakai PID biasa
  camera_yaw = pid_ctrl_camera.calculate((float)camera_error, dt);
  camera_yaw = constrain(camera_yaw, -3.0f, 3.0f);

  // ============================================================
  // 11. CONTROL LOGIC BERDASARKAN STATUS
  // ============================================================
  // t_yaw sekarang sudah dari trapezoidal+rate loop, tinggal pakai!

  if (status == "stop") {
    ssyController.control(0, 0, 0, thrust_ssy);
    dprController.control(0, 0, 0, thrust_dpr);  // BUG FIX: langsung 0, jangan panggil PID
    // Reset semua PID dan planner saat stop
    pid_ctrl_depth.reset();
    pid_ctrl_pitch.reset();
    pid_ctrl_roll.reset();
    yaw_planner.active = false;
    prev_set_point_yaw = -999;
    smoothed_accel = 0.0f;
    smoothed_output = 0.0f;
  }
  else if (status == "all") {
    ssyController.control(0, 0.5, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "all_boost") {
    ssyController.control(0, 2, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "backward") {
    ssyController.control(0, -2, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "all_slow") {
    ssyController.control(0, 1, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "last") {
    ssyController.control(0, 2, 0, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "last_slow") {
    ssyController.control(0, 1, 0, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "pitch") {
    ssyController.control(0, 0, 0, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(0, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(0, dt), thrust_dpr);
  }
  else if (status == "roll") {
    ssyController.control(0, 0, 0, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(0, dt), pid_ctrl_pitch.calculate(0, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "yaw") {
    ssyController.control(0, 0, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(0, dt), pid_ctrl_pitch.calculate(0, dt), pid_ctrl_roll.calculate(0, dt), thrust_dpr);
  }
  else if (status == "depth") {
    ssyController.control(0, 0, 0, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(0, dt), pid_ctrl_roll.calculate(0, dt), thrust_dpr);
  }
  else if (status == "dpr") {
    ssyController.control(0, 0, 0, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "dpr_ssy") {
    ssyController.control(0, 0, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "camera") {
    ssyController.control(0, 1, camera_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "camera_sway") {
    ssyController.control((camera_yaw * 0.5), 0, camera_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "camera_yaw") {
    ssyController.control(0, 0, camera_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "camera_sway_forward") {
    ssyController.control(-(camera_yaw * 0.5), 1, (t_yaw * 0.5), thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "sway_right_forward") {
    ssyController.control(-3, 1, (t_yaw + 2), thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "sway_left_forward") {
    ssyController.control(3, 2, (t_yaw - 2), thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "sway_right") {
    ssyController.control(-3, 0, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "sway_left") {
    ssyController.control(3, 0.4, t_yaw, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "yaw_right") {
    ssyController.control(0, 0, -0.3, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }
  else if (status == "yaw_left") {
    ssyController.control(0, 0, 0.3, thrust_ssy);
    dprController.control(pid_ctrl_depth.calculate(error_depth, dt), pid_ctrl_pitch.calculate(error_pitch, dt), pid_ctrl_roll.calculate(-error_roll, dt), thrust_dpr);
  }

  // ============================================================
  // 12. CALCULATE PWM & APPLY
  // ============================================================

  pwm_thruster[0] = constrain(1500 - (thrust_ssy[1] * 500.0), min_pwm, max_pwm);
  pwm_thruster[1] = constrain(1500 - (thrust_ssy[0] * 500.0), min_pwm, max_pwm);
  pwm_thruster[2] = constrain(1500 - (thrust_ssy[3] * 500.0), min_pwm, max_pwm);
  pwm_thruster[3] = constrain(1500 - (thrust_ssy[2] * 500.0), min_pwm, max_pwm);
  pwm_thruster[4] = constrain(1500 - thrust_dpr[2], min_pwm, max_pwm);
  pwm_thruster[5] = constrain(1500 - thrust_dpr[1], min_pwm, max_pwm);
  pwm_thruster[6] = constrain(1500 + thrust_dpr[0], min_pwm, max_pwm);
  pwm_thruster[7] = constrain(1500 + thrust_dpr[3], min_pwm, max_pwm);
  pwm_thruster[8] = constrain(1500 - (thrust_ssy[1] * constrain_boost), (1500.0 - constrain_boost), (1500.0 + constrain_boost));
  pwm_thruster[9] = constrain(1500 - (thrust_ssy[0] * constrain_boost), (1500.0 - constrain_boost), (1500.0 + constrain_boost));

  if (status == "all_boost") {
    pwm_thruster[0] = 1500.0;
    pwm_thruster[1] = 1500.0;
    pwm_thruster[2] = 1500.0;
    pwm_thruster[3] = 1500.0;
  }

  for (int i = 0; i < 10; i++) {
    thruster[i].writeMicroseconds(pwm_thruster[i]);
  }

  // ============================================================
  // 13. DEBUG OUTPUT
  // ============================================================

  Serial.println("=== CONTROL LOOP V4-SMOOTH ===");
  Serial.print("dt = "); Serial.print(dt * 1000, 1); Serial.println(" ms");
  Serial.print("yaw = "); Serial.print(yaw, 1);
  Serial.print(" | sp = "); Serial.print(set_point_yaw, 1);
  Serial.print(" | err = "); Serial.println(error_yaw, 1);
  Serial.print("heading_ref = "); Serial.print(heading_ref, 1);
  Serial.print(" | accel_raw = "); Serial.print(accel_ref, 2);
  Serial.print(" | accel_smooth = "); Serial.println(smoothed_accel, 2);
  Serial.print("FF = "); Serial.print(ff_term, 3);
  Serial.print(" | P = "); Serial.print(p_term, 3);
  Serial.print(" | t_yaw = "); Serial.println(t_yaw, 3);
  Serial.print("planner active = "); Serial.println(yaw_planner.isActive() ? "YES" : "NO");
  Serial.print("pitch = "); Serial.print(pitch, 2);
  Serial.print(" | roll = "); Serial.print(roll, 2);
  Serial.print(" | depth = "); Serial.println(depth, 3);

  for (int i = 0; i < 10; i++) {
    Serial.print("T"); Serial.print(i+1); Serial.print("=");
    Serial.print(pwm_thruster[i], 0); Serial.print(" ");
  }
  Serial.println();

  // Update messages
  pwm_msg.thruster_1 = pwm_thruster[0];
  pwm_msg.thruster_2 = pwm_thruster[1];
  pwm_msg.thruster_3 = pwm_thruster[2];
  pwm_msg.thruster_4 = pwm_thruster[3];
  pwm_msg.thruster_5 = pwm_thruster[4];
  pwm_msg.thruster_6 = pwm_thruster[5];
  pwm_msg.thruster_7 = pwm_thruster[6];
  pwm_msg.thruster_8 = pwm_thruster[7];
  pwm_msg.thruster_9 = pwm_thruster[8];
  pwm_msg.thruster_10 = pwm_thruster[9];

  sensor_msg.depth = depth;
  sensor_msg.roll = roll;
  sensor_msg.pitch = pitch;
  sensor_msg.yaw = yaw;

  error_msg.depth = error_depth;
  error_msg.roll = error_roll;
  error_msg.pitch = error_pitch;
  error_msg.yaw = error_yaw;
  error_msg.camera = float(camera_error);

  delay(5);
}

// ============================================================
// =================== MAIN LOOP ===============================
// ============================================================

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) { destroy_entities(); }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        Serial.println("Executor running...");
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        run_control_loop();
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