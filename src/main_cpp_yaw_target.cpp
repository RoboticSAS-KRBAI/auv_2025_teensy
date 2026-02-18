#include <Servo.h>
#include <Arduino.h>
#include <ModbusMaster.h>

/* ================= CONFIG ================= */
#define NUM_THRUSTERS 8
const int thruster_pin[NUM_THRUSTERS] = {0,1,2,3,4,5,6,7};

// ===== DEAD BAND DATA (HASIL UKUR KAMU) =====
const int PWM_REVERSE_START = 1476;  // mulai gerak kiri
const int PWM_FORWARD_START = 1520;  // mulai gerak kanan
// const int PWM_DB_CENTER = (PWM_REVERSE_START + PWM_FORWARD_START) / 2; // 1497

const int PWM_NEUTRAL = 1500;
const int PWM_MIN = 1250;
const int PWM_MAX = 1750;
const float LOOP_DT = 0.01; // 100 Hz

#define DE_RE 20  // RS485 direction pin

/* ================= GLOBALS ================= */
Servo thruster[NUM_THRUSTERS];
ModbusMaster nodemod;

// PID
float kp = 0.0, ki = 0.0, kd = 0.0;
float setpoint = 0.0;
float measured_velocity = 0.0;

float integral = 0.0;
float last_error = 0.0;

// Thruster PWM values
int thruster_pwm[NUM_THRUSTERS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Compass
float yaw_current = 0.0;
float yaw_previous = 0.0;
float yaw_rate = 0.0;

/* ===== MOVING AVERAGE ===== */
#define YAW_MA_WINDOW 50
float yaw_rate_buffer[YAW_MA_WINDOW] = {0};
int yaw_rate_idx = 0;
bool yaw_rate_full = false;
float yaw_rate_ma = 0.0;
/* =========================== */

unsigned long last_time = 0;
unsigned long last_compass_time = 0;
unsigned long last_print_time = 0;

const unsigned long COMPASS_INTERVAL = 10;
const unsigned long PRINT_INTERVAL = 50;   // 20 Hz print (AMAN)

/* ================= RS485 ================= */
void preTransmission() {
  digitalWrite(DE_RE, HIGH);
  delayMicroseconds(10);
}

void postTransmission() {
  delayMicroseconds(10);
  digitalWrite(DE_RE, LOW);
}

/* ============== ANGLE WRAP ============== */
float angleDifference(float current, float previous) {
  float diff = current - previous;
  if (diff > 180.0) diff -= 360.0;
  else if (diff < -180.0) diff += 360.0;
  return diff;
}

/* =========== MOVING AVERAGE ============ */
float updateYawRateMA(float new_rate)
{
  yaw_rate_buffer[yaw_rate_idx] = new_rate;
  yaw_rate_idx++;

  if (yaw_rate_idx >= YAW_MA_WINDOW) {
    yaw_rate_idx = 0;
    yaw_rate_full = true;
  }

  int count = yaw_rate_full ? YAW_MA_WINDOW : yaw_rate_idx;
  float sum = 0.0;

  for (int i = 0; i < count; i++) {
    sum += yaw_rate_buffer[i];
  }

  return sum / count;
}

/* =========== UPDATE COMPASS ============ */
void updateCompass() {
  uint8_t result = nodemod.readHoldingRegisters(0xDE, 1);

  if (result == nodemod.ku8MBSuccess) {
    int16_t raw = nodemod.getResponseBuffer(0);

    yaw_previous = yaw_current;

    yaw_current = raw / 10.0f;
    yaw_current = -yaw_current;

    if (yaw_current < 0) yaw_current += 360.0f;
    if (yaw_current >= 360.0) yaw_current -= 360.0f;

    unsigned long now = millis();
    float dt = (now - last_compass_time) / 1000.0;

    if (dt > 0.001 && last_compass_time > 0) {
      float delta_yaw = angleDifference(yaw_current, yaw_previous);
      yaw_rate = delta_yaw / dt;

      if (abs(yaw_rate) > 500.0) {
        yaw_rate = measured_velocity;
      } else {
        yaw_rate_ma = updateYawRateMA(yaw_rate);

        const float alpha = 0.3;
        measured_velocity = alpha * yaw_rate_ma
                          + (1.0 - alpha) * measured_velocity;
      }
    }

    last_compass_time = now;
  }
}

/* ================= PID ================= */
struct PIDOutput {
  float p;
  float i;
  float d;
  float total;
};

float err = 0.0;

PIDOutput pid_update(float sp, float pv, int pwm_out)
{
//   err = sp - pv;
  err = angleDifference(sp, pv);

  if (pwm_out > PWM_MIN && pwm_out < PWM_MAX) {
    integral += err * LOOP_DT;
  }
  integral = constrain(integral, -10, 10);

  float derivative = (err - last_error) / LOOP_DT;
  last_error = err;

  PIDOutput output;
  output.p = kp * err;
  output.i = ki * integral;
  output.d = kd * derivative;
  output.total = output.p + output.i + output.d;

  return output;
}

/* =============== THRUSTER =============== */
void applyYaw(int pwm) {
  thruster[0].writeMicroseconds(pwm);
  thruster[2].writeMicroseconds(pwm);
  thruster[1].writeMicroseconds(3000 - pwm);
  thruster[3].writeMicroseconds(3000 - pwm);

  // Store PWM values for sending
  thruster_pwm[0] = pwm;
  thruster_pwm[1] = 3000 - pwm;
  thruster_pwm[2] = pwm;
  thruster_pwm[3] = 3000 - pwm;


  for (int i = 4; i < 8; i++) {
    thruster[i].writeMicroseconds(PWM_NEUTRAL);
    thruster_pwm[i] = PWM_NEUTRAL;
  }
}

/* ============ SERIAL PARSER ============ */
void parseSerial()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "stop") {
    setpoint = yaw_current;
    kp = 0.0;
    ki = 0.0;
    kd = 0.0;
    integral = 0.0;
    last_error = 0.0;

    for (int i = 0; i < NUM_THRUSTERS; i++)
      thruster[i].writeMicroseconds(PWM_NEUTRAL);

    Serial.println("STOP_OK");
    return;
  }

  if (cmd.startsWith("sp ")) {
    setpoint = cmd.substring(3).toFloat();
    if (setpoint == 0.0) integral = 0.0;
  }
  else if (cmd.startsWith("kp ")) {
    kp = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("ki ")) {
    ki = cmd.substring(3).toFloat();
  }
  else if (cmd.startsWith("kd ")) {
    kd = cmd.substring(3).toFloat();
  }

  Serial.println("OK");
}

/* ================= SETUP ================= */
void setup()
{
  Serial.begin(115200);
  Serial7.begin(115200, SERIAL_8N1);
  delay(1000);

  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);

  nodemod.begin(0x50, Serial7);
  nodemod.preTransmission(preTransmission);
  nodemod.postTransmission(postTransmission);

  for (int i = 0; i < NUM_THRUSTERS; i++) {
    thruster[i].attach(thruster_pin[i]);
    thruster[i].writeMicroseconds(PWM_NEUTRAL);
  }

  last_time = millis();
  last_compass_time = millis();
  last_print_time = millis();
}

/* ================= LOOP ================= */
void loop()
{
  parseSerial();

  unsigned long now = millis();

  if ((now - last_compass_time) >= COMPASS_INTERVAL) {
    updateCompass();
  }

  if ((now - last_time) >= (LOOP_DT * 1000)) {
    last_time = now;

    PIDOutput pid = pid_update(setpoint, yaw_current, PWM_NEUTRAL);

    // int pwm = PWM_NEUTRAL + pid.total;
    // pwm = constrain(pwm, PWM_MIN, PWM_MAX);

    float control = pid.total;
    int pwm;

    // ===== DEAD BAND COMPENSATION =====
    if (control > 0) {  // kanan
      pwm = PWM_FORWARD_START + control;
    }
    else if (control < 0) {  // kiri
      pwm = PWM_REVERSE_START + control;
    }
    else {
      pwm = PWM_NEUTRAL;
    }

    // constrain akhir
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);

    applyYaw(pwm);

    if ((now - last_print_time) >= PRINT_INTERVAL) {
      last_print_time = now;


      // Send all data as JSON
      Serial.print("{");
      Serial.print("\"yaw\":");Serial.print(yaw_current, 1);Serial.print(",");
      Serial.print("\"velocity\":");Serial.print(measured_velocity, 2);Serial.print(",");
      Serial.print("\"setpoint\":");Serial.print(setpoint, 2);Serial.print(",");
      Serial.print("\"error\":");Serial.print(err, 2);Serial.print(",");
      Serial.print("\"p\":");Serial.print(kp, 3);Serial.print(",");
      Serial.print("\"i\":");Serial.print(ki, 4);Serial.print(",");
      Serial.print("\"d\":");Serial.print(kd, 4);Serial.print(",");
      Serial.print("\"total\":");Serial.print(pid.total, 3);Serial.print(",");
      Serial.print("\"pwm\":");Serial.print(pwm);Serial.print(",");
      Serial.print("\"thrusters\":[");
      for(int i = 0; i < 8; i++) {
        Serial.print(thruster_pwm[i]);
        if(i < 7) Serial.print(",");
      }
      Serial.println("]}");
    }
  }
}