/*
 * AUV SENSOR FUSION FINAL
 * - HWT905 (SDK Mode) on Serial6 (9600/115200)
 * - HWT3100 (Modbus) on Serial7
 * - Kalman Filter Integration
 */

#include <Arduino.h>
#include <REG.h>
#include <wit_c_sdk.h>
#include <ModbusMaster.h>

// ================= CONFIG =================
// HWT905 (TTL)
#define HWT905_SERIAL Serial6

// HWT3100 (RS485)
#define HWT3100_SERIAL Serial7
#define RS485_DE_PIN   20
#define RS485_BAUD     9600
#define MODBUS_RATE    100   // Baca kompas tiap 100ms (10Hz)

// ================= OBJEK & VAR =================
ModbusMaster node;

// Variabel Data
float gyroZ = 0.0f;
float compassYaw = 0.0f;
unsigned long lastGyroTime = 0;
unsigned long lastModbusTime = 0;

// Flags SDK WitMotion
static volatile char s_cDataUpdate = 0;

// ================= KALMAN FILTER =================
struct KalmanYaw {
  float x; float P; float Q; float R;
} kf;

float wrapAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void kalmanInit() {
  kf.x = 0.0f; kf.P = 1.0f;
  // Tuning parameters: 
  // Semakin kecil Q, semakin percayai gyro.
  // Semakin kecil R, semakin percayai kompas.
  kf.Q = 0.02f;  // Gyro trust    | scenario: 1) 0.02f (Balanced/Default) 2) 0.005f (Percaya Gyro)  3) 0.001f (Sangat Percaya Gyro)  4) 0.10f (Percaya Kompas)  5) 0.10f (Sangat Percaya Kompas)                                           
  kf.R = 1.00f;  // Compass trust  |              1.00f                       5.00f                     10.00f                           0.10f                      0.50f            
}

void kalmanPredict(float rate, float dt) {
  kf.x += rate * dt;
  kf.x = wrapAngle(kf.x);
  kf.P += kf.Q;
}

void kalmanUpdate(float meas) {
  float y = wrapAngle(meas - kf.x);
  float S = kf.P + kf.R;
  float K = kf.P / S;
  kf.x += K * y;
  kf.x = wrapAngle(kf.x);
  kf.P = (1.0f - K) * kf.P;
}

// ================= CALLBACKS WITMOTION SDK =================
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  HWT905_SERIAL.write(p_data, uiSize);
  HWT905_SERIAL.flush();
}

static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

// Callback ini dipanggil otomatis oleh SDK saat data valid masuk
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  for (int i = 0; i < uiRegNum; i++) {
    // Kita hanya peduli Gyro Z (GZ)
    if (uiReg == GZ) {
      s_cDataUpdate = 1; 
    }
    uiReg++;
  }
}

// AutoScan sederhana (dicopy dari kodemu, disederhanakan)
const uint32_t c_uiBaud[] = {9600, 115200}; // Cek 2 baudrate umum saja biar cepat
static void AutoScanSensor(void) {
    int i, iRetry;
    for(i = 0; i < 2; i++) {
        HWT905_SERIAL.begin(c_uiBaud[i]);
        HWT905_SERIAL.flush();
        iRetry = 2;
        s_cDataUpdate = 0;
        
        while(iRetry--) {
            WitReadReg(AX, 3);
            delay(100);
            while (HWT905_SERIAL.available()) {
                WitSerialDataIn(HWT905_SERIAL.read());
            }
            if(s_cDataUpdate != 0) {
                Serial.print("HWT905 Found at: ");
                Serial.println(c_uiBaud[i]);
                return;
            }
        }
    }
    Serial.println("HWT905TTL NOT FOUND!");
}

// ================= CALLBACKS RS485 =================
void preTransmission() { digitalWrite(RS485_DE_PIN, HIGH); }
void postTransmission() { delayMicroseconds(20); digitalWrite(RS485_DE_PIN, LOW); }


// ================= MAIN SETUP =================
void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 3000);
  
  Serial.println("--- FUSION START ---");

  // 1. Setup HWT905 (SDK Mode)
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  
  Serial.println("Scanning HWT905...");
  AutoScanSensor(); // Ini akan set baudrate Serial6 otomatis

  // 2. Setup HWT3100 (Modbus)
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  HWT3100_SERIAL.begin(RS485_BAUD);
  
  node.begin(0x00, HWT3100_SERIAL);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // 3. Init Kalman
  kalmanInit();
  lastGyroTime = millis();
}


// ================= MAIN LOOP =================
void loop() {
  unsigned long now = millis();

  // ------------------------------------------
  // TASK 1: PROCESS SDK HWT905 (Terus Menerus)
  // ------------------------------------------
  while (HWT905_SERIAL.available()) {
    // Masukkan data serial ke mesin SDK WitMotion
    WitSerialDataIn(HWT905_SERIAL.read());
  }

  // Jika SDK bilang "Ada Data Gyro Z Baru!" (via Callback tadi)
  if (s_cDataUpdate) {
    // 1. Hitung DT
    float dt = (now - lastGyroTime) / 1000.0f;
    lastGyroTime = now;

    // 2. Ambil Data Gyro Z dari variable global SDK (sReg)
    // Rumus: Raw / 32768 * 2000
    gyroZ = (float)sReg[GZ] / 32768.0f * 2000.0f;

    // 3. PREDIKSI KALMAN
    // Filter glitch DT (cegah lompatan saat startup)
    if (dt > 0.0001f && dt < 0.2f) {
       kalmanPredict(gyroZ, dt);
    }
    
    // Reset flag
    s_cDataUpdate = 0;
  }

  // ------------------------------------------
  // TASK 2: PROCESS MODBUS HWT3100 (Timer 10Hz)
  // ------------------------------------------
  if (now - lastModbusTime >= MODBUS_RATE) {
    lastModbusTime = now;

    // Baca Modbus (Blocking sebentar gpp, asal jarang-jarang)
    uint8_t result = node.readHoldingRegisters(0x00DB, 4);
    
    if (result == node.ku8MBSuccess) {
      // Ambil Yaw (Index 3 dari blok 0xDB)
      int16_t rawYaw = node.getResponseBuffer(3);
      compassYaw = (float)rawYaw / 10.0f;
      compassYaw = wrapAngle(compassYaw);

      // 4. UPDATE KALMAN (Koreksi dengan Kompas)
      kalmanUpdate(compassYaw);

      // 5. PRINT HASIL FUSION
      // Format: KF, Compass, Gyro (Buka Serial Plotter!)
      
      // --- UPDATE: PRINT TIMESTAMP (SECONDS) ---
      // Dibagi 1000.0 agar jadi detik desimal (float), ditampilkan 3 angka belakang koma
      Serial.print("Time:"); Serial.println(millis() / 1000.0, 3);
      Serial.println();
      Serial.print("KF  :"); Serial.println(kf.x, 2);
      Serial.print("Comp:"); Serial.println(compassYaw, 2);
      Serial.print("GZ  :"); Serial.println(gyroZ, 2);
      Serial.println("-----------");
    }
    else {
      Serial.print("HWT3100-485 | Modbus Error: 0x");
      Serial.println(result, HEX);
    }
  }
}