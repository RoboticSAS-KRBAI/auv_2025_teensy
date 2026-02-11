// #include <Arduino.h>

// #include <REG.h>
// #include <wit_c_sdk.h>

// #define ANGLE_UPDATE 0x04

// static volatile char s_cDataUpdate = 0;

// static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
// static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
// static void Delayms(uint16_t ucMs);

// float roll, pitch, yaw; // Variabel untuk menyimpan Roll, Pitch, dan Yaw

// static void AutoScanSensor(void) {
//   const uint32_t c_uiBaud[8] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
//   int iRetry;

//   for (size_t i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) { // Menggunakan size_t
//     Serial5.begin(c_uiBaud[i]); // Coba baud rate
//     Serial5.flush();
//     iRetry = 2;
//     s_cDataUpdate = 0;

//     do {
//       WitReadReg(AX, 3); // Minta data dari sensor
//       delay(200);

//       // Cek data yang masuk
//       while (Serial5.available()) {
//         WitSerialDataIn(Serial5.read());
//       }

//       if (s_cDataUpdate != 0) {
//         Serial.print("Sensor ditemukan pada baud rate: ");
//         Serial.println(c_uiBaud[i]);
//         return;
//       }

//       iRetry--;
//     } while (iRetry);
//   }

//   Serial.println("Sensor tidak ditemukan. Periksa koneksi!");
// }

// void setup() {
//   Serial.begin(115200); // Inisialisasi komunikasi serial
//   WitInit(WIT_PROTOCOL_NORMAL, 0x50); // Inisialisasi sensor
//   WitSerialWriteRegister(SensorUartSend); 
//   WitRegisterCallBack(SensorDataUpdate);
//   WitDelayMsRegister(Delayms);

//   Serial.println("=== Roll, Pitch, Yaw Data ===");

//   // Coba deteksi sensor secara otomatis
//   AutoScanSensor();
// }

// void loop() {
//   // Membaca data dari sensor
//   while (Serial5.available()) {
//     WitSerialDataIn(Serial5.read());
//   }

//   // Proses data jika ada pembaruan
//   if (s_cDataUpdate & ANGLE_UPDATE) {
//     // Cetak nilai sudut
//     Serial.print("Roll: ");
//     Serial.print(roll, 3);
//     Serial.print("°, Pitch: ");
//     Serial.print(pitch, 3);
//     Serial.print("°, Yaw: ");
//     Serial.print(yaw, 3);
//     Serial.println("°");

//     s_cDataUpdate &= ~ANGLE_UPDATE; // Reset flag
//   }
// }

// static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
//   Serial5.write(p_data, uiSize); // Kirim data melalui UART
//   Serial5.flush();
// }

// static void Delayms(uint16_t ucMs) {
//   delay(ucMs);
// }

// static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum) {
//   for (uint32_t i = 0; i < uiRegNum; i++) { // Menggunakan uint32_t
//     if (uiReg == Yaw) { // Update sudut jika data sudut diterima
//       roll = sReg[Roll] / 32768.0f * 180.0f; // Ambil nilai Roll
//       pitch = sReg[Pitch] / 32768.0f * 180.0f; // Ambil nilai Pitch
//       yaw = sReg[Yaw] / 32768.0f * 180.0f; // Ambil nilai Yaw
//       s_cDataUpdate |= ANGLE_UPDATE; // Set flag pembaruan sudut
//     }
//     uiReg++;
//   }
// }