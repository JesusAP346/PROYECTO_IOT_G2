/*
 * ESP8266 + RAK3172 (LoRaWAN OTAA) + KY-038
 * - Sensado acústico con linealización ADC + RMS → dB SPL
 * - JSON compacto {"noise":xx.xx} → HEX ASCII → AT+SEND=2:<HEX>
 * - Join robusto (AT+NJS?) + ADR + región AU915 subbanda 1 por defecto
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// ==================== CONFIG LORAWAN ====================
#define RAK_RX   13   // ESP8266 RX  <= RAK3172 TX
#define RAK_TX   15   // ESP8266 TX  => RAK3172 RX
#define RAK_BAUD 9600
SoftwareSerial rakSerial(RAK_RX, RAK_TX);

// Credenciales OTAA
const char* DEV_EUI = "AC1F09FFFE139500";
const char* APP_EUI = "741b87cd62d73596";
const char* APP_KEY = "99781e5f85f073bda158ebef0716183f";

// Región/subbanda (ajusta si es necesario)
const bool   USE_ADR    = true;   // habilitar ADR para optimizar DR/tx
const int    LORA_BAND  = 6;      // 6 = AU915 en RAK
const char*  SUBBAND    = "0001"; // sub-banda 1 típica en UG56
const uint8_t FPORT     = 2;      // puerto de aplicación para decoder

// ==================== CONFIG SENSOR KY-038 ====================
const int PIN_AUDIO = A0;
const int SAMPLES = 2000;
const unsigned long SAMPLING_TIME_US = 50; // ~20 kHz
const int OVERSAMPLE = 4;

const float SENSOR_SENSITIVITY = 0.050f;     // V/Pa (ajusta a tu módulo)
const float REFERENCE_PRESSURE = 0.00002f;   // 20 uPa
float CALIBRATION_OFFSET_DB = 0.0f;          // calibra con sonómetro

// Buffers y variables
static int   samples[SAMPLES];
static float rmsValue = 0, dbValue = 0, dbCalibrated = 0;

// ==================== LUT LINEALIZACIÓN ADC (0–3.0V) ====================
// Si tu A0 es 0–1.0V, AJUSTA ESTA LUT a 0–1.0V o usa divisor adecuado.
const int   LUT_SIZE = 11;
const int   adcLUT[LUT_SIZE]   = {0, 102, 204, 307, 409, 512, 614, 717, 819, 921, 1023};
const float voltLUT[LUT_SIZE]  = {0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4, 2.7, 3.0};

// ==================== UTILIDADES ====================
void yieldOften(uint16_t n) { for (uint16_t i=0;i<n;i++) yield(); }

float linearizeADC(int rawADC) {
  if (rawADC <= 0)    return 0.0f;
  if (rawADC >= 1023) return voltLUT[LUT_SIZE - 1];
  for (int i=0; i<LUT_SIZE-1; i++) {
    if (rawADC >= adcLUT[i] && rawADC <= adcLUT[i+1]) {
      float t = float(rawADC - adcLUT[i]) / float(adcLUT[i+1] - adcLUT[i]);
      return voltLUT[i] + t * (voltLUT[i+1] - voltLUT[i]);
    }
  }
  return voltLUT[LUT_SIZE - 1];
}

int readADC_Filtered() {
  long sum = 0;
  for (int i=0; i<OVERSAMPLE; i++) {
    sum += analogRead(PIN_AUDIO);
    delayMicroseconds(8);
  }
  return sum / OVERSAMPLE;
}

void performSampling() {
  unsigned long t0 = micros();
  for (int i=0; i<SAMPLES; i++) {
    unsigned long target = t0 + (i * SAMPLING_TIME_US);
    // Espera activa con cedencia al WDT
    while (micros() < target) { /* spin */ }
    samples[i] = readADC_Filtered();
    if ((i & 0x3F) == 0) yield(); // cede periódicamente
  }
}

float calculateRMS() {
  long acc = 0;
  for (int i=0; i<SAMPLES; i++) acc += samples[i];
  long dc = acc / SAMPLES;

  double sumSq = 0.0;
  for (int i=0; i<SAMPLES; i++) {
    long a = samples[i] - dc;
    sumSq += double(a) * double(a);
  }
  return sqrt(float(sumSq / SAMPLES));
}

float adcToVoltage(float adcRms) {
  return linearizeADC(int(adcRms));
}

float voltageToPressure(float v) {
  // pequeña corrección de extremos (opcional)
  if (v < 0.3f) v *= 1.05f;
  else if (v > 2.7f) v *= 0.98f;
  return v / SENSOR_SENSITIVITY;
}

float toDecibels(float pressure) {
  if (pressure <= 0.0f) return -100.0f;
  return 20.0f * log10f(pressure / REFERENCE_PRESSURE);
}

float measureNoiseDB() {
  performSampling();
  rmsValue = calculateRMS();
  dbValue  = toDecibels( voltageToPressure( adcToVoltage(rmsValue) ) );
  dbCalibrated = dbValue + CALIBRATION_OFFSET_DB;
  return dbCalibrated;
}

// =============== Serial AT helpers (buffers sin String) ===============
bool readLineFromRak(char* out, size_t outLen, uint32_t timeoutMs = 3000) {
  size_t idx = 0; unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (rakSerial.available()) {
      char c = rakSerial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (idx > 0) { out[idx] = 0; return true; }
      } else if (idx < outLen - 1) {
        out[idx++] = c;
      }
    }
    yield();
  }
  if (idx > 0) { out[idx] = 0; return true; }
  return false;
}

void sendAT(const char* cmd) {
  rakSerial.print(cmd); rakSerial.print("\r\n");
  Serial.print("> "); Serial.println(cmd);
}

bool expectOK(uint32_t timeoutMs = 3000) {
  char line[96];
  while (readLineFromRak(line, sizeof(line), timeoutMs)) {
    Serial.println(String("< ") + line);
    if (strstr(line, "OK")) return true;
    if (strstr(line, "ERROR")) return false;
  }
  return false;
}

// Devuelve 1 si joined, 0 si no, -1 si error/time-out
int queryJoined() {
  sendAT("AT+NJS=?"); // algunos firmware; si no, AT+NJS?
  char line[64];
  while (readLineFromRak(line, sizeof(line), 2000)) {
    Serial.println(String("< ") + line);
    // Respuestas típicas: "1" o "0", o "AT+NJS: 1"
    if (strstr(line, "AT+NJS") || strlen(line)<=3) {
      if (strchr(line, '1')) return 1;
      if (strchr(line, '0')) return 0;
    }
    if (strstr(line, "ERROR")) return -1;
  }
  return -1;
}

bool waitForJoin(uint8_t attempts = 5, uint16_t backoff_s = 5) {
  for (uint8_t i=0; i<attempts; i++) {
    int st = queryJoined();
    if (st == 1) return true;
    if (st == 0) {
      Serial.println("Aún no join. Reintentando JOIN...");
      sendAT("AT+JOIN=1:1:10:5");
      expectOK(5000);
    } else {
      Serial.println("No se pudo leer estado de JOIN, reintento...");
    }
    for (uint16_t s=0; s<backoff_s; s++) { delay(1000); yield(); }
    if (backoff_s < 60) backoff_s *= 2; // backoff exponencial suave
  }
  return (queryJoined() == 1);
}

// JSON → HEX (buffers)
size_t jsonToHex(const char* json, char* hexOut, size_t hexOutLen) {
  // Cada byte => 2 chars hex; asegúrate hexOutLen >= len(json)*2 + 1
  size_t n = strlen(json);
  const char* hex = "0123456789ABCDEF";
  size_t w = 0;
  for (size_t i=0; i<n && (w+2)<hexOutLen; i++) {
    uint8_t b = (uint8_t)json[i];
    hexOut[w++] = hex[b >> 4];
    hexOut[w++] = hex[b & 0x0F];
  }
  hexOut[w] = 0;
  return w;
}

bool sendJSON(float noiseDB) {
  char json[48];
  // JSON compacto. Si necesitas más métricas, añadir aquí.
  // Ojo con el tamaño máximo por DR/Región (≈ 51B a DR0 AU915).
  snprintf(json, sizeof(json), "{\"noise\":%.2f}", noiseDB);

  char hex[110]; // 2x JSON + nulo
  jsonToHex(json, hex, sizeof(hex));

  char cmd[140];
  snprintf(cmd, sizeof(cmd), "AT+SEND=%u:%s", FPORT, hex);
  sendAT(cmd);
  bool ok = expectOK(5000);
  if (ok) {
    Serial.print("Uplink enviado (JSON): ");
    Serial.println(json);
  } else {
    Serial.println("ERROR al enviar uplink");
  }
  return ok;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(50);
  rakSerial.begin(RAK_BAUD);
  delay(500);

  pinMode(PIN_AUDIO, INPUT);

  Serial.println("\n=== Init LoRaWAN (RAK3172) ===");
  // Modo LoRaWAN, OTAA, Clase A, región y subbanda
  sendAT("AT+NWM=1");      expectOK();
  sendAT("AT+NJM=1");      expectOK();
  sendAT("AT+CLASS=1");    expectOK();
  // Región AU915 y subbanda (ajustar según gateway)
  char cmd[48];
  snprintf(cmd, sizeof(cmd), "AT+BAND=%d", LORA_BAND); sendAT(cmd); expectOK();
  snprintf(cmd, sizeof(cmd), "AT+MASK=%s", SUBBAND);   sendAT(cmd); expectOK();
  if (USE_ADR) { sendAT("AT+ADR=1"); expectOK(); }

  // Credenciales
  snprintf(cmd, sizeof(cmd), "AT+DEVEUI=%s", DEV_EUI); sendAT(cmd); expectOK();
  snprintf(cmd, sizeof(cmd), "AT+APPEUI=%s", APP_EUI); sendAT(cmd); expectOK();
  snprintf(cmd, sizeof(cmd), "AT+APPKEY=%s", APP_KEY); sendAT(cmd); expectOK();

  // Intentar JOIN + esperar join aceptado
  sendAT("AT+JOIN=1:1:10:5"); expectOK(5000);
  if (!waitForJoin(6, 5)) {
    Serial.println("❌ No se logró OTAA JOIN. Revisa credenciales/región/subbanda.");
  } else {
    Serial.println("✅ JOIN aceptado. Listo para enviar.");
  }
}

// ==================== LOOP ====================
void loop() {
  // 1) Medir ruido (dB SPL calibrado)
  float noise = measureNoiseDB();
  Serial.printf("Nivel sonoro: %.2f dB SPL\n", noise);

  // 2) Enviar JSON por LoRaWAN
  sendJSON(noise);

  // 3) Periodo de reporte (ajusta si necesitas)
  for (int i=0; i<60; i++) { delay(1000); yield(); } // 60 s
}