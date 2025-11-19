/*
 * ESP8266 + RAK3172 + KY-038 (Versión Fusionada)
 * - Sensado: Alta precisión (New Code logic)
 * - Comunicación: LoRaWAN OTAA (Old Code logic)
 * - Comportamiento: Mide cada segundo, acumula estadísticas y envía cada 60s.
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// ==================== CONFIG LORAWAN (RAK3172) ====================
#define RAK_RX   13   // ESP8266 RX  <= RAK3172 TX
#define RAK_TX   15   // ESP8266 TX  => RAK3172 RX
#define RAK_BAUD 9600
SoftwareSerial rakSerial(RAK_RX, RAK_TX);

// Credenciales OTAA
const char* DEV_EUI = "AC1F09FFFE139500";
const char* APP_EUI = "741b87cd62d73596";
const char* APP_KEY = "99781e5f85f073bda158ebef0716183f";

// Configuración de Región
const bool   USE_ADR    = true;
const int    LORA_BAND  = 6;      // 6 = AU915
const char* SUBBAND    = "0001"; // Subbanda 1
const uint8_t FPORT     = 2;

// Intervalo de envío a Chirpstack (en milisegundos)
const unsigned long TX_INTERVAL = 60000; // 60 segundos

// ==================== CONFIG SENSOR DE AUDIO (NUEVO) ====================
const int PIN_AUDIO = A0;
const int SAMPLES = 4545;             // Muestras por ventana de 100ms
const unsigned long SAMPLING_TIME_US = 22;   
// Nota: El periodo de muestreo físico es ~100ms con estos valores

// Parámetros de calibración (Del código nuevo)
float VOLTAGE_REFERENCE = 3.3;
float SENSOR_SENSITIVITY = 0.050;     // V/Pa
float REFERENCE_PRESSURE = 0.00002;   // 20 uPa
float CALIBRATION_OFFSET_DB = -25.0;  // AJUSTA ESTO SEGÚN TU CALIBRACIÓN FINAL

// Variables de Audio
static int samples[SAMPLES];          // Buffer estático para no saturar el stack
float rmsValue = 0;

// Variables para estadísticas de Envío (Ciclo de 60s)
float txMinDB = 1000;
float txMaxDB = -1000;
double txSumDB = 0;
int   txCount = 0;
unsigned long lastTxTime = 0;

// ==================== FUNCIONES DE AUDIO (NUEVO CÓDIGO) ====================

void performSampling() {
    unsigned long startTime = micros();
    
    // Desactivar interrupciones momentáneamente para precisión en ADC
    // NOTA: SoftwareSerial podría perder bytes aquí si el RAK habla, 
    // pero en este diseño el RAK solo habla cuando le preguntamos.
    noInterrupts();
    
    for(int i = 0; i < SAMPLES; i++) {
        while(micros() - startTime < (i * SAMPLING_TIME_US)) {
            // Espera activa
        }
        samples[i] = analogRead(PIN_AUDIO);
    }
    
    interrupts();
}

float calculateRMS() {
    long sumSquares = 0;
    long sum = 0;
    
    // 1. Calcular media (DC offset)
    for(int i = 0; i < SAMPLES; i++) sum += samples[i];
    int dcOffset = sum / SAMPLES;
    
    // 2. Suma de cuadrados sin DC
    for(int i = 0; i < SAMPLES; i++) {
        int adjusted = samples[i] - dcOffset;
        sumSquares += (long)adjusted * adjusted;
    }
    
    return sqrt((float)sumSquares / SAMPLES);
}

float convertToDecibels(float rmsADC) {
    // ADC -> Voltaje
    float voltage = (rmsADC / 1024.0) * VOLTAGE_REFERENCE;
    
    // Voltaje -> Presión
    float pressure = voltage / SENSOR_SENSITIVITY;
    
    // Presión -> dB
    if(pressure <= 0) return -100;
    float db = 20.0 * log10(pressure / REFERENCE_PRESSURE);
    
    return db + CALIBRATION_OFFSET_DB; // Incluye calibración
}

// ==================== FUNCIONES LORAWAN (ANTIGUO CÓDIGO) ====================

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

int queryJoined() {
    sendAT("AT+NJS=?");
    char line[64];
    while (readLineFromRak(line, sizeof(line), 2000)) {
        Serial.println(String("< ") + line);
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
            Serial.println("Reintentando JOIN...");
            sendAT("AT+JOIN=1:1:10:5");
            expectOK(5000);
        }
        for (uint16_t s=0; s<backoff_s; s++) { delay(1000); yield(); }
        if (backoff_s < 60) backoff_s *= 2;
    }
    return (queryJoined() == 1);
}

// Convierte cadena JSON a HEX para RAK
size_t jsonToHex(const char* json, char* hexOut, size_t hexOutLen) {
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

// Envía Min, Max y Promedio
bool sendMetrics(float min_db, float max_db, float avg_db) {
    char json[64];
    // Formato JSON: {"min":30.5,"max":80.2,"avg":55.4}
    // Se usa %.1f para ahorrar bytes en el payload
    snprintf(json, sizeof(json), "{\"min\":%.1f,\"max\":%.1f,\"avg\":%.1f}", min_db, max_db, avg_db);

    char hex[130]; // Espacio para HEX (2*len(json) + null)
    jsonToHex(json, hex, sizeof(hex));

    char cmd[150];
    snprintf(cmd, sizeof(cmd), "AT+SEND=%u:%s", FPORT, hex);
    sendAT(cmd);
    
    bool ok = expectOK(5000);
    if (ok) {
        Serial.print("📡 Uplink enviado: ");
        Serial.println(json);
    } else {
        Serial.println("❌ ERROR al enviar uplink");
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

    Serial.println("\n=== ESP8266 Audio Monitor + LoRaWAN ===");
    
    // --- 1. Init LoRaWAN ---
    sendAT("AT+NWM=1");      expectOK();
    sendAT("AT+NJM=1");      expectOK();
    sendAT("AT+CLASS=1");    expectOK();
    
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "AT+BAND=%d", LORA_BAND); sendAT(cmd); expectOK();
    snprintf(cmd, sizeof(cmd), "AT+MASK=%s", SUBBAND);   sendAT(cmd); expectOK();
    if (USE_ADR) { sendAT("AT+ADR=1"); expectOK(); }

    snprintf(cmd, sizeof(cmd), "AT+DEVEUI=%s", DEV_EUI); sendAT(cmd); expectOK();
    snprintf(cmd, sizeof(cmd), "AT+APPEUI=%s", APP_EUI); sendAT(cmd); expectOK();
    snprintf(cmd, sizeof(cmd), "AT+APPKEY=%s", APP_KEY); sendAT(cmd); expectOK();

    Serial.println("Intentando JOIN...");
    sendAT("AT+JOIN=1:1:10:5"); expectOK(5000);
    
    if (!waitForJoin(6, 5)) {
        Serial.println("❌ Error fatal: No hay conexión LoRaWAN.");
    } else {
        Serial.println("✅ Conectado a LoRaWAN.");
    }
    
    // Inicializar timer
    lastTxTime = millis();
}

// ==================== LOOP ====================
void loop() {
    // 1. Realizar una medición (toma aprox 100ms + procesamiento)
    performSampling();
    float rms = calculateRMS();
    float dbCurrent = convertToDecibels(rms);
    
    // 2. Actualizar estadísticas de este periodo de transmisión
    if (dbCurrent < txMinDB) txMinDB = dbCurrent;
    if (dbCurrent > txMaxDB) txMaxDB = dbCurrent;
    txSumDB += dbCurrent;
    txCount++;
    
    // Feedback local (Opcional: para ver que sigue vivo)
    Serial.printf("Medición #%d: %.2f dB\n", txCount, dbCurrent);
    
    // 3. Verificar si es hora de enviar por LoRaWAN
    if (millis() - lastTxTime > TX_INTERVAL) {
        // Calcular promedio
        float txAvgDB = 0;
        if (txCount > 0) txAvgDB = (float)(txSumDB / txCount);
        
        Serial.println("\n--- 📤 Enviando Reporte LoRaWAN ---");
        Serial.printf("Min: %.1f | Max: %.1f | Avg: %.1f\n", txMinDB, txMaxDB, txAvgDB);
        
        sendMetrics(txMinDB, txMaxDB, txAvgDB);
        
        // Resetear estadísticas
        txMinDB = 1000;
        txMaxDB = -1000;
        txSumDB = 0;
        txCount = 0;
        lastTxTime = millis();
    }
    
    // 4. Espera para completar ciclo de medición
    // El muestreo dura 100ms. Esperamos 900ms para completar ~1 seg por ciclo
    // Esto nos da ~60 mediciones por cada envío de 60 segundos.
    delay(900);
    yield();
}