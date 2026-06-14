#include <U8g2lib.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiS3.h>
#include <math.h>

#define PIN_NEOPIXEL  6
#define NUM_LEDS      16
#define PIN_BUZZER    5
#define ENV_PIN       A0

const float ADC_MAX = 16383.0;
const float VREF_MV = 5000.0;

const char* ssid     = "Titoelmaskul"; //Nombre de WiFi
const char* password = "Titoelmaskuldelcondado"; //Contraseña de WiFi
bool wifiOK = false;

const char* GS_HOST     = "script.google.com";
const char* GS_SCRIPTID = "AKfycbyP-PUV4kwtGnM2WbVSwKHOYACev7JPApCmf1Xx-S2sOcp2bPSm98E2xeWmUjOudAJkHw";
WiFiSSLClient sslClient;

U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(
  U8G2_R0, /cs=/10, /dc=/9, /reset=/8
);
Adafruit_NeoPixel anillo(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

double baselineMV    = 0.0;
double sigmaMV       = 0.0;
double umbralMV      = 0.0;
const double K_SIGMA = 3.0;
#define N_CAL 200
double muestrasCal[N_CAL];

int    contadorReps  = 0;
bool   musculoActivo = false;
double sumaCuadrados = 0.0;
double sumaAbsolutos = 0.0;
long   totalMuestras = 0;
float  rmsFinal      = 0.0;
float  arvFinal      = 0.0;
double envActual     = 0.0;

const unsigned long TIEMPO_MIN_CONTRACCION = 80;
unsigned long tiempoInicioContraccion = 0;

int estado = 0;
const int REPETICIONES_OBJETIVO = 10;

#define GRAPH_W 116
#define GRAPH_H  44
#define GRAPH_Y  16
#define GRAPH_X  12
float graficaBuf[GRAPH_W];
int   graficaIdx   = 0;
float envSuavizada = 0.0;
float escalaMaxima = 3000.0;

void pantallaTexto(const char* l1, const char* l2);
void pantallaCaptura(double envMV);
void pantallaResultados(float rms, float arv);
void ejecutarCalibracion();
void procesarCapturaEMG();
void cuentaRegresiva(int repNum);
void finalizarYProcesar();
void resetearDatos();
void conectarWiFi();
void enviarGoogleSheets(float rms, float arv, int reps);
void ajustarEscala(float val);
void setLEDs(uint8_t r, uint8_t g, uint8_t b);
void tonoInicio();
void tonoContraccion();
void tonoFin();
void tonoError();
void tonoCuenta();


// NEOPIXEL
void setLEDs(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++)
    anillo.setPixelColor(i, anillo.Color(r, g, b));
  anillo.show();
}


// BUZZER

void tonoInicio()      { tone(PIN_BUZZER, 1000, 250); delay(300); }
void tonoContraccion() { tone(PIN_BUZZER, 1200, 80); }
void tonoFin() {
  for (int f = 800; f <= 1600; f += 400) { tone(PIN_BUZZER, f, 200); delay(250); }
  noTone(PIN_BUZZER);
}
void tonoError()  { tone(PIN_BUZZER, 300, 600); delay(700); noTone(PIN_BUZZER); }
void tonoCuenta() { tone(PIN_BUZZER, 800, 100); delay(150); noTone(PIN_BUZZER); }


// FUNCIONES AUXILIARES

double leerENV() {
  return (analogRead(ENV_PIN) * VREF_MV) / ADC_MAX;
}

void ajustarEscala(float val) {
  if (val > escalaMaxima) escalaMaxima = val * 1.1f;
  else                    escalaMaxima *= 0.995f;
  if (escalaMaxima < 500.0f)  escalaMaxima = 500.0f;
  if (escalaMaxima > 5000.0f) escalaMaxima = 5000.0f;
}

void resetearDatos() {
  contadorReps = 0; musculoActivo = false;
  sumaCuadrados = 0.0; sumaAbsolutos = 0.0; totalMuestras = 0;
  baselineMV = 0.0; sigmaMV = 0.0; umbralMV = 0.0;
  rmsFinal = 0.0; arvFinal = 0.0; envActual = 0.0;
  tiempoInicioContraccion = 0; graficaIdx = 0;
  envSuavizada = 0.0; escalaMaxima = 3000.0;
  for (int i = 0; i < GRAPH_W; i++) graficaBuf[i] = 0;
}


// WIFI

void conectarWiFi() {
  pantallaTexto("Conectando", "WiFi...");
  setLEDs(255, 128, 0);
  WiFi.begin(ssid, password);
  unsigned long tInicio = millis();
  while (millis() - tInicio < 15000UL) {
    if (WiFi.status() == WL_CONNECTED &&
        WiFi.localIP() != IPAddress(0, 0, 0, 0)) break;
    delay(300);
  }
  if (WiFi.status() == WL_CONNECTED &&
      WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
    wifiOK = true;
    char ipBuf[20];
    WiFi.localIP().toString().toCharArray(ipBuf, 20);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0, 14, "WiFi OK!");
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 30, "Sheets activo");
    u8g2.drawStr(0, 46, ipBuf);
    u8g2.sendBuffer();
    setLEDs(0, 50, 255);
    delay(3000);
  } else {
    WiFi.disconnect();
    wifiOK = false;
    pantallaTexto("Sin WiFi", "Continua local");
    setLEDs(255, 0, 0);
    delay(2000);
  }
}


// GOOGLE SHEETS

void enviarGoogleSheets(float rms, float arv, int reps) {
  if (!wifiOK) return;
  pantallaTexto("Enviando...", "Google Sheets");
  setLEDs(0, 100, 255);
  String url = "/macros/s/";
  url += GS_SCRIPTID;
  url += "/exec?rms=" + String(rms, 2);
  url += "&arv="      + String(arv, 2);
  url += "&reps="     + String(reps);
  if (sslClient.connect(GS_HOST, 443)) {
    sslClient.print(String("GET ") + url + " HTTP/1.1\r\n" +
                    "Host: " + GS_HOST + "\r\n" +
                    "User-Agent: ArduinoR4WiFi\r\n" +
                    "Connection: close\r\n\r\n");
    unsigned long t = millis();
    while (sslClient.connected() && millis() - t < 5000) {
      while (sslClient.available()) sslClient.readStringUntil('\n');
    }
    sslClient.stop();
  }
}


// PANTALLAS OLED

void pantallaTexto(const char* linea1, const char* linea2 = "") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(0, 20, linea1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 40, linea2);
  u8g2.sendBuffer();
}

void pantallaCaptura(double envMV) {
  envSuavizada = 0.6f * envSuavizada + 0.4f * (float)envMV;
  ajustarEscala(envSuavizada);
  graficaBuf[graficaIdx] = envSuavizada;
  graficaIdx = (graficaIdx + 1) % GRAPH_W;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, musculoActivo ? "ACTIVADO" : "Mueve brazo");
  char repBuf[16];
  snprintf(repBuf, sizeof(repBuf), "Rep %d/%d", contadorReps + 1, REPETICIONES_OBJETIVO);
  u8g2.drawStr(70, 10, repBuf);

  u8g2.setFont(u8g2_font_4x6_tr);
  char lblMax[8], lblMid[8];
  snprintf(lblMax, sizeof(lblMax), "%.0f", escalaMaxima);
  snprintf(lblMid, sizeof(lblMid), "%.0f", escalaMaxima / 2.0f);
  u8g2.drawStr(0, GRAPH_Y + 5,               lblMax);
  u8g2.drawStr(0, GRAPH_Y + GRAPH_H / 2 + 3, lblMid);
  u8g2.drawStr(0, GRAPH_Y + GRAPH_H + 1,     "0");

  int yFin = GRAPH_Y + GRAPH_H;
  u8g2.drawFrame(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H + 1);

  int yUmbral = yFin - (int)((umbralMV / escalaMaxima) * GRAPH_H);
  yUmbral = constrain(yUmbral, GRAPH_Y + 1, yFin - 1);
  for (int x = GRAPH_X; x < GRAPH_X + GRAPH_W; x += 3)
    u8g2.drawPixel(x, yUmbral);

  int yAnterior = -1, xAnterior = -1;
  for (int x = 1; x < GRAPH_W - 1; x++) {
    int   idx   = (graficaIdx + x) % GRAPH_W;
    float valor = graficaBuf[idx];
    int   y     = yFin - (int)((valor / escalaMaxima) * GRAPH_H);
    y = constrain(y, GRAPH_Y + 1, yFin - 1);
    int xPantalla = GRAPH_X + x;
    if (yAnterior != -1)
      u8g2.drawLine(xAnterior, yAnterior, xPantalla, y);
    yAnterior = y;
    xAnterior = xPantalla;
  }
  u8g2.sendBuffer();
}

void pantallaResultados(float rms, float arv) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 14, "Resultados");
  u8g2.drawHLine(0, 17, 128);
  char buf[32];
  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "Det:   %d/%d", contadorReps, REPETICIONES_OBJETIVO);
  u8g2.drawStr(0, 30, buf);
  snprintf(buf, sizeof(buf), "RMS:   %.1f mV", rms);
  u8g2.drawStr(0, 44, buf);
  snprintf(buf, sizeof(buf), "ARV:   %.1f mV", arv);
  u8g2.drawStr(0, 58, buf);
  u8g2.sendBuffer();
}


// CUENTA REGRESIVA PARA INICIAR CADA CONTRACCIÓN

void cuentaRegresiva(int repNum) {
  setLEDs(255, 255, 255);
  for (int c = 3; c >= 1; c--) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    char linea1[24];
    snprintf(linea1, sizeof(linea1), "Rep %d de %d", repNum, REPETICIONES_OBJETIVO);
    u8g2.drawStr(0, 14, linea1);
    u8g2.drawStr(0, 30, "Preparar en:");
    u8g2.setFont(u8g2_font_fur30_tn);
    char numBuf[4];
    snprintf(numBuf, sizeof(numBuf), "%d", c);
    u8g2.drawStr(52, 62, numBuf);
    u8g2.sendBuffer();
    tonoCuenta();
    delay(1000);
  }

  tone(PIN_BUZZER, 600, 400);
  delay(450);
  noTone(PIN_BUZZER);
  setLEDs(0, 0, 200); // azul = capturando
}


// CALIBRACIÓN

void ejecutarCalibracion() {
  pantallaTexto("Paciente", "en reposo...");
  setLEDs(255, 255, 255);
  delay(4000);

  pantallaTexto("Calibrando", "No mover brazo");
  setLEDs(255, 255, 255);

  double sumaM = 0.0;
  for (int i = 0; i < N_CAL; i++) {
    muestrasCal[i] = leerENV();
    sumaM += muestrasCal[i];
    if (i % 20 == 0) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.drawStr(0, 14, "Calibrando...");
      u8g2.drawFrame(0, 24, 128, 10);
      u8g2.drawBox(1, 25, map(i, 0, N_CAL, 0, 126), 8);
      u8g2.setFont(u8g2_font_6x10_tr);
      char buf[20];
      snprintf(buf, sizeof(buf), "%d%%", (i * 100) / N_CAL);
      u8g2.drawStr(56, 46, buf);
      u8g2.sendBuffer();
    }
    delay(10);
  }

  baselineMV = sumaM / N_CAL;
  double sumaSigma = 0.0;
  for (int i = 0; i < N_CAL; i++) {
    double diff = muestrasCal[i] - baselineMV;
    sumaSigma += diff * diff;
  }
  sigmaMV  = sqrt(sumaSigma / N_CAL);
  umbralMV = baselineMV + K_SIGMA * sigmaMV;

  if (sigmaMV < 1.0) {
    pantallaTexto("Senal plana", "Verifica sensor");
    setLEDs(255, 0, 0);
    tonoError();
    delay(3000);
    umbralMV = baselineMV + 50.0;
  }

  if (sigmaMV > 200.0) {
    pantallaTexto("Mucho ruido", "Reintentando...");
    setLEDs(255, 0, 0);
    tonoError();
    delay(2000);
    estado = 0;
    return;
  }

  char buf1[24];
  snprintf(buf1, sizeof(buf1), "U=%.0f mV", umbralMV);
  pantallaTexto("Calibrado OK", buf1);
  setLEDs(0, 255, 0);
  tonoInicio();
  delay(2500);
  estado = 1;
}


// CAPTURA

void procesarCapturaEMG() {
  cuentaRegresiva(contadorReps + 1);

  for (int i = 0; i < GRAPH_W; i++) graficaBuf[i] = (float)baselineMV;
  graficaIdx = 0;
  envSuavizada = (float)baselineMV;
  musculoActivo = false;
  bool repDetectada = false;

  while (!repDetectada) {
    envActual = leerENV();

    if (envActual > umbralMV) {
      if (!musculoActivo) {
        musculoActivo = true;
        tiempoInicioContraccion = millis();
      }
      double centrada = envActual - baselineMV;
      sumaCuadrados += centrada * centrada;
      sumaAbsolutos += fabs(centrada);
      totalMuestras++;

      double exceso   = envActual - umbralMV;
      double rangoVis = umbralMV - baselineMV;
      int intensidad  = constrain((int)((exceso / rangoVis) * 200.0) + 80, 80, 255);
      setLEDs(intensidad, 0, 0); // rojo = activado

    } else {
      if (musculoActivo) {
        unsigned long dur = millis() - tiempoInicioContraccion;
        if (dur >= TIEMPO_MIN_CONTRACCION) {
          contadorReps++;
          delay(200);        // espera a que el brazo baje completamente
          tonoContraccion(); 
          repDetectada = true;
        }
        musculoActivo = false;
      }
      setLEDs(0, 0, 200); // azul = esperando
    }

    pantallaCaptura(envActual);
    delay(10);
  }

  setLEDs(0, 255, 0); // verde = rep completada
  delay(800);

  if (contadorReps >= REPETICIONES_OBJETIVO) {
    finalizarYProcesar();
  }
}


// FINALIZACIÓN

void finalizarYProcesar() {
  estado = 2;
  setLEDs(0, 255, 0);
  tonoFin();

  if (totalMuestras > 0) {
    rmsFinal = sqrt(sumaCuadrados / totalMuestras);
    arvFinal = sumaAbsolutos / totalMuestras;
  }

  pantallaResultados(rmsFinal, arvFinal);
  delay(2000);
  enviarGoogleSheets(rmsFinal, arvFinal, contadorReps);

  if (wifiOK) {
    pantallaTexto("Listo!", "Datos en Sheets");
  } else {
    pantallaTexto("Listo!", "Sin WiFi hoy");
  }
  delay(3000);
}


// SETUP Y LOOP

void setup() {
  analogReadResolution(14);
  pinMode(PIN_BUZZER, OUTPUT);
  u8g2.begin();
  u8g2.setContrast(200);
  anillo.begin();
  anillo.setBrightness(80);
  setLEDs(10, 10, 10);
  pantallaTexto("EMG Pasivo", "Iniciando...");
  delay(1000);
  conectarWiFi();
}

void loop() {
  if (estado == 0) {
    resetearDatos();
    ejecutarCalibracion();
  }
  if (estado == 1) {
    procesarCapturaEMG();
  }
  if (estado == 2) {
    delay(1000);
  }
}