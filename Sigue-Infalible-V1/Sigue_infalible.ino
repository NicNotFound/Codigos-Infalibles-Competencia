// Pines motores
#include "Pines.h"

#define AIN1 16
#define AIN2 17
#define PWMA 4
#define BIN1 5
#define BIN2 18
#define PWMB 19

// Botón
#define BOTON 12

#define LED 2

#include <QTRSensors.h>
#include <Wire.h>
#include <MPU6050_light.h>

//variables por entender
bool primerCuadradoIzquierda  = false;
bool primerCuadradoDerecha    = false;
bool segundoCuadradoIzquierda = false;
bool segundoCuadradoDerecha   = false;
int  contadorCasosEspeciales  = 0;
bool huboLineaCentral         = false; // flag general

int  marcaCuadradoDir[2] = {0, 0};  // hasta 2 marcas: -1 izq, +1 der
int  totalMarcasGuardadas = 0;
bool tieneMarcaCuadrado = false;
bool forzarProximaSemi  = false;


// ----------------- Sensores QTR
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
QTRSensors qtr;

// ----------------- PID (modo normal)
float Kp = 0.15, Ki = 0.0, Kd = 0.5;
int lastError = 0, integral = 0;
int umbral = 4000;
const int velocidadBase = 50;

// ----------------- Estados
bool escaneando = false;
bool flagIzquierda = false, flagDerecha = false;
int filtroBordeCount = 0;

// ----------------- Tiempos
const unsigned long tRetroceso = 350;
const unsigned long tScan = 1000;
unsigned long tInicioScan = 0;

// ----------------- MPU6050
MPU6050 mpu(Wire);
float yawZero = 0.0;

// ----------------- Flags de marca
bool marcaPrimeraIzq = false;
bool marcaPrimeraDer = false;
bool marcaGuardada = false;  // indica si ya se guardó la primera marca

// ----------------- Prototipos

const int freq = 5000; const int resolution = 8;


void inicializarMotores();
void Motor(int velIzq, int velDer);
void girarIzquierda(float grados);
void girarDerecha(float grados);
void PID(uint16_t position);
void guardarMarca();

void setup() {
  Serial.begin(115200);
  inicializarMotores();
  pinMode(LED, OUTPUT);
  pinMode(BOTON, INPUT);

  calibracionSensores();
  calibracionGiroscopio();
}

void loop() {
  qtr.read(sensorValues);

  // Filtrado simple para el cálculo de posición
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 3500) sensorValues[i] = 0;
  }

  // Posición manual ponderada
  uint32_t sumaPesada = 0;
  uint32_t sumaTotal  = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    sumaPesada += (uint32_t)sensorValues[i] * (i * 1000);
    sumaTotal  += sensorValues[i];
  }
  uint16_t position = (sumaTotal > 0) ? (sumaPesada / sumaTotal) : 0;

  // ---- NUEVO: manejo de GAPS (todo blanco) ----
  if (sumaTotal == 0) {
    // Avanza recto con velocidad base para no “morir” en líneas segmentadas
    Motor(velocidadBase - 10, velocidadBase - 10);
    // Evita acumular integral/derivativos inútiles
    integral = 0;
    lastError = 0;
    // No corremos PID en esta iteración
    return;
  }

  // Disparador de cruce por extremos
  static int contadorCruce = 0;
  if (sensorValues[0] > 4000 || sensorValues[7] > 4000) contadorCruce++;
  else contadorCruce = 0;

  if (contadorCruce > 3) {
    evaluarCruce();
    contadorCruce = 0;
    return;
  }

  // Seguimiento de línea normal
  PID(position);
}

void evaluarCruce() {
  // Umbrales
  const int TH_LADO   = 4000; // extremos (0 y 7)
  const int TH_CENTRO = 3200; // centrales (2..5) para "hay línea al frente"

  // (2) Retroceder
  Motor(-40, -40);
  delay(300);
  Motor(0, 0);
  delay(100);

  // (3) Avanzar ESCANEANDO para clasificar
  bool vioIzq = false, vioDer = false, vioCentroDuranteScan = false;
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    qtr.read(sensorValues);
    if (sensorValues[0] > TH_LADO) vioIzq = true;
    if (sensorValues[7] > TH_LADO) vioDer = true;
    for (int i = 2; i <= 4; i++) {
      if (sensorValues[i] > TH_CENTRO) {
        vioCentroDuranteScan = true;
        break;
      }
    }
    Motor(40, 40);   // avance suave de confirmación
    delay(10);
  }

  // (4) Detenerse
  Motor(0, 0);
  delay(100);

  // (5) Revisar si hay línea al frente (lectura estática final)
  qtr.read(sensorValues);
  bool hayLineaFinal = false;
  for (int i = 2; i <= 4; i++) {
    if (sensorValues[i] > TH_CENTRO) {
      hayLineaFinal = true;
      break;
    }
  }

  // (6) Tomar decisión

  // --- SEMI-INTERSECCIÓN (solo un lado) ---
  if (vioIzq ^ vioDer) {
    if (hayLineaFinal) {
      // Si hay orden pendiente: FORZAR giro en esta semi (según la semi actual)
      if (forzarProximaSemi) {
        if (vioIzq)  {
          girarIzquierda(80);
        }
        else         {
          girarDerecha(80);
        }
        forzarProximaSemi = false;  // consumir la orden
        return;
      }

      // Si NO hay forzado: guardar marca (máx. 2)
      if (totalMarcasGuardadas < 2) {
        if (vioIzq)  {
          marcaCuadradoDir[totalMarcasGuardadas] = -1;
        }
        if (vioDer)  {
          marcaCuadradoDir[totalMarcasGuardadas] = +1;
        }
        totalMarcasGuardadas++;
        tieneMarcaCuadrado = true;
      }

      Motor(40, 40);
      delay(140);
      Motor(0, 0);
      return; // volver al PID
    } else {
      // SEMI sin línea → giro normal inmediato
      if (vioIzq)  {
        girarIzquierda(80);
        return;
      }
      if (vioDer)  {
        girarDerecha(80);
        return;
      }
    }
  }

  // --- INTERSECCIÓN COMPLETA (ambos lados) ---
  if (vioIzq && vioDer) {
    if (!hayLineaFinal) {
      // COMPLETA y SIN línea: usar próxima marca si existe
      if (totalMarcasGuardadas > 0) {
        int dir = marcaCuadradoDir[0];

        // Desplazar las marcas para que la segunda pase a ser primera
        for (int i = 0; i < totalMarcasGuardadas - 1; i++) {
          marcaCuadradoDir[i] = marcaCuadradoDir[i + 1];
        }
        totalMarcasGuardadas--;

        // Ejecutar el giro
        if (dir < 0) girarIzquierda(80);
        else          girarDerecha(80);

        // Preparar forzado en la próxima semi si aún queda otra marca
        forzarProximaSemi = true;
        return;
      } else {
        // *** NUEVO: Final sin marcas → detener 5 segundos y terminar ***
        Motor(0, 0);
        delay(5000);
        while (true) {} // fin
      }
    } else {
      // COMPLETA con línea → comportamiento normal: recto un poco
      Motor(40, 40); delay(140); Motor(0, 0);
      return;
    }
  }

  // --- Nada concluyente ---
  Motor(0, 0);
  while (true) {}
}
