/*
  ============================================================
  MotoGuard V9 PESOv4
  ============================================================
  v4: MENOS SENSIBLE (viento / vibraciones leves). Ver SENSIBILIDAD.
    - Un solo parametro, SENSIBILIDAD (1 baja, 2 media, 3 alta), ajusta
      todos los umbrales del MPU y del SW420.
    - El SW420 ahora necesita varios pulsos seguidos (no uno solo) para
      pasar a ALERTA, y ya no cuenta pulsos que lleguen demasiado juntos.
    - Un disparo debil del MPU necesita 2 lecturas seguidas fuera de
      umbral, y la confirmacion en ALERTA pide mas eventos.
    - Consejo fisico: el modulo SW420 tiene un potenciometro azul; girarlo
      para BAJAR su sensibilidad es lo mas efectivo contra el viento.

  Cambios de v3 respecto a V9_PESOv2:

  1) MPU6050 INDEPENDIENTE DEL SW420
     - Antes: cada llamada HTTP (Firebase / Telegram / GPS) bloqueaba el
       loop entre 0.5 y 3 segundos. Durante ese tiempo el MPU no se leia
       (el SW420 si "se acordaba" porque usa interrupcion). Por eso solo
       se detectaba cuando habia vibracion.
     - Ahora TODA la red corre en una tarea aparte (nucleo 0). El loop
       principal nunca se bloquea y lee el MPU 25 veces por segundo.
     - Deteccion nueva, sin necesidad del SW420:
         * Inclinacion: angulo entre la gravedad actual y la del baseline
           (funciona en cualquier orientacion de montaje).
         * Rotacion acumulada del giroscopio: detecta un cambio de
           DIRECCION lento (girar la moto sobre su eje vertical), que el
           acelerometro por si solo no puede ver.
         * Giro rapido y sacudida/levantamiento (umbrales mas sensibles).
     - Un disparo "fuerte" del MPU confirma el movimiento directamente.
       El SW420 y los disparos "debiles" siguen pasando por ALERTA.

  2) PESO
     - Imprime por Serial el peso cada segundo.
     - Publica en Firebase desde el arranque (la app ya ve "INICIANDO"
       o "BALANZA_ERROR" si el HX711 no responde).
     - Mensaje "PESO CORRECTO" / "PESO INCORRECTO" por Serial y en la app.
     - Peso incorrecto -> alerta en historial + Telegram + moto bloqueada.
     - Salida de rele (RELE_MOTO_PIN): solo habilita la moto con peso
       dentro del rango autorizado.

  Conexionado (ESP32): igual que antes, mas:
    Rele/modulo de corte de encendido -> GPIO 26 (RELE_MOTO_PIN)
  ============================================================
*/

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <TinyGPS++.h>
#include <HX711.h>

// ---------------------- WIFI ----------------------
const char* WIFI_SSID = "30 Wifi";
const char* WIFI_PASSWORD = "tiziano2008@";

// ---------------------- TELEGRAM ----------------------
const char* TELEGRAM_BOT_TOKEN = "8804247675:AAFr0FMysJJCqRkSEIM83jRw7Jaye0zdYuE";
String telegramChatId = "";   // solo lo toca la tarea de red

// ---------------------- IDENTIDAD / FIREBASE ----------------------
const char* CODIGO_DISPOSITIVO = "MG-0003";
const char* FIREBASE_BASE_URL =
  "https://motoguard-45eeb-default-rtdb.firebaseio.com/MotoGuard/dispositivos/";

// ---------------------- PINES ----------------------
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68
#define SW420_PIN 4
#define LED_ALERTA_PIN 2
#define RELE_MOTO_PIN 26          // salida que habilita/bloquea el encendido
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600
#define HX711_DT_PIN  32
#define HX711_SCK_PIN 33

// true  = el modulo rele se ACTIVA con nivel alto (HIGH)
// false = el modulo rele se activa con nivel bajo (muchos modulos de 1 canal)
// Rele ACTIVADO = moto habilitada. Conectar el contacto NA (normalmente
// abierto) en serie con la linea de encendido/CDI de la moto.
const bool RELE_ACTIVO_ALTO = true;

// ---------------------- MPU6050 ----------------------
#define LSB_POR_G 16384.0
#define LSB_POR_DPS 131.0

// Muestreo del MPU (ms). 40 ms = 25 lecturas por segundo.
const unsigned long INTERVALO_MPU_MS = 40;
const int MUESTRAS_BASELINE = 30;

// ---------------------- SENSIBILIDAD (UN SOLO AJUSTE) ----------------------
// 1 = BAJA   (casi inmune al viento; hay que mover/levantar de verdad)
// 2 = MEDIA  (recomendada)
// 3 = ALTA   (muy sensible, como la v3)
// Si todavia se dispara con viento -> bajar a 1. Si no detecta cuando la
// levantan o la mueven -> subir a 3. Para afinar, usar DEPURAR_MPU = true.
const int SENSIBILIDAD = 2;

//                                     baja  media  alta
const float TAB_ANGULO[3]    = {      12.0,   8.0,   5.0 };  // grados de inclinacion
const float TAB_ACEL[3]      = {      0.45,  0.30,  0.20 };  // desvio de 1 g
const float TAB_GIRO[3]      = {      60.0,  40.0,  25.0 };  // deg/s
const float TAB_ROTACION[3]  = {      30.0,  20.0,  10.0 };  // grados acumulados
const int   TAB_PULSOS_SW[3] = {         5,     3,     2 };  // pulsos del SW420 en la ventana

// Inclinacion respecto al baseline (grados) y lecturas seguidas necesarias.
const float UMBRAL_ANGULO_GRADOS = TAB_ANGULO[SENSIBILIDAD - 1];
const int   LECT_CONSEC_INCLINACION = 5;      // ~200 ms sostenido

// Desvio de la aceleracion total respecto a 1 g (levantar, tirar, golpear).
const float UMBRAL_ACEL_G = TAB_ACEL[SENSIBILIDAD - 1];
const int   LECT_CONSEC_ACEL = 3;

// Velocidad de giro instantanea (deg/s, ya sin el sesgo del giroscopio).
const float UMBRAL_GIRO_DPS = TAB_GIRO[SENSIBILIDAD - 1];
const int   LECT_CONSEC_GIRO = 3;

// Rotacion ACUMULADA (grados). Detecta girar la moto despacio (cambio de
// direccion). Solo suma cuando el giro supera la zona muerta.
const float UMBRAL_ROTACION_GRADOS = TAB_ROTACION[SENSIBILIDAD - 1];
const float GIRO_ZONA_MUERTA_DPS = 5.0;
const unsigned long REINICIO_ROTACION_MS = 1000;   // quieto este tiempo = se borra lo acumulado

// Un disparo DEBIL (que manda a ALERTA) necesita esta cantidad de lecturas
// seguidas fuera de umbral: filtra rafagas de viento de una sola muestra.
const int LECT_CONSEC_DEBIL = 2;

// SW420: pulsos necesarios dentro de la ventana para considerarlo vibracion
// real (un golpe de viento suele dar 1 pulso suelto). Los pulsos que llegan
// a menos de SW420_REBOTE_MS del anterior se ignoran (rebote del sensor).
const int SW420_PULSOS_MIN = TAB_PULSOS_SW[SENSIBILIDAD - 1];
const unsigned long SW420_VENTANA_MS = 1500;
const unsigned long SW420_REBOTE_MS = 30;

// Ventana de ALERTA (disparos debiles / SW420) y eventos debiles necesarios.
const unsigned long VENTANA_ALERTA_MS = 4000;
const int CONFIRMACIONES_NECESARIAS = 3;

// Imprime lecturas del MPU por Serial cada segundo (util para ajustar).
const bool DEPURAR_MPU = true;

// ---------------------- SENSOR DE PESO ----------------------
const float FACTOR_CALIBRACION = 113.7839;
const unsigned long INTERVALO_EVALUACION_PESO_MS = 300;
const unsigned long INTERVALO_ENVIO_PESO_MS = 2000;
const unsigned long INTERVALO_IMPRIMIR_PESO_MS = 1000;
const unsigned long TIMEOUT_SIN_DATOS_HX711_MS = 3000;
const int EVALUACIONES_PARA_CAMBIAR = 3;
#define MUESTRAS_PROMEDIO_PESO 5

// Por debajo de este peso (g) se considera "asiento vacio": la moto no
// enciende pero NO se manda alerta. Poner 0 para desactivarlo.
const float PESO_ASIENTO_VACIO_G = 20.0;

// El peso incorrecto debe sostenerse este tiempo antes de alertar.
const unsigned long TIEMPO_ALERTA_PESO_MS = 2000;
// Minimo entre mensajes de Telegram por peso incorrecto.
const unsigned long COOLDOWN_TELEGRAM_PESO_MS = 60000;

const bool MOSTRAR_PESO_SERIAL = true;

// ---------------------- INTERVALOS DE RED (tarea aparte) ----------------------
const unsigned long INTERVALO_CONSULTA_COMANDO_MS = 4000;
const unsigned long INTERVALO_CONSULTA_ACTIVO_MS = 4000;
const unsigned long INTERVALO_CONSULTA_RANGO_MS = 5000;
const unsigned long INTERVALO_CONSULTA_TELEGRAM_MS = 15000;
const unsigned long INTERVALO_ENVIO_GPS_MS = 8000;

// ---------------------- TIPOS ----------------------
enum EstadoSistema { REPOSO, ALERTA, MOVIMIENTO_CONFIRMADO };

enum EstadoPeso {
  PESO_INICIANDO = 0,
  PESO_AUTORIZADO,
  PESO_INSUFICIENTE,
  PESO_EXCEDIDO,
  PESO_SIN_REGISTRO,
  PESO_ASIENTO_VACIO,
  PESO_BALANZA_ERROR
};

enum CausaMovimiento {
  CAUSA_NINGUNA = 0,
  CAUSA_INCLINACION,
  CAUSA_ROTACION,
  CAUSA_GIRO,
  CAUSA_SACUDIDA
};

enum TipoEventoRed {
  EV_ESTADO = 0,          // valor = EstadoSistema
  EV_HISTORIAL,           // valor = EstadoSistema
  EV_HISTORIAL_PESO,      // peso  = gramos medidos
  EV_TELEGRAM_MOV,        // valor = CausaMovimiento
  EV_TELEGRAM_PESO        // valor = EstadoPeso, peso = gramos medidos
};

struct EventoRed {
  uint8_t tipo;
  uint8_t valor;
  float peso;
};

// ---------------------- OBJETOS ----------------------
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
HX711 balanza;
QueueHandle_t colaRed = NULL;

// ---------------------- ESTADO COMPARTIDO ENTRE TAREAS ----------------------
volatile bool activoRemoto = true;          // lo escribe la red, lo lee el loop
volatile bool cmdCalibrar = false;
volatile bool cmdTara = false;

volatile float pesoMinG = 0;
volatile float pesoMaxG = 0;
volatile bool hayRangoPeso = false;

volatile float pesoActualG = 0;
volatile uint8_t estadoPeso = PESO_INICIANDO;
volatile bool motoEncendida = false;
volatile bool pesoParaPublicar = true;      // el loop lo pone en true; la red lo atiende

double gpsLat = 0, gpsLng = 0;
volatile bool gpsFixNuevo = false;
portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------- ESTADO DEL LOOP PRINCIPAL ----------------------
EstadoSistema estadoActual = REPOSO;
bool sistemaActivo = true;
bool mpuOk = false;

// Baseline MPU
float baseAx = 0, baseAy = 0, baseAz = 1, baseMag = 1;
float baseGx = 0, baseGy = 0, baseGz = 0;
float baseAngX = 0, baseAngY = 0, baseAngZ = 0;

// Deteccion
int cntInc = 0, cntAcel = 0, cntGiro = 0;
float rotAcumGrados = 0;
unsigned long ultimoGiroMs = 0;
unsigned long ultimaLecturaMPU = 0;
unsigned long ultimoReintentoMPU = 0;
unsigned long ultimaImpresionMPU = 0;
int fallosMPU = 0;

unsigned long inicioVentanaAlerta = 0;
int contadorVibraciones = 0;
int eventosDebiles = 0;
uint8_t causaMovimiento = CAUSA_NINGUNA;
volatile bool vibracionDetectada = false;     // (ya no se usa para decidir; ver pulsosSW420)
volatile uint16_t pulsosSW420 = 0;             // pulsos acumulados desde la ultima lectura
volatile unsigned long ultimoPulsoSW420 = 0;
portMUX_TYPE swMux = portMUX_INITIALIZER_UNLOCKED;
unsigned long inicioVentanaSW = 0;
int pulsosEnVentanaSW = 0;
int cntDebil = 0;

// Peso
bool balanzaLista = false;
bool faltaTara = false;
float bufferPeso[MUESTRAS_PROMEDIO_PESO];
int idxBufferPeso = 0;
int muestrasEnBuffer = 0;
int contadorCambioPeso = 0;
unsigned long ultimaEvaluacionPeso = 0;
unsigned long ultimaImpresionPeso = 0;
unsigned long ultimaMuestraHX = 0;
unsigned long ultimoReintentoHX = 0;
unsigned long inicioEstadoPeso = 0;
bool alertaPesoEnviada = false;

// ---------------------- UTILIDADES DE TEXTO ----------------------
const char* estadoATexto(uint8_t estado) {
  switch (estado) {
    case REPOSO: return "REPOSO";
    case ALERTA: return "ALERTA";
    case MOVIMIENTO_CONFIRMADO: return "MOVIMIENTO_CONFIRMADO";
  }
  return "REPOSO";
}

const char* estadoPesoATexto(uint8_t estado) {
  switch (estado) {
    case PESO_INICIANDO:     return "INICIANDO";
    case PESO_AUTORIZADO:    return "AUTORIZADO";
    case PESO_INSUFICIENTE:  return "PESO_INSUFICIENTE";
    case PESO_EXCEDIDO:      return "PESO_EXCEDIDO";
    case PESO_SIN_REGISTRO:  return "SIN_REGISTRO";
    case PESO_ASIENTO_VACIO: return "ASIENTO_VACIO";
    case PESO_BALANZA_ERROR: return "BALANZA_ERROR";
  }
  return "INICIANDO";
}

const char* causaATexto(uint8_t causa) {
  switch (causa) {
    case CAUSA_INCLINACION: return "inclinacion";
    case CAUSA_ROTACION:    return "cambio de direccion (rotacion)";
    case CAUSA_GIRO:        return "giro brusco";
    case CAUSA_SACUDIDA:    return "sacudida o levantamiento";
  }
  return "vibracion y movimiento";
}

// Pone un evento en la cola de la tarea de red. NUNCA bloquea.
void encolar(uint8_t tipo, uint8_t valor, float peso) {
  if (colaRed == NULL) return;
  EventoRed ev;
  ev.tipo = tipo;
  ev.valor = valor;
  ev.peso = peso;
  if (xQueueSend(colaRed, &ev, 0) != pdTRUE) {
    Serial.println("[Red] Cola llena, evento descartado.");
  }
}

// ============================================================
//  TAREA DE RED (nucleo 0). Aqui viven TODAS las llamadas HTTP.
//  Nada de esto puede frenar la lectura de sensores.
// ============================================================
unsigned long ultimaConsultaComando = 0;
unsigned long ultimaConsultaActivo = 0;
unsigned long ultimaConsultaRango = 0;
unsigned long ultimaConsultaTelegram = 0;
unsigned long ultimoEnvioGPS = 0;
unsigned long ultimoEnvioPeso = 0;
unsigned long ultimoIntentoWiFi = 0;
unsigned long ultimoTelegramPeso = 0;

bool wifiOk() {
  return WiFi.status() == WL_CONNECTED;
}

String urlDisp(const char* sufijo) {
  return String(FIREBASE_BASE_URL) + CODIGO_DISPOSITIVO + sufijo;
}

// Envia PUT / PATCH / POST. Devuelve el codigo HTTP (o negativo si fallo).
int firebaseEnviar(const char* metodo, const String& url, const String& payload) {
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int codigo = http.sendRequest(metodo, payload);
  if (codigo <= 0) {
    Serial.print("[Red] ERROR ");
    Serial.print(metodo);
    Serial.print(": ");
    Serial.println(http.errorToString(codigo));
  }
  http.end();
  return codigo;
}

// GET. Devuelve el codigo HTTP y deja el cuerpo en 'cuerpo'.
int firebaseLeer(const String& url, String &cuerpo) {
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  http.begin(url);
  int codigo = http.GET();
  if (codigo == 200) cuerpo = http.getString();
  http.end();
  return codigo;
}

String limpiarTexto(String s) {
  s.replace("\\", "");
  s.replace("\"", "");
  s.trim();
  return s;
}

bool parsearNumero(String payload, float &valor) {
  payload = limpiarTexto(payload);
  if (payload.length() == 0 || payload.equalsIgnoreCase("null")) return false;
  char c = payload.charAt(0);
  if (!(isDigit(c) || c == '-' || c == '.')) return false;
  valor = payload.toFloat();
  return true;
}

// 1 = leido, 0 = no existe / no es numero, -1 = error de red
int leerCampoNumerico(const char* campo, float &valor) {
  String url = String(FIREBASE_BASE_URL) + CODIGO_DISPOSITIVO + "/" + campo + ".json";
  String cuerpo;
  int codigo = firebaseLeer(url, cuerpo);
  if (codigo != 200) return -1;
  return parsearNumero(cuerpo, valor) ? 1 : 0;
}

void red_asegurarWiFi() {
  if (wifiOk()) return;
  unsigned long ahora = millis();
  if (ahora - ultimoIntentoWiFi < 10000) return;
  ultimoIntentoWiFi = ahora;
  Serial.println("[WiFi] Sin conexion, reintentando...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void red_enviarEstado(uint8_t estado) {
  if (!wifiOk()) { Serial.println("[Firebase] Sin WiFi, estado no enviado."); return; }
  String payload = String("\"") + estadoATexto(estado) + "\"";
  int c = firebaseEnviar("PUT", urlDisp("/estado.json"), payload);
  Serial.print("[Firebase] Estado enviado (");
  Serial.print(estadoATexto(estado));
  Serial.print(") - HTTP ");
  Serial.println(c);
}

void red_guardarHistorial(uint8_t estado) {
  if (!wifiOk()) return;
  String payload = "{\"estado\":\"" + String(estadoATexto(estado)) +
                   "\",\"timestamp\":{\".sv\":\"timestamp\"}}";
  int c = firebaseEnviar("POST", urlDisp("/historial.json"), payload);
  Serial.print("[Historial] ");
  Serial.print(estadoATexto(estado));
  Serial.print(" - HTTP ");
  Serial.println(c);
}

void red_guardarHistorialPeso(float pesoG) {
  if (!wifiOk()) return;
  String payload = "{\"estado\":\"PESO_INCORRECTO\",\"peso\":" + String(pesoG, 1) +
                   ",\"timestamp\":{\".sv\":\"timestamp\"}}";
  int c = firebaseEnviar("POST", urlDisp("/historial.json"), payload);
  Serial.print("[Historial] PESO_INCORRECTO - HTTP ");
  Serial.println(c);
}

void red_enviarTelegram(String mensaje) {
  if (telegramChatId.length() == 0) {
    Serial.println("[Telegram] Todavia no hay chat_id vinculado, no se envia.");
    return;
  }
  if (!wifiOk()) { Serial.println("[Telegram] Sin WiFi."); return; }

  WiFiClientSecure cliente;
  cliente.setInsecure();
  HTTPClient https;
  String url = "https://api.telegram.org/bot" + String(TELEGRAM_BOT_TOKEN) + "/sendMessage";
  if (https.begin(cliente, url)) {
    https.setTimeout(6000);
    https.addHeader("Content-Type", "application/json");
    String payload = "{\"chat_id\":\"" + telegramChatId + "\",\"text\":\"" + mensaje + "\"}";
    int c = https.POST(payload);
    Serial.print("[Telegram] HTTP ");
    Serial.println(c);
    https.end();
  } else {
    Serial.println("[Telegram] ERROR al iniciar HTTPS.");
  }
}

void red_procesarEvento(uint8_t tipo, uint8_t valor, float peso) {
  switch (tipo) {
    case EV_ESTADO:
      red_enviarEstado(valor);
      break;
    case EV_HISTORIAL:
      red_guardarHistorial(valor);
      break;
    case EV_HISTORIAL_PESO:
      red_guardarHistorialPeso(peso);
      break;
    case EV_TELEGRAM_MOV:
      red_enviarTelegram("ALERTA MotoGuard: se detecto movimiento sospechoso en tu moto " +
                         String(CODIGO_DISPOSITIVO) + " (motivo: " + causaATexto(valor) +
                         "). Revisa la app para ver la ubicacion.");
      break;
    case EV_TELEGRAM_PESO: {
      unsigned long ahora = millis();
      if (ultimoTelegramPeso != 0 && ahora - ultimoTelegramPeso < COOLDOWN_TELEGRAM_PESO_MS) break;
      ultimoTelegramPeso = ahora;
      String detalle = (valor == PESO_EXCEDIDO) ? "excede el rango autorizado" : "es menor al rango autorizado";
      red_enviarTelegram("ALERTA MotoGuard: peso NO autorizado en la moto " + String(CODIGO_DISPOSITIVO) +
                         ". Peso medido: " + String(peso, 1) + " g (" + detalle +
                         "). La moto NO enciende.");
      break;
    }
  }
}

// Atiende todos los eventos pendientes (espera hasta 'esperaMs' por el primero).
void red_drenarCola(uint32_t esperaMs) {
  EventoRed ev;
  while (xQueueReceive(colaRed, &ev, pdMS_TO_TICKS(esperaMs)) == pdTRUE) {
    red_procesarEvento(ev.tipo, ev.valor, ev.peso);
    esperaMs = 0;
  }
}

// Publica peso_actual / moto_encendida / estado_peso. Se manda enseguida si
// hubo un cambio (pesoParaPublicar) y, si no, cada INTERVALO_ENVIO_PESO_MS.
void red_publicarPeso() {
  unsigned long ahora = millis();
  bool urgente = pesoParaPublicar;
  if (!urgente && ahora - ultimoEnvioPeso < INTERVALO_ENVIO_PESO_MS) return;
  if (!wifiOk()) return;

  pesoParaPublicar = false;
  ultimoEnvioPeso = ahora;

  float p = pesoActualG;
  bool m = motoEncendida;
  uint8_t e = estadoPeso;

  String payload = "{\"peso_actual\":" + String(p, 1) +
                   ",\"moto_encendida\":" + String(m ? "true" : "false") +
                   ",\"estado_peso\":\"" + String(estadoPesoATexto(e)) + "\"" +
                   ",\"peso_ts\":{\".sv\":\"timestamp\"}}";
  int c = firebaseEnviar("PATCH", urlDisp(".json"), payload);
  if (urgente || c != 200) {
    Serial.print("[Peso] Publicado en Firebase (");
    Serial.print(estadoPesoATexto(e));
    Serial.print(") - HTTP ");
    Serial.println(c);
  }
}

void red_enviarUbicacion() {
  unsigned long ahora = millis();
  if (ahora - ultimoEnvioGPS < INTERVALO_ENVIO_GPS_MS) return;
  if (!gpsFixNuevo) return;
  if (!wifiOk()) return;

  double lat, lng;
  portENTER_CRITICAL(&gpsMux);
  lat = gpsLat;
  lng = gpsLng;
  gpsFixNuevo = false;
  portEXIT_CRITICAL(&gpsMux);

  ultimoEnvioGPS = ahora;
  String payload = "{\"lat\":" + String(lat, 6) + ",\"lng\":" + String(lng, 6) + "}";
  int c = firebaseEnviar("PATCH", urlDisp(".json"), payload);
  Serial.print("[GPS] Ubicacion enviada (");
  Serial.print(lat, 6);
  Serial.print(", ");
  Serial.print(lng, 6);
  Serial.print(") - HTTP ");
  Serial.println(c);
}

void red_revisarComando() {
  unsigned long ahora = millis();
  if (ahora - ultimaConsultaComando < INTERVALO_CONSULTA_COMANDO_MS) return;
  ultimaConsultaComando = ahora;
  if (!wifiOk()) return;

  String cuerpo;
  if (firebaseLeer(urlDisp("/comando.json"), cuerpo) != 200) return;
  String cmd = limpiarTexto(cuerpo);

  if (cmd.equalsIgnoreCase("CALIBRAR")) {
    Serial.println("[Firebase] Comando CALIBRAR recibido desde la app.");
    cmdCalibrar = true;
    firebaseEnviar("PUT", urlDisp("/comando.json"), "null");
  } else if (cmd.equalsIgnoreCase("TARA")) {
    Serial.println("[Firebase] Comando TARA recibido desde la app.");
    cmdTara = true;
    firebaseEnviar("PUT", urlDisp("/comando.json"), "null");
  }
}

void red_revisarActivo() {
  unsigned long ahora = millis();
  if (ahora - ultimaConsultaActivo < INTERVALO_CONSULTA_ACTIVO_MS) return;
  ultimaConsultaActivo = ahora;
  if (!wifiOk()) return;

  String cuerpo;
  if (firebaseLeer(urlDisp("/activo.json"), cuerpo) != 200) return;
  String v = limpiarTexto(cuerpo);
  if (v.equalsIgnoreCase("false")) activoRemoto = false;
  else if (v.equalsIgnoreCase("true")) activoRemoto = true;
}

// Lee peso_min / peso_max (KILOS en Firebase) y los pasa a gramos.
void red_actualizarRango() {
  unsigned long ahora = millis();
  if (ahora - ultimaConsultaRango < INTERVALO_CONSULTA_RANGO_MS) return;
  ultimaConsultaRango = ahora;
  if (!wifiOk()) return;

  float minKg = 0, maxKg = 0;
  int rMin = leerCampoNumerico("peso_min", minKg);
  int rMax = leerCampoNumerico("peso_max", maxKg);
  if (rMin < 0 || rMax < 0) return;   // error de red: se conserva el rango anterior

  if (rMin == 1 && rMax == 1 && maxKg > 0 && maxKg >= minKg) {
    float nuevoMin = minKg * 1000.0;
    float nuevoMax = maxKg * 1000.0;
    if (!hayRangoPeso || nuevoMin != pesoMinG || nuevoMax != pesoMaxG) {
      Serial.print("[Peso] Rango autorizado: ");
      Serial.print(nuevoMin, 1);
      Serial.print(" a ");
      Serial.print(nuevoMax, 1);
      Serial.println(" g");
    }
    pesoMinG = nuevoMin;
    pesoMaxG = nuevoMax;
    hayRangoPeso = true;
  } else {
    hayRangoPeso = false;
  }
}

void red_revisarChatId() {
  unsigned long ahora = millis();
  if (ahora - ultimaConsultaTelegram < INTERVALO_CONSULTA_TELEGRAM_MS) return;
  ultimaConsultaTelegram = ahora;
  if (!wifiOk()) return;

  String cuerpo;
  if (firebaseLeer(urlDisp("/telegram_chat_id.json"), cuerpo) != 200) return;
  String id = limpiarTexto(cuerpo);
  if (id.length() > 0 && !id.equalsIgnoreCase("null")) telegramChatId = id;
}

void tareaRed(void* parametro) {
  for (;;) {
    red_drenarCola(40);          // eventos urgentes primero (espera corta, cede CPU)
    red_asegurarWiFi();
    red_publicarPeso();
    red_drenarCola(0);
    red_enviarUbicacion();
    red_drenarCola(0);
    red_revisarComando();
    red_drenarCola(0);
    red_revisarActivo();
    red_drenarCola(0);
    red_actualizarRango();
    red_drenarCola(0);
    red_revisarChatId();
  }
}

// ============================================================
//  WIFI (arranque)
// ============================================================
void conectarWiFi() {
  Serial.print("Conectando a WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("ERROR: sin WiFi. La tarea de red reintentara sola.");
  }
}

// ============================================================
//  SW420
// ============================================================
void IRAM_ATTR isrVibracion() {
  unsigned long ahora = millis();
  if (ahora - ultimoPulsoSW420 < SW420_REBOTE_MS) return;   // rebote: se ignora
  ultimoPulsoSW420 = ahora;
  portENTER_CRITICAL_ISR(&swMux);
  pulsosSW420++;
  portEXIT_CRITICAL_ISR(&swMux);
}

// Devuelve los pulsos nuevos del SW420 y los pone a cero.
uint16_t tomarPulsosSW420() {
  portENTER_CRITICAL(&swMux);
  uint16_t n = pulsosSW420;
  pulsosSW420 = 0;
  portEXIT_CRITICAL(&swMux);
  return n;
}

// true cuando hubo SW420_PULSOS_MIN pulsos dentro de SW420_VENTANA_MS.
bool vibracionSostenidaSW420(unsigned long ahora, uint16_t nuevos) {
  if (nuevos == 0) {
    if (pulsosEnVentanaSW > 0 && ahora - inicioVentanaSW > SW420_VENTANA_MS) pulsosEnVentanaSW = 0;
    return false;
  }
  if (pulsosEnVentanaSW == 0 || ahora - inicioVentanaSW > SW420_VENTANA_MS) {
    inicioVentanaSW = ahora;
    pulsosEnVentanaSW = 0;
  }
  pulsosEnVentanaSW += nuevos;
  if (pulsosEnVentanaSW >= SW420_PULSOS_MIN) {
    pulsosEnVentanaSW = 0;
    return true;
  }
  return false;
}

// ============================================================
//  MPU6050 (acceso I2C directo)
// ============================================================
int16_t leer16() {
  uint8_t alto = Wire.read();
  uint8_t bajo = Wire.read();
  return (int16_t)((alto << 8) | bajo);
}

// Lee acelerometro (g) y giroscopio (deg/s) en una sola rafaga de 14 bytes.
bool leerMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom(MPU_ADDR, 14, true);
  if (n < 14 || Wire.available() < 14) return false;

  int16_t rax = leer16();
  int16_t ray = leer16();
  int16_t raz = leer16();
  leer16();                 // temperatura (se descarta)
  int16_t rgx = leer16();
  int16_t rgy = leer16();
  int16_t rgz = leer16();

  ax = rax / LSB_POR_G;
  ay = ray / LSB_POR_G;
  az = raz / LSB_POR_G;
  gx = rgx / LSB_POR_DPS;
  gy = rgy / LSB_POR_DPS;
  gz = rgz / LSB_POR_DPS;
  return true;
}

bool escribirRegistroMPU(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registro);
  Wire.write(valor);
  return Wire.endTransmission(true) == 0;
}

bool inicializarMPU6050() {
  if (!escribirRegistroMPU(0x6B, 0x00)) return false;   // salir de sleep
  delay(50);
  escribirRegistroMPU(0x1A, 0x03);                      // filtro pasa-bajos ~44 Hz (menos ruido)
  escribirRegistroMPU(0x1B, 0x00);                      // giroscopio +-250 deg/s
  if (!escribirRegistroMPU(0x1C, 0x00)) return false;   // acelerometro +-2 g

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, 1, true);
  if (Wire.available() < 1) return false;
  Wire.read();
  return true;
}

void calcularAngulos(float ax_g, float ay_g, float az_g,
                     float &angX, float &angY, float &angZ) {
  angX = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
  angY = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  angZ = atan2(sqrt(ax_g * ax_g + ay_g * ay_g), az_g) * 180.0 / PI;
}

void reiniciarDeteccion() {
  cntInc = 0;
  cntAcel = 0;
  cntGiro = 0;
  rotAcumGrados = 0;
  ultimoGiroMs = millis();
  cntDebil = 0;
  eventosDebiles = 0;
  contadorVibraciones = 0;
  vibracionDetectada = false;
  tomarPulsosSW420();
  pulsosEnVentanaSW = 0;
}

// Promedia varias lecturas: gravedad de referencia + sesgo del giroscopio.
void establecerBaseline() {
  Serial.println("Calibrando posicion de reposo... no mover la moto.");

  float sAx = 0, sAy = 0, sAz = 0, sGx = 0, sGy = 0, sGz = 0;
  int validas = 0;

  for (int i = 0; i < MUESTRAS_BASELINE; i++) {
    float ax, ay, az, gx, gy, gz;
    if (leerMPU(ax, ay, az, gx, gy, gz)) {
      sAx += ax; sAy += ay; sAz += az;
      sGx += gx; sGy += gy; sGz += gz;
      validas++;
    }
    delay(40);
  }

  if (validas == 0) {
    Serial.println("ERROR: no se pudo leer el MPU6050 durante la calibracion.");
    return;
  }

  baseAx = sAx / validas; baseAy = sAy / validas; baseAz = sAz / validas;
  baseGx = sGx / validas; baseGy = sGy / validas; baseGz = sGz / validas;
  baseMag = sqrt(baseAx * baseAx + baseAy * baseAy + baseAz * baseAz);
  if (baseMag < 0.5) baseMag = 1.0;

  calcularAngulos(baseAx, baseAy, baseAz, baseAngX, baseAngY, baseAngZ);

  Serial.println("Baseline establecido:");
  Serial.print("  AngX: "); Serial.println(baseAngX, 2);
  Serial.print("  AngY: "); Serial.println(baseAngY, 2);
  Serial.print("  AngZ: "); Serial.println(baseAngZ, 2);
  Serial.print("  Sesgo giroscopio (dps): ");
  Serial.print(baseGx, 2); Serial.print(" ");
  Serial.print(baseGy, 2); Serial.print(" ");
  Serial.println(baseGz, 2);
  Serial.println("Sistema armado en estado REPOSO.");

  reiniciarDeteccion();
}

// ============================================================
//  MAQUINA DE ESTADOS DE MOVIMIENTO
// ============================================================
void cambiarEstado(EstadoSistema nuevo) {
  if (nuevo == estadoActual) return;
  estadoActual = nuevo;

  encolar(EV_ESTADO, nuevo, 0);
  if (nuevo == ALERTA || nuevo == MOVIMIENTO_CONFIRMADO) {
    encolar(EV_HISTORIAL, nuevo, 0);
  }

  switch (nuevo) {
    case REPOSO:
      Serial.println(">>> Estado: REPOSO");
      digitalWrite(LED_ALERTA_PIN, LOW);
      eventosDebiles = 0;
      contadorVibraciones = 0;
      break;

    case ALERTA:
      Serial.println(">>> Estado: ALERTA - movimiento detectado, verificando...");
      digitalWrite(LED_ALERTA_PIN, HIGH);
      inicioVentanaAlerta = millis();
      break;

    case MOVIMIENTO_CONFIRMADO:
      Serial.println("############################################");
      Serial.println("### MOVIMIENTO SOSPECHOSO CONFIRMADO ###");
      Serial.print("### Motivo: ");
      Serial.println(causaATexto(causaMovimiento));
      Serial.println("############################################");
      digitalWrite(LED_ALERTA_PIN, HIGH);
      encolar(EV_TELEGRAM_MOV, causaMovimiento, 0);
      break;
  }
}

void procesarMPU(unsigned long ahora) {
  if (!mpuOk) {
    if (ahora - ultimoReintentoMPU > 3000) {
      ultimoReintentoMPU = ahora;
      if (inicializarMPU6050()) {
        mpuOk = true;
        fallosMPU = 0;
        Serial.println("MPU6050 reconectado.");
        establecerBaseline();
      } else {
        Serial.println("[AVISO] MPU6050 sin respuesta. Revisar SDA/SCL.");
      }
    }
    return;
  }

  if (ahora - ultimaLecturaMPU < INTERVALO_MPU_MS) return;
  float dt = (ahora - ultimaLecturaMPU) / 1000.0;
  if (dt > 0.5) dt = INTERVALO_MPU_MS / 1000.0;
  ultimaLecturaMPU = ahora;

  float ax, ay, az, gx, gy, gz;
  if (!leerMPU(ax, ay, az, gx, gy, gz)) {
    fallosMPU++;
    if (fallosMPU >= 25) {            // ~1 segundo seguido sin leer
      mpuOk = false;
      Serial.println("[AVISO] Lectura I2C fallando, se reintentara iniciar el MPU6050.");
    }
    return;
  }
  fallosMPU = 0;

  // ---- Magnitudes ----
  float mag = sqrt(ax * ax + ay * ay + az * az);

  // Angulo entre la gravedad actual y la del baseline (vale para cualquier montaje).
  float angDesv = 0;
  if (mag > 0.05) {
    float c = (ax * baseAx + ay * baseAy + az * baseAz) / (mag * baseMag);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    angDesv = acos(c) * 180.0 / PI;
  }

  float desvioG = fabs(mag - 1.0);

  float gxn = gx - baseGx;
  float gyn = gy - baseGy;
  float gzn = gz - baseGz;
  float giroDps = sqrt(gxn * gxn + gyn * gyn + gzn * gzn);

  // Rotacion acumulada (cambio de direccion lento).
  if (giroDps > GIRO_ZONA_MUERTA_DPS) {
    rotAcumGrados += giroDps * dt;
    ultimoGiroMs = ahora;
  } else if (ahora - ultimoGiroMs > REINICIO_ROTACION_MS) {
    rotAcumGrados = 0;
  }

  // ---- Condiciones instantaneas y contadores de lecturas seguidas ----
  bool condInc  = angDesv  > UMBRAL_ANGULO_GRADOS;
  bool condAcel = desvioG  > UMBRAL_ACEL_G;
  bool condGiro = giroDps  > UMBRAL_GIRO_DPS;

  cntInc  = condInc  ? cntInc  + 1 : 0;
  cntAcel = condAcel ? cntAcel + 1 : 0;
  cntGiro = condGiro ? cntGiro + 1 : 0;

  // Disparo FUERTE: evidencia sostenida de que la moto se mueve.
  uint8_t causa = CAUSA_NINGUNA;
  if (cntInc >= LECT_CONSEC_INCLINACION)            causa = CAUSA_INCLINACION;
  else if (rotAcumGrados >= UMBRAL_ROTACION_GRADOS) causa = CAUSA_ROTACION;
  else if (cntGiro >= LECT_CONSEC_GIRO)             causa = CAUSA_GIRO;
  else if (cntAcel >= LECT_CONSEC_ACEL)             causa = CAUSA_SACUDIDA;
  bool fuerte = (causa != CAUSA_NINGUNA);

  // Disparo DEBIL: una sola lectura fuera de umbral.
  bool algunaCond = condInc || condAcel || condGiro;
  cntDebil = algunaCond ? cntDebil + 1 : 0;
  bool debil = cntDebil >= LECT_CONSEC_DEBIL;
  uint8_t causaDebil = condInc ? CAUSA_INCLINACION : (condGiro ? CAUSA_GIRO : CAUSA_SACUDIDA);

  if (DEPURAR_MPU && ahora - ultimaImpresionMPU >= 1000) {
    ultimaImpresionMPU = ahora;
    Serial.print("[MPU] inclin=");  Serial.print(angDesv, 1);
    Serial.print(" deg | desvioG="); Serial.print(desvioG, 3);
    Serial.print(" | giro=");        Serial.print(giroDps, 1);
    Serial.print(" dps | rotAcum="); Serial.print(rotAcumGrados, 1);
    Serial.print(" deg | estado=");  Serial.println(estadoATexto(estadoActual));
  }

  // Pulsos del SW420 desde la ultima vuelta (se leen siempre, para no acumular).
  uint16_t nuevosPulsos = tomarPulsosSW420();
  bool swSostenido = vibracionSostenidaSW420(ahora, nuevosPulsos);

  switch (estadoActual) {

    case REPOSO:
      if (fuerte) {
        causaMovimiento = causa;
        Serial.print("[Disparo] MPU6050 FUERTE (sin SW420): ");
        Serial.println(causaATexto(causa));
        cambiarEstado(MOVIMIENTO_CONFIRMADO);
      }
      else if (swSostenido) {
        contadorVibraciones = SW420_PULSOS_MIN;
        eventosDebiles = 0;
        Serial.println("[Disparo] SW420 (vibracion sostenida)");
        cambiarEstado(ALERTA);
      }
      else if (debil) {
        eventosDebiles = 1;
        contadorVibraciones = 0;
        Serial.print("[Disparo] MPU6050 debil: ");
        Serial.println(causaATexto(causaDebil));
        cambiarEstado(ALERTA);
      }
      break;

    case ALERTA:
      contadorVibraciones += nuevosPulsos;

      if (fuerte) {
        causaMovimiento = causa;
        cambiarEstado(MOVIMIENTO_CONFIRMADO);
        break;
      }

      if (debil) {
        eventosDebiles++;
        Serial.print("Confirmando movimiento (");
        Serial.print(eventosDebiles);
        Serial.print("/");
        Serial.print(CONFIRMACIONES_NECESARIAS);
        Serial.print(") inclin="); Serial.print(angDesv, 1);
        Serial.print(" desvioG="); Serial.print(desvioG, 3);
        Serial.print(" giro=");    Serial.println(giroDps, 1);
      }

      if (eventosDebiles >= CONFIRMACIONES_NECESARIAS) {
        causaMovimiento = debil ? causaDebil : (uint8_t)CAUSA_NINGUNA;
        cambiarEstado(MOVIMIENTO_CONFIRMADO);
      }
      else if (ahora - inicioVentanaAlerta > VENTANA_ALERTA_MS) {
        if (contadorVibraciones >= SW420_PULSOS_MIN) {
          Serial.println("Vibraciones del SW420 repetidas, pero el MPU no confirmo movimiento. Falso positivo. Vuelve a REPOSO.");
        } else {
          Serial.println("Deteccion aislada sin confirmacion. Vuelve a REPOSO.");
        }
        cambiarEstado(REPOSO);
      }
      break;

    case MOVIMIENTO_CONFIRMADO:
      // Permanece hasta REARMAR (Serial) o CALIBRAR (app).
      break;
  }
}

// ============================================================
//  SENSOR DE PESO
// ============================================================
void actualizarRele() {
  bool nivelAlto = motoEncendida ? RELE_ACTIVO_ALTO : !RELE_ACTIVO_ALTO;
  digitalWrite(RELE_MOTO_PIN, nivelAlto ? HIGH : LOW);
}

bool iniciarBalanza() {
  balanza.begin(HX711_DT_PIN, HX711_SCK_PIN);
  balanza.set_scale(FACTOR_CALIBRACION);
  return balanza.wait_ready_timeout(2000);
}

void tararBalanza() {
  if (!balanzaLista) {
    Serial.println("[Peso] Balanza no disponible, no se puede tarar.");
    return;
  }
  balanza.tare(10);
  muestrasEnBuffer = 0;
  idxBufferPeso = 0;
  contadorCambioPeso = 0;
  faltaTara = false;
  ultimaMuestraHX = millis();
  Serial.println("[Peso] Tara realizada (asiento vacio = 0 g).");
}

// Aplica un nuevo estado de peso: rele, mensajes y aviso a la app.
void aplicarEstadoPeso(uint8_t nuevo) {
  estadoPeso = nuevo;
  motoEncendida = (nuevo == PESO_AUTORIZADO);
  inicioEstadoPeso = millis();
  alertaPesoEnviada = false;
  contadorCambioPeso = 0;
  actualizarRele();
  pesoParaPublicar = true;

  float p = pesoActualG;
  switch (nuevo) {
    case PESO_AUTORIZADO:
      Serial.print(">>> PESO CORRECTO: "); Serial.print(p, 1);
      Serial.print(" g (rango "); Serial.print(pesoMinG, 1);
      Serial.print(" a "); Serial.print(pesoMaxG, 1);
      Serial.println(" g) -> MOTO HABILITADA");
      break;
    case PESO_INSUFICIENTE:
    case PESO_EXCEDIDO:
      Serial.print(">>> PESO INCORRECTO: "); Serial.print(p, 1);
      Serial.print(" g (rango "); Serial.print(pesoMinG, 1);
      Serial.print(" a "); Serial.print(pesoMaxG, 1);
      Serial.println(nuevo == PESO_EXCEDIDO ? " g) -> excede el maximo. MOTO BLOQUEADA"
                                            : " g) -> menor al minimo. MOTO BLOQUEADA");
      break;
    case PESO_ASIENTO_VACIO:
      Serial.println(">>> Asiento vacio -> MOTO BLOQUEADA");
      break;
    case PESO_SIN_REGISTRO:
      Serial.println(">>> Sin rango de peso guardado en la app -> MOTO BLOQUEADA");
      break;
    case PESO_BALANZA_ERROR:
      Serial.println(">>> BALANZA SIN DATOS -> MOTO BLOQUEADA");
      break;
  }
}

uint8_t clasificarPeso(float p) {
  if (!hayRangoPeso) return PESO_SIN_REGISTRO;

  float minG = pesoMinG;
  float maxG = pesoMaxG;
  float umbralVacio = PESO_ASIENTO_VACIO_G;
  if (umbralVacio > minG) umbralVacio = minG;   // si el minimo autorizado es muy bajo, manda el minimo

  if (PESO_ASIENTO_VACIO_G > 0 && p < umbralVacio) return PESO_ASIENTO_VACIO;
  if (p < minG) return PESO_INSUFICIENTE;
  if (p > maxG) return PESO_EXCEDIDO;
  return PESO_AUTORIZADO;
}

// Toma UNA muestra cuando el HX711 la tiene lista (no bloquea).
void leerBalanza() {
  unsigned long ahora = millis();

  // Si la balanza no arranco, se reintenta cada 5 s (sin tarar sola: hay que
  // mandar TARA con el asiento vacio).
  if (!balanzaLista) {
    if (ahora - ultimoReintentoHX > 5000) {
      ultimoReintentoHX = ahora;
      if (iniciarBalanza()) {
        balanzaLista = true;
        faltaTara = true;
        ultimaMuestraHX = ahora;
        Serial.println("[Peso] HX711 detectado. Enviar 'TARA' (Serial o app) con el asiento VACIO.");
      } else {
        Serial.println("[Peso] HX711 no responde. Revisar DT(32)/SCK(33)/alimentacion.");
        if (estadoPeso != PESO_BALANZA_ERROR) aplicarEstadoPeso(PESO_BALANZA_ERROR);
      }
    }
    return;
  }

  if (faltaTara) return;

  if (balanza.is_ready()) {
    float v = balanza.get_units(1);
    ultimaMuestraHX = ahora;
    if (v != v || fabs(v) > 1.0e9) return;   // descarta NaN / valores absurdos
    bufferPeso[idxBufferPeso] = v;
    idxBufferPeso = (idxBufferPeso + 1) % MUESTRAS_PROMEDIO_PESO;
    if (muestrasEnBuffer < MUESTRAS_PROMEDIO_PESO) muestrasEnBuffer++;
  } else if (ahora - ultimaMuestraHX > TIMEOUT_SIN_DATOS_HX711_MS) {
    // El HX711 dejo de entregar datos: avisar y reintentar despertarlo.
    if (ahora - ultimoReintentoHX > 5000) {
      ultimoReintentoHX = ahora;
      Serial.println("[Peso] HX711 sin datos. Reintentando...");
      balanza.power_down();
      delay(10);
      balanza.power_up();
    }
    if (estadoPeso != PESO_BALANZA_ERROR) aplicarEstadoPeso(PESO_BALANZA_ERROR);
  }
}

// Calcula el peso (promedio) y decide si la moto puede encenderse.
void evaluarPeso() {
  unsigned long ahora = millis();
  if (ahora - ultimaEvaluacionPeso < INTERVALO_EVALUACION_PESO_MS) return;
  ultimaEvaluacionPeso = ahora;

  if (!balanzaLista || faltaTara || muestrasEnBuffer == 0) return;

  float suma = 0;
  for (int i = 0; i < muestrasEnBuffer; i++) suma += bufferPeso[i];
  float promedio = suma / muestrasEnBuffer;
  if (promedio < 0) promedio = 0;
  pesoActualG = promedio;

  if (muestrasEnBuffer < MUESTRAS_PROMEDIO_PESO) return;   // todavia no hay suficientes para decidir

  uint8_t nuevo = clasificarPeso(promedio);

  if (nuevo == estadoPeso) {
    contadorCambioPeso = 0;
  } else {
    // La primera decision (o salir de un error) es inmediata; despues se
    // exige que se repita para evitar parpadeos cerca del borde del rango.
    bool inmediata = (estadoPeso == PESO_INICIANDO || estadoPeso == PESO_BALANZA_ERROR);
    contadorCambioPeso++;
    if (inmediata || contadorCambioPeso >= EVALUACIONES_PARA_CAMBIAR) {
      aplicarEstadoPeso(nuevo);
    }
  }

  // Alerta por peso incorrecto (solo si se sostiene unos segundos).
  if ((estadoPeso == PESO_INSUFICIENTE || estadoPeso == PESO_EXCEDIDO) &&
      !alertaPesoEnviada && ahora - inicioEstadoPeso >= TIEMPO_ALERTA_PESO_MS) {
    alertaPesoEnviada = true;
    Serial.println("### ALERTA: PESO NO AUTORIZADO - la moto no enciende ###");
    encolar(EV_HISTORIAL_PESO, 0, pesoActualG);
    encolar(EV_TELEGRAM_PESO, estadoPeso, pesoActualG);
  }
}

// Muestra el peso por Serial cada segundo.
void imprimirPeso() {
  if (!MOSTRAR_PESO_SERIAL) return;
  unsigned long ahora = millis();
  if (ahora - ultimaImpresionPeso < INTERVALO_IMPRIMIR_PESO_MS) return;
  ultimaImpresionPeso = ahora;

  if (!balanzaLista) {
    Serial.println("[Peso] HX711 no inicializado.");
    return;
  }
  if (faltaTara) {
    Serial.println("[Peso] Falta TARA (enviar 'TARA' con el asiento vacio).");
    return;
  }
  if (muestrasEnBuffer == 0) {
    Serial.println("[Peso] Esperando primeras lecturas del HX711...");
    return;
  }

  Serial.print("[Peso] "); Serial.print(pesoActualG, 1); Serial.print(" g | ");
  if (hayRangoPeso) {
    Serial.print("rango "); Serial.print(pesoMinG, 1);
    Serial.print("-"); Serial.print(pesoMaxG, 1); Serial.print(" g | ");
  } else {
    Serial.print("sin rango | ");
  }
  Serial.print(estadoPesoATexto(estadoPeso));
  Serial.println(motoEncendida ? " | moto HABILITADA" : " | moto BLOQUEADA");
}

// ============================================================
//  GPS
// ============================================================
void leerGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid() && gps.location.isUpdated()) {
    portENTER_CRITICAL(&gpsMux);
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
    gpsFixNuevo = true;
    portEXIT_CRITICAL(&gpsMux);
  }
}

// ============================================================
//  COMANDOS (Serial y app)
// ============================================================
void procesarSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.equalsIgnoreCase("REARMAR")) {
    Serial.println("Sistema rearmado por el usuario.");
    establecerBaseline();
    cambiarEstado(REPOSO);
  } else if (cmd.equalsIgnoreCase("TARA")) {
    tararBalanza();
  }
}

void procesarComandosRemotos() {
  if (cmdCalibrar) {
    cmdCalibrar = false;
    establecerBaseline();
    cambiarEstado(REPOSO);
  }

  if (cmdTara) {
    cmdTara = false;
    tararBalanza();
  }

  bool activoNuevo = activoRemoto;
  if (activoNuevo != sistemaActivo) {
    sistemaActivo = activoNuevo;
    if (sistemaActivo) {
      Serial.println("[Sistema] ARMADO desde la app. Recalibrando y volviendo a REPOSO.");
      establecerBaseline();
      cambiarEstado(REPOSO);
    } else {
      Serial.println("[Sistema] DESARMADO desde la app. Ignorando sensores de movimiento.");
      digitalWrite(LED_ALERTA_PIN, LOW);
      encolar(EV_ESTADO, REPOSO, 0);
    }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("================================================");
  Serial.println("MotoGuard V9 PESOv4");
  Serial.println("================================================");

  pinMode(SW420_PIN, INPUT);
  pinMode(LED_ALERTA_PIN, OUTPUT);
  digitalWrite(LED_ALERTA_PIN, LOW);

  // La moto arranca BLOQUEADA hasta que el peso sea correcto.
  pinMode(RELE_MOTO_PIN, OUTPUT);
  motoEncendida = false;
  actualizarRele();

  conectarWiFi();

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS inicializado. Esperando senal...");

  attachInterrupt(digitalPinToInterrupt(SW420_PIN), isrVibracion, RISING);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (inicializarMPU6050()) {
    mpuOk = true;
    Serial.println("MPU6050 conectado correctamente.");
    establecerBaseline();
  } else {
    mpuOk = false;
    Serial.println("ERROR: MPU6050 no encontrado. Revisar SDA/SCL. Se reintentara solo.");
  }

  // Balanza: el asiento debe estar VACIO al encender.
  if (iniciarBalanza()) {
    balanzaLista = true;
    ultimaMuestraHX = millis();
    tararBalanza();
    Serial.println("HX711 listo (factor " + String(FACTOR_CALIBRACION, 4) + ").");
  } else {
    balanzaLista = false;
    estadoPeso = PESO_BALANZA_ERROR;
    Serial.println("ERROR: HX711 no responde. Revisar DT/SCK y alimentacion.");
  }

  // Tarea de red en el nucleo 0 (el loop corre en el nucleo 1).
  colaRed = xQueueCreate(20, sizeof(EventoRed));
  xTaskCreatePinnedToCore(tareaRed, "red", 16384, NULL, 1, NULL, 0);

  encolar(EV_ESTADO, REPOSO, 0);
  pesoParaPublicar = true;

  Serial.println("Sistema listo. Comandos por Serial: REARMAR | TARA");
  Serial.println("================================================");
}

// ============================================================
//  LOOP (nunca se bloquea por la red)
// ============================================================
void loop() {
  unsigned long ahora = millis();

  leerGPS();
  procesarSerial();
  procesarComandosRemotos();

  // Peso: siempre activo, aunque la alarma de movimiento este desarmada.
  leerBalanza();
  evaluarPeso();
  imprimirPeso();

  if (sistemaActivo) {
    procesarMPU(ahora);
  }

  delay(1);
}
