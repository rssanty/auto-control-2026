#include <SoftwareSerial.h>
#include <string.h>

//variables de configuracion

const char ID_ROBOT[] = "01";
const char NOMBRE_BT[] = "robot4";
const char PIN_BT[] = "1234";
const bool CONFIGURAR_BT = false;
const long BAUD_BT = 9600;
int velocidad = 255;
SoftwareSerial BT(2, 3);

const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;

const unsigned long TIMEOUT_BT = 1000;
unsigned long ultimoComando = 0;
bool moviendo = false;
bool leyendoVelocidad = false;
int velTemp = 0;
byte velDigitos = 0;
int acel = 0;
int giro = 0;
const float FACTOR_GIRO = 0.35; //1.0
const bool INVERTIR_AVANCE = true;
const bool INVERTIR_PIVOTE = false;

//----------------------------------------------------//

bool enviarAT(const char* comando,
              const char* respuestaEsperada) {

  while (BT.available()) BT.read();

  BT.print(comando);

  char respuesta[48] = {0};
  byte posicion = 0;

  unsigned long inicio = millis();

  while (millis() - inicio < 1200) {

    while (BT.available()) {
      char c = BT.read();

      if (posicion < sizeof(respuesta) - 1) {
        respuesta[posicion++] = c;
        respuesta[posicion] = '\0';
      }
    }

    if (strstr(respuesta, respuestaEsperada)) {
      Serial.println(respuesta);
      return true;
    }
  }

  Serial.print("Respuesta: ");
  Serial.println(respuesta);

  return false;
}

void configurarBluetooth() {

  Serial.println("Configurando HC-06...");

  if (!enviarAT("AT", "OK")) {
    Serial.println("ERROR: HC-06 no responde");
    Serial.println("Revisar conexion y baudios");
    return;
  }

  char comando[40];

  snprintf(comando, sizeof(comando),
           "AT+NAME%s", NOMBRE_BT);

  if (enviarAT(comando, "OKsetname")) {
    Serial.println("Nombre configurado");
  } else {
    Serial.println("Error al configurar nombre");
  }

  delay(500);

  snprintf(comando, sizeof(comando),
           "AT+PIN%s", PIN_BT);

  if (enviarAT(comando, "OKsetPIN")) {
    Serial.println("PIN configurado");
  } else {
    Serial.println("Error al configurar PIN");
  }
}

void setup() {

  Serial.begin(9600);
  BT.begin(BAUD_BT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  detener();

  Serial.print("Iniciando robot ");
  Serial.println(ID_ROBOT);

  delay(2000);

  if (CONFIGURAR_BT) {
    configurarBluetooth();
  }

  Serial.println("Robot listo");
}

void loop() {

  while (BT.available()) {

    char comando = BT.read();

    if (comando >= 'a' && comando <= 'z') {
      comando -= 32;
    }

    if (leyendoVelocidad) {
      if (comando >= '0' && comando <= '9' && velDigitos < 3) {
        velTemp = velTemp * 10 + (comando - '0');
        velDigitos++;
        continue;
      }
      velocidad = constrain(velTemp, 0, 255);
      leyendoVelocidad = false;
    }
    if (comando == 'V') {
      leyendoVelocidad = true;
      velTemp = 0;
      velDigitos = 0;
      continue;
    }

    bool esMovimiento = true;

    switch (comando) {

      case 'F': acel = 1;  giro = 0;  break;
      case 'B': acel = -1; giro = 0;  break;
      case 'L': acel = 0;  giro = -1; break;
      case 'R': acel = 0;  giro = 1;  break;

      case 'G': acel = 1;  giro = -1; break;
      case 'I': acel = 1;  giro = 1;  break;
      case 'H': acel = -1; giro = -1; break;
      case 'J': acel = -1; giro = 1;  break;

      case 'S': acel = 0;  giro = 0;  break;

      case 'X': giro = 0; break;
      case 'Y': acel = 0; break;

      case '1': velocidad = 100; esMovimiento = false; break;
      case '2': velocidad = 150; esMovimiento = false; break;
      case '3': velocidad = 200; esMovimiento = false; break;

      default: esMovimiento = false; break;
    }

    if (esMovimiento) {
      ultimoComando = millis();
    }
    aplicarMovimiento();
  }

  if (TIMEOUT_BT > 0 && moviendo && millis() - ultimoComando > TIMEOUT_BT) {
    acel = 0;
    giro = 0;
    detener();
  }
}

void motorIzquierdo(int v) {
  digitalWrite(IN1, v > 0 ? HIGH : LOW);
  digitalWrite(IN2, v < 0 ? HIGH : LOW);
  analogWrite(ENA, abs(v));
}

void motorDerecho(int v) {
  digitalWrite(IN3, v > 0 ? HIGH : LOW);
  digitalWrite(IN4, v < 0 ? HIGH : LOW);
  analogWrite(ENB, abs(v));
}

void aplicarMovimiento() {

  if (acel == 0 && giro == 0) {
    detener();
    return;
  }

  int izq, der;

  if (acel == 0) {

    izq = (giro > 0) ? velocidad : -velocidad;
    der = -izq;
    if (INVERTIR_PIVOTE) { izq = -izq; der = -der; }
  } else {

    int base = (INVERTIR_AVANCE ? -acel : acel) * velocidad;
    int reducida = base * FACTOR_GIRO;
    izq = (giro < 0) ? reducida : base;
    der = (giro > 0) ? reducida : base;
  }

  motorIzquierdo(izq);
  motorDerecho(der);
  moviendo = true;
}

void detener() {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  acel = 0;
  giro = 0;
  moviendo = false;
}
