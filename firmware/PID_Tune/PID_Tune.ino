#include <Arduino.h>

// --- PINES ---
const int IN1 = 32;
const int IN2 = 33;
const int PIN_A = 25;
const int PIN_B = 26;

// --- CONFIGURACIÓN PWM ---
const int frecuencia = 1000;
const int resol = 10;      // 0-1023
const double MAX_PWM = 1023.0;

// --- TIEMPO DE MUESTREO ---
const float DELTA_T_S = 0.01; // 10ms

// --- VARIABLES PID ---
double Kp = 3.2;  
double Ki = 0.0; 
double Kd = 0.0; 

// Variables de estado
double referencia_posicion = 0.0;
double actuacion = 0.0;
double error_anterior = 0.0;
double suma_error = 0.0;

// --- VARIABLES ENCODER Y TIMER ---
volatile long contador = 0;
long contador_anterior = 0;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- INTERRUPCIONES ENCODER ---
void IRAM_ATTR manejador_interrupcion_A() {
  int a = digitalRead(PIN_A);
  int b = digitalRead(PIN_B);
  if (a == b) contador--; else contador++;
}

void IRAM_ATTR manejador_interrupcion_B() {
  int a = digitalRead(PIN_A);
  int b = digitalRead(PIN_B);
  if (a != b) contador--; else contador++;
}

// --- ACTUACIÓN MOTOR ---
void Ir_CW(int pwm_valor) {
  ledcWrite(IN1, pwm_valor);
  ledcWrite(IN2, 0);
}

void Ir_NCW(int pwm_valor) {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, pwm_valor);
}

void Parar_Motor() {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
}

void P3_actua_Segura(double actuacion_valor) {
  if (actuacion_valor > MAX_PWM) actuacion_valor = MAX_PWM;
  if (actuacion_valor < -MAX_PWM) actuacion_valor = -MAX_PWM;

  if (actuacion_valor > 0) Ir_CW((int)actuacion_valor);
  else if (actuacion_valor < 0) Ir_NCW((int)(-actuacion_valor));
  else Parar_Motor();
}

// --- ALGORITMO PID ---
void P3_PID_Segura(double referencia, double valor_actual) {
  double error = referencia - valor_actual;

  if (Ki > 0) {
    suma_error += error;
    
    // Anti-windup
    double max_suma = MAX_PWM / Ki;
    
    if (suma_error > max_suma) suma_error = max_suma;
    if (suma_error < -max_suma) suma_error = -max_suma;
  } else {
    suma_error = 0;
  }

  actuacion = (Kp * error) + (Ki * suma_error) + (Kd * (error - error_anterior));

  error_anterior = error;
}

// --- INTERRUPCIÓN TIMER ---
void IRAM_ATTR ISR_Control_PID() {
  long pos_actual;
  portENTER_CRITICAL_ISR(&timerMux);
  pos_actual = contador;
  portEXIT_CRITICAL_ISR(&timerMux);

  P3_PID_Segura(referencia_posicion, (double)pos_actual);
  P3_actua_Segura(actuacion);
}

void setup() {
  Serial.begin(115200);

  // PWM
  ledcAttach(IN1, frecuencia, resol);
  ledcAttach(IN2, frecuencia, resol);

  // Encoder
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), manejador_interrupcion_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), manejador_interrupcion_B, CHANGE);

  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &ISR_Control_PID);
  timerAlarm(timer, 10000, true, 0); // 10ms

  Serial.println("MODO DIRECTO: Kp, Ki, Kd.");
  Serial.println("Ejemplos: 'P 1.5', 'I 0.05', 'D 2.0' o '1000' (Ref)");
  Serial.println("Referencia,Posicion"); 
}

void loop() {
  // --- LECTURA COMANDOS ---
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim(); 

    if (entrada.length() > 0) {
      char comando = toupper(entrada.charAt(0)); 

      if (comando == 'P' || comando == 'I' || comando == 'D') {
        double valor = entrada.substring(1).toDouble();

        if (comando == 'P') Kp = valor;
        if (comando == 'I') Ki = valor;
        if (comando == 'D') Kd = valor;

        Serial.print(">> SET PID | Kp:"); Serial.print(Kp);
        Serial.print(" Ki:"); Serial.print(Ki);
        Serial.print(" Kd:"); Serial.println(Kd);

      } else {
        referencia_posicion = entrada.toDouble();
      
        // Reset para estabilidad en el nuevo salto
        portENTER_CRITICAL(&timerMux);
        suma_error = 0.0;
        error_anterior = 0.0;
        portEXIT_CRITICAL(&timerMux);
      }
    }
  }

  // --- TELEMETRÍA (Excel) ---
  static unsigned long last_print = 0;
  if (millis() - last_print >= 50) {
    last_print = millis();
    
    long pos_print;
    portENTER_CRITICAL(&timerMux);
    pos_print = contador;
    portEXIT_CRITICAL(&timerMux);

    // CSV
    Serial.print(referencia_posicion);
    Serial.print(",");
    Serial.println(pos_print);
  }
}