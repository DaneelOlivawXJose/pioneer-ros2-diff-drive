#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <WiFi.h>

#if !defined(ESP32)
#error This example is only available for ESP32
#endif

// ==========================================
//   CONFIGURACIÓN
// ==========================================

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Publicadores
rcl_publisher_t publisher; // Odometría (ticks)
rcl_publisher_t publisher_laser; // Sensor Distancia (MultiArray)
rcl_publisher_t pub_pos_estimada;
std_msgs__msg__Int32MultiArray encoder_msg;
std_msgs__msg__Float32MultiArray laser_msg; // Mensaje de 3 posiciones
geometry_msgs__msg__Point pos_msg;

// Suscriptor
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;

rcl_subscription_t subscriber_settings;
std_msgs__msg__Float32MultiArray msg_sub_settings;

rcl_subscription_t subscriber_reset;
std_msgs__msg__Int32 msg_reset;

// PINES
#define LED_PIN 2

// --- VARIABLES DE ESTADO ---
volatile bool sistema_seguro = false;
volatile bool obstaculo_delante = false;

const int frecuencia = 25000;

// --- MOTORES ---
const int IN1_L = 32; const int IN2_L = 33;
const int PIN_A_L = 25; const int PIN_B_L = 26; 
const int IN1_R = 27; const int IN2_R = 14;
const int PIN_A_R = 12; const int PIN_B_R = 13; 

// --- CINEMÁTICA Y PID ---
const double RADIO_RUEDA = 0.0347;
const double ANCHO_ROBOT = 0.1941;
const float DELTA_T_S = 0.01; // 10ms
const float CUENTAS_POR_REV_TOTAL = 1564.65;
const float FACTOR_CUENTAS_A_RADS = (2.0 * 3.14159265) / CUENTAS_POR_REV_TOTAL;

double pos_x_global = 0.0;
double pos_y_global = 0.0;
double theta_actual = 0.0;
long sl_prev = 0;
long sr_prev = 0;

// PID CONSTANTES
const double Kp_L = 40.289; const double Ki_L = 1120.3135; const double Kd_L = 0.0;
const double Kp_R = 19.6078; const double Ki_R = 586.746; const double Kd_R = 0.0;

// Variables PID
volatile double target_L = 0.0;
volatile double target_R = 0.0;

volatile long contador_L = 0;
long contador_L_ant = 0;
double error_acumulado_L = 0.0;
double error_anterior_L = 0.0;

volatile long contador_R = 0;
long contador_R_ant = 0;
double error_acumulado_R = 0.0;
double error_anterior_R = 0.0;

// Hardware Timers y Mux
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE paramMux = portMUX_INITIALIZER_UNLOCKED;

#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    Serial.print("Fallo en inicialización de ROS. Código de error: "); \
    Serial.println((int)temp_rc); \
    Serial.println("Reiniciando ESP32 en 3 segundos..."); \
    delay(3000); \
    ESP.restart(); \
  } \
}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.print("MicroROS Error Code: "); Serial.println((int)temp_rc); }}

// ==========================================
//   CONFIGURACIÓN DE SENSORES US-016
// ==========================================
const int NUM_MUESTRAS = 7;
const float MULTIPLICADOR_RC = 1.5; // Divisor de tensión (3/2)
const float alfa = 0.3;

struct US016Sensor {
  uint8_t pin;
  float lecturas[NUM_MUESTRAS];
  float voltaje_mV;
  float voltaje_mV_anterior_suavizado;
  float voltaje_mV_minimo;
};

// Instanciamos los 3 sensores
US016Sensor sensores[3] = {
  {36, {0}, 0.0, 0.0, 100.0},
  {39, {0}, 0.0, 0.0, 100.0},
  {34, {0}, 0.0, 0.0, 100.0}
};

float obtenerMediana(float array[], int size)
{
  float tempArray[size];
  memcpy(tempArray, array, size * sizeof(float));
  
  for(int i = 0; i < (size - 1); i++) {
    for(int j = 0; j < (size - (i + 1)); j++) {
      if(tempArray[j] > tempArray[j+1]) {
        float t = tempArray[j];
        tempArray[j] = tempArray[j+1];
        tempArray[j+1] = t;
      }
    }
  }
  return tempArray[size / 2];
}

void inicializarSensor(US016Sensor &sensor)
{
  for (int i = 0; i < NUM_MUESTRAS; i++) {
    sensor.lecturas[i] = analogReadMilliVolts(sensor.pin) * MULTIPLICADOR_RC;
    delay(20);
  }
  float medianaInicial = obtenerMediana(sensor.lecturas, NUM_MUESTRAS);
  sensor.voltaje_mV = medianaInicial;
  sensor.voltaje_mV_anterior_suavizado = medianaInicial;
}



// Tarea en segundo plano para leer y filtrar
void tareaLecturaUS016(void *pvParameters)
{
  for(;;)
  {
    for (int s = 0; s < 3; s++)
    {
      for (int i = 0; i < NUM_MUESTRAS - 1; i++)
      {
        sensores[s].lecturas[i] = sensores[s].lecturas[i + 1];
      }

      float nuevaLectura_mV = 0.0;
      if (sensores[s].pin != 34)
      {
        nuevaLectura_mV = analogReadMilliVolts(sensores[s].pin) * MULTIPLICADOR_RC;
      } else {
        nuevaLectura_mV = analogReadMilliVolts(sensores[s].pin);
      }
      
      sensores[s].lecturas[NUM_MUESTRAS - 1] = nuevaLectura_mV;

      float mediana = obtenerMediana(sensores[s].lecturas, NUM_MUESTRAS);

      sensores[s].voltaje_mV = alfa * mediana + (1 - alfa) * sensores[s].voltaje_mV_anterior_suavizado;
      sensores[s].voltaje_mV_anterior_suavizado = sensores[s].voltaje_mV;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==========================================
//   FUNCIONES BÁSICAS Y RUTINAS
// ==========================================
void pararMotores()
{
  ledcWrite(IN1_L, 0); ledcWrite(IN2_L, 0);
  ledcWrite(IN1_R, 0); ledcWrite(IN2_R, 0);
}

void error_loop() {
  Serial.println("!!! ERROR FATAL !!!");
  pararMotores();
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200);
  }
}

void IRAM_ATTR isr_L_A() { if(digitalRead(PIN_A_L)==digitalRead(PIN_B_L)) contador_L--; else contador_L++; }
void IRAM_ATTR isr_L_B() { if(digitalRead(PIN_A_L)!=digitalRead(PIN_B_L)) contador_L--; else contador_L++; }
void IRAM_ATTR isr_R_A() { if(digitalRead(PIN_A_R)==digitalRead(PIN_B_R)) contador_R++; else contador_R--; } 
void IRAM_ATTR isr_R_B() { if(digitalRead(PIN_A_R)!=digitalRead(PIN_B_R)) contador_R++; else contador_R--; }

void setMotorL(int pwm) {
  if (pwm > 1023) pwm = 1023; if (pwm < -1023) pwm = -1023;
  if (pwm >= 0) { ledcWrite(IN1_L, pwm); ledcWrite(IN2_L, 0); }
  else { ledcWrite(IN1_L, 0); ledcWrite(IN2_L, -pwm); }
}

void setMotorR(int pwm) {
  if (pwm > 1023) pwm = 1023; if (pwm < -1023) pwm = -1023;
  if (pwm >= 0) { ledcWrite(IN1_R, pwm); ledcWrite(IN2_R, 0); }
  else { ledcWrite(IN1_R, 0); ledcWrite(IN2_R, -pwm); }
}

double calcularPID(double ref, double medicion, double &err_acum, double &err_prev, double Kp, double Ki, double Kd) {
  double error = ref - medicion;
  err_acum += error * DELTA_T_S;
  
  double max_I = 1023.0; 
  if (Ki > 0) max_I = 1023.0 / Ki;
  if (err_acum > max_I) err_acum = max_I;
  if (err_acum < -max_I) err_acum = -max_I;

  err_prev = error;
  return (Kp * error) + (Ki * err_acum);
}

void IRAM_ATTR onTimerPID() {
  bool intentando_avanzar = (target_L > 0 && target_R > 0);

  if (!sistema_seguro || (obstaculo_delante && intentando_avanzar)) {
    pararMotores();
    error_acumulado_L=0; error_anterior_L=0;
    error_acumulado_R=0; error_anterior_R=0;
    return;
  }

  double ref_L, ref_R;
  portENTER_CRITICAL_ISR(&paramMux);
  ref_L = target_L; ref_R = target_R;
  portEXIT_CRITICAL_ISR(&paramMux);

  long pos_L, pos_R;
  portENTER_CRITICAL_ISR(&timerMux);
  pos_L = contador_L; pos_R = contador_R;
  portEXIT_CRITICAL_ISR(&timerMux);

  double vel_L = ((double)(pos_L - contador_L_ant) / DELTA_T_S) * FACTOR_CUENTAS_A_RADS;
  double vel_R = ((double)(pos_R - contador_R_ant) / DELTA_T_S) * FACTOR_CUENTAS_A_RADS;
  
  contador_L_ant = pos_L; contador_R_ant = pos_R;

  double out_L = calcularPID(ref_L, vel_L, error_acumulado_L, error_anterior_L, Kp_L, Ki_L, Kd_L);
  double out_R = calcularPID(ref_R, vel_R, error_acumulado_R, error_anterior_R, Kp_R, Ki_R, Kd_R);

  setMotorL((int)out_L); setMotorR((int)out_R);
}

void calcular_posicion() {
  long sl_curr = contador_L;
  long sr_curr = contador_R;

  long delta_l_ticks = sl_curr - sl_prev;
  long delta_r_ticks = sr_curr - sr_prev;
  sl_prev = sl_curr;
  sr_prev = sr_curr;

  double sl = 2.0 * PI * RADIO_RUEDA * ((double)delta_l_ticks / CUENTAS_POR_REV_TOTAL);
  double sr = 2.0 * PI * RADIO_RUEDA * ((double)delta_r_ticks / CUENTAS_POR_REV_TOTAL);

  double dx_l = 0, dy_l = 0, dt = 0;

  if (abs(sr - sl) < 1e-6) {
    dx_l = (sr + sl) / 2.0;
  } else {
    double l = ANCHO_ROBOT;
    double common = ((sr + sl) / (sr - sl)) * (l / 2.0);
    double arg = (sr - sl) / l;
    dx_l = common * sin(arg);
    dy_l = common * (1.0 - cos(arg));
    dt = arg;
  }

  double dx_g = dx_l * cos(theta_actual) - dy_l * sin(theta_actual);
  double dy_g = dx_l * sin(theta_actual) + dy_l * cos(theta_actual);

  pos_x_global += dx_g;
  pos_y_global += dy_g;
  theta_actual += dt;

  while(theta_actual > PI) theta_actual -= 2*PI;
  while(theta_actual < -PI) theta_actual += 2*PI;
}

void reset_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    portENTER_CRITICAL(&paramMux);
    pos_x_global = 0.0;
    pos_y_global = 0.0;
    theta_actual = 0.0;
    error_acumulado_L = 0; error_anterior_L = 0;
    error_acumulado_R = 0; error_anterior_R = 0;
    target_L = 0; target_R = 0;
    portEXIT_CRITICAL(&paramMux);

    portENTER_CRITICAL(&timerMux);
    contador_L = 0; contador_R = 0;
    contador_L_ant = 0; contador_R_ant = 0;
    portEXIT_CRITICAL(&timerMux);
    
    sl_prev = 0; sr_prev = 0;

    digitalWrite(LED_PIN, LOW); delay(50); digitalWrite(LED_PIN, HIGH);
  }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  if (obstaculo_delante && linear_x > 0) {
    linear_x = 0; 
  }

  double vel_izq_ms = linear_x - (angular_z * ANCHO_ROBOT / 2.0);
  double vel_der_ms = linear_x + (angular_z * ANCHO_ROBOT / 2.0);

  portENTER_CRITICAL(&paramMux);
  target_L = vel_izq_ms / RADIO_RUEDA; 
  target_R = vel_der_ms / RADIO_RUEDA; 
  portEXIT_CRITICAL(&paramMux);
}

void subs_settings_callback(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // Verificamos que el PC nos ha mandado al menos 3 valores para no leer memoria basura
  if (msg->data.size >= 3) {
    portENTER_CRITICAL(&paramMux);
    sensores[0].voltaje_mV_minimo = msg->data.data[0];
    sensores[1].voltaje_mV_minimo = msg->data.data[1];
    sensores[2].voltaje_mV_minimo = msg->data.data[2];
    portEXIT_CRITICAL(&paramMux);
    
    Serial.println("Umbrales de seguridad actualizados por ROS2");
  }
}

// ==========================================
//   SETUP
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- ARRANCANDO EL CEREBRO DEL ROBOT ---");

  Serial.println("1. Configurando motores...");
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pararMotores();

  ledcAttach(IN1_L, frecuencia, 10); ledcAttach(IN2_L, frecuencia, 10);
  ledcAttach(IN1_R, frecuencia, 10); ledcAttach(IN2_R, frecuencia, 10);

  Serial.println("2. Configurando encoders...");
  pinMode(PIN_A_L, INPUT_PULLUP); pinMode(PIN_B_L, INPUT_PULLUP);
  pinMode(PIN_A_R, INPUT_PULLUP); pinMode(PIN_B_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A_L), isr_L_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B_L), isr_L_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_A_R), isr_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B_R), isr_R_B, CHANGE);

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);

  Serial.println("3. Inicializando sensores US-016...");
  // --- INIT SENSORES Y TAREA FREERTOS ---
  analogSetAttenuation(ADC_11db); 
  for (int s = 0; s < 3; s++) {
    inicializarSensor(sensores[s]);
  }
  Serial.println("4. Creando Tarea FreeRTOS...");
  xTaskCreate(tareaLecturaUS016, "Lectura_US016", 2048, NULL, 1, NULL);

  // --- WIFI Y MICRO-ROS ---
  Serial.println("5. Conectando al WiFi (Esto puede tardar)...");
  set_microros_wifi_transports("NOMBRE_WIFI", "CONTRASEÑA_WIFI", "IP_MV", 8888);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500); Serial.print(".");
  }
  Serial.println("\n--- RED ---");
  Serial.println("WiFi Conectado correctamente");
  Serial.print("IP asignada al ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.println("----------------");
  Serial.println("Esperando 2 segundos para estabilizar la red...");
  delay(2000);

  sistema_seguro = true;

  if (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK)
  {
    Serial.println("Error Agente ROS"); error_loop();
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/wheel_ticks"));

  // Inicialización del publicador Float32MultiArray
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_laser, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/laser_scan"));

  RCCHECK(rclc_publisher_init_default(&pub_pos_estimada, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/posicion_estimada"));

  encoder_msg.data.capacity = 2;
  encoder_msg.data.size = 2;
  encoder_msg.data.data = (int32_t *)malloc(encoder_msg.data.capacity * sizeof(int32_t));

  // Asignar memoria para los 3 sensores
  laser_msg.data.capacity = 3;
  laser_msg.data.size = 3;
  laser_msg.data.data = (float *)malloc(laser_msg.data.capacity * sizeof(float));

  msg_sub_settings.data.capacity = 3;
  msg_sub_settings.data.size = 0; // Se inicializa a 0, micro-ROS lo actualiza al recibir
  msg_sub_settings.data.data = (float *)malloc(msg_sub_settings.data.capacity * sizeof(float));

  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_reset, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/reset_odometry"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_settings, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/settings/robot"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_reset, &msg_reset, &reset_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_settings, &msg_sub_settings, &subs_settings_callback, ON_NEW_DATA));

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimerPID);
  timerAlarm(timer, 10000, true, 0); 

  Serial.println("--- SISTEMA OK (PID + 3 Sensores Crudos) ---");
}

// ==========================================
//   LOOP PRINCIPAL
// ==========================================

unsigned long last_pub_odom = 0;

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    sistema_seguro = false;
    WiFi.disconnect(); WiFi.reconnect();
    delay(500); return;
  } else {
    if (!sistema_seguro) sistema_seguro = true;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  if (millis() - last_pub_odom > 50) { 
    last_pub_odom = millis();
    
    // --- LÓGICA OBSTÁCULOS POR MILIVOLTIOS ---
    
    if (sensores[0].voltaje_mV < sensores[0].voltaje_mV_minimo || 
        sensores[1].voltaje_mV < sensores[1].voltaje_mV_minimo || 
        sensores[2].voltaje_mV < sensores[2].voltaje_mV_minimo) {
       obstaculo_delante = true; 
       digitalWrite(LED_PIN, LOW); 
    } else {
       obstaculo_delante = false;
       digitalWrite(LED_PIN, HIGH);
    }

    // --- PUBLICAR POSICIÓN ---
    calcular_posicion();
    pos_msg.x = pos_x_global;
    pos_msg.y = pos_y_global;
    pos_msg.z = theta_actual;
    RCSOFTCHECK(rcl_publish(&pub_pos_estimada, &pos_msg, NULL));

    // --- PUBLICAR SENSORES (3 valores en mV) ---
    laser_msg.data.data[0] = sensores[0].voltaje_mV;
    laser_msg.data.data[1] = sensores[1].voltaje_mV;
    laser_msg.data.data[2] = sensores[2].voltaje_mV;
    RCSOFTCHECK(rcl_publish(&publisher_laser, &laser_msg, NULL));
  }
}