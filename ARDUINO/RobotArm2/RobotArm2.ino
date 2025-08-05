//LIBRERIAS MICROROS
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <WiFi.h>

//LIBBRERIA BRAZO ROBOTICO
#include <ESP32Servo.h>

//____________INTERNET_________________
const char* ssid = "Latitude3490";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)

const uint32_t agent_port = 8888;
//______________________________________


//_______variables microros___________
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

rcl_timer_t sync_timer;

// Mensajes
std_msgs__msg__Int32MultiArray sub_msg;
std_msgs__msg__Bool pub_msg;

// Buffer para los números recibidos
int32_t received_numbers[4];

unsigned long long time_offset = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int led_error = 2;


struct timespec getTime() {
  struct timespec tp = { 0 };
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

//___________Definición de pines____________
#define Step 5
#define Dir 18
#define Enable 19
#define Step2 27
#define Dir2 12
#define Enable2 25
#define iman 17
const int FC1 = 32, FC2 = 33, FC3 = 35, FC4 = 34;
const int pinServo = 15;

Servo miServo;
int retardo = 1000, pasos = 0, dir = 0;
int32_t q1 = 0, q2 = 0, q3 = 0;
int32_t q1a = 0, q2a = 0, q3a = 0;
int32_t estado_iman = 0;
//__________________________________________________

void error_loop() {
  unsigned long long last_error_time = millis();
  while (1) {

    digitalWrite(led_error, !digitalRead(led_error));
    Serial.println("error..");
    delay(500);

    if (millis() - last_error_time > 3000) ESP.restart();
  }
}
//_________________________



void setup() {
  pinMode(led_error, OUTPUT);
  digitalWrite(led_error, HIGH);
  Serial.begin(115200);  // Inicializa el puerto serial

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a WiFi");

  //Configurar transporte micro-ROS por UDP
  set_microros_wifi_transports(
    (char*)ssid,          // Conversión explícita a char*
    (char*)password,      // Conversión explícita a char*
    (char*)agent_ip,      // Conversión explícita a char*
    agent_port            // uint32_t
  );

  allocator = rcl_get_default_allocator();

  //create init_options, node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm", "robot2", &support));

  delay(1000);

  // 4. Crear suscriptor para los 4 enteros
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "request_srv"  // Mejor práctica: usar '/' al inicio
          ));

  // 5. Crear publicador para el booleano
  RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "response_srv" // Mejor práctica: usar '/' al inicio
          ));

  //timer function for controlling the range
  RCCHECK(rclc_timer_init_default(
            &sync_timer,
            &support,
            RCL_MS_TO_NS(120000),
            sync_timer_callback));

  // 6. Inicializar mensajes
  sub_msg.data.capacity = 4;
  sub_msg.data.size = 0;
  sub_msg.data.data = (int32_t*)malloc(sub_msg.data.capacity * sizeof(int32_t));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));
  RCCHECK(rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &sub_msg,
            &subscription_callback,
            ON_NEW_DATA
          ));

  delay(1000);
  syncTime();

  // Configuración de hardware
  pinMode(FC1, INPUT); pinMode(FC2, INPUT); pinMode(FC3, INPUT); pinMode(FC4, INPUT);
  pinMode(Step, OUTPUT); pinMode(Dir, OUTPUT); pinMode(Enable, OUTPUT);
  pinMode(Step2, OUTPUT); pinMode(Dir2, OUTPUT); pinMode(Enable2, OUTPUT);
  miServo.attach(pinServo, 600, 2400); // Pin, µs mínimo, µs máximo
  //miServo.attach(pinServo);
  pinMode(iman, OUTPUT);
  
  // Posición home
  home();

  Serial.println("CORRECTO Y FUNCIONANDO");
  digitalWrite(led_error, LOW);
}

void loop() {
  //Serial.println("Ejecutando LOOP");
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  //Serial.println("DESPUES DE rclc_executor_spin_some");
  delay(10);
}

//____________________MICROROS_________________________
void syncTime() {
  unsigned long now = millis();
  Serial.println("Sincronizando tiempo con ROS 2...");

  for (int i = 0; i < 5; i++) {
    rcl_ret_t ret = rmw_uros_sync_session(10);
    if (ret == RCL_RET_OK) {
      unsigned long long ros_time_ms = rmw_uros_epoch_millis();
      time_offset = ros_time_ms - now;
      Serial.println("¡Sincronización exitosa!");
      return;
    }
    Serial.println("Reintentando sincronización...");
    delay(500);
  }

  Serial.println("¡Fallo al sincronizar tiempo! Continuando sin sync...");

  //RCCHECK(rmw_uros_sync_session(10));
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}

// Callback del servicio
void subscription_callback(const void *msg_in)
{
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;

  if (msg->data.size >= 4) {
    for (int i = 0; i < 4; i++) {
      received_numbers[i] = msg->data.data[i];
      //Serial.printf("Número %d recibido: %d\n", i+1, received_numbers[i]);
    }


    q1 = received_numbers[0];
    q2 = received_numbers[1];
    q3 = received_numbers[2];
    estado_iman = received_numbers[3];

    // Imprimir
    Serial.printf("q1: %d\n", q1);
    Serial.printf("q2: %d\n", q2);
    Serial.printf("q3: %d\n", q3);
    Serial.printf("efector: %d\n", estado_iman);

    // Asignar valores recibidos

    if (q1 > 180) {
      q1 = 180;
    } else if (q1 < 0) {
      q1 = 0;
    }

    if (q1 != q1a) {
      q1a = q1;
      miServo.write(q1);
      delay(500);
    }

    // Procesar movimientos
    if (q2 != q2a) {
      int delta_q2 = q2 - q2a;
      pasos = abs(delta_q2) * 25;
      if (delta_q2 > 0) {
        giro(Step, Dir, Enable, 0, 1);
      } else {
        giro(Step, Dir, Enable, 1, 1);
      }
      q2a = q2;
      pasos = 0;
      delay(500);
    }

    if (q3 != q3a) {
      int delta_q3 = q3 - q3a;
      pasos = abs(delta_q3) * 25;
      if (delta_q3 > 0) {
        giro(Step2, Dir2, Enable2, 1, 2);
      } else {
        giro(Step2, Dir2, Enable2, 0, 2);
      }
      q3a = q3;
      pasos = 0;
      delay(500);
    }
    
    delay(500);

    if (estado_iman == 0) {
      digitalWrite(iman, HIGH);
    } else {
      digitalWrite(iman, LOW);
    }

    delay(500);
    publish_bool(true);
  } else {
    Serial.println("Error: No se recibieron 4 números");
    publish_bool(false);
  }

}

// Función para publicar el booleano
void publish_bool(bool value) {
  pub_msg.data = value;
  //RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  rcl_publish(&publisher, &pub_msg, NULL);
  Serial.printf("Booleano publicado: %s\n", value ? "true" : "false");
}
//_______________________________________________________-

//___________FUNCIONES BRAZO ROBOTICO___________________
void home() {
  Serial.println("Ejecutando HOME");
  digitalWrite(iman, HIGH);
  pasos = 12000;
  giro(Step2, Dir2, Enable2, 1, 2);
  delay(500);
  giro(Step, Dir, Enable, 0, 1);
  delay(500);
  miServo.write(180);
  delay(500);
  pasos = 0;
  q1a = 180;
  q2a = 70;
  q3a = 80;
}

void giro(int paso_, int dire_, int habi_, int dir_, int nema_) {
  digitalWrite(habi_, LOW);
  bool emergencia = false;

  digitalWrite(dire_, dir_ ? HIGH : LOW);

  for (int i = 0; i < pasos; i++) {
    if (dir_ == 0) {
      if (digitalRead(FC2) == HIGH && nema_ == 1 ) {
        emergencia = true;
        q2 = 70;
      } else if (digitalRead(FC3) == HIGH && nema_ == 2 ) {
        emergencia = true;
        q3 = 0;
      }
    } else {
      if (digitalRead(FC1) == HIGH && nema_ == 1 ) {
        emergencia = true;
        q2 = 0;
      } else if (digitalRead(FC4) == HIGH && nema_ == 2 ) {
        emergencia = true;
        q3 = 80;
      }
    }

    if (emergencia) {
      for (int j = 0; j < 3; j++) {
        digitalWrite(paso_, HIGH);
        delayMicroseconds(100);
        digitalWrite(paso_, LOW);
        delayMicroseconds(100);
      }
      break;
    }

    digitalWrite(paso_, HIGH);
    delayMicroseconds(retardo);
    digitalWrite(paso_, LOW);
    delayMicroseconds(retardo);
  }

  digitalWrite(habi_, HIGH);
}
