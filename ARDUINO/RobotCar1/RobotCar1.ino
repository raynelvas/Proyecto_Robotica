#include <ESP32Encoder.h>
#include "motorControl.h"
#include "imu_publisher.h"
#include "odometry.h"

/*Motor controller using micro_ros serial set_microros_transports*/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

#include <WiFi.h>  // ← Nueva librería


//_______variables microros___________
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
rcl_subscription_t reset_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t range_publisher;
rcl_publisher_t reset_publisher;

geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range range_msg;
std_msgs__msg__Bool sub_msg;
std_msgs__msg__Bool pub_msg;

rcl_timer_t odom_timer;
rcl_timer_t imu_timer;
rcl_timer_t range_timer;
rcl_timer_t sync_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

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

void error_loop() {
  unsigned long long last_error_time = millis();
  while (1) {

    digitalWrite(led_error, !digitalRead(led_error));
    Serial.println("error..");
    delay(500);

    if (millis() - last_error_time > 3000) ESP.restart();
  }
}
//_______________________________________________________

//____________BIBLIOTECAS PROPIAS___________________________
Odometry odometry;
ImuPublisher imu_pub;
//IMUReader imu;
ESP32Encoder encoderI;
ESP32Encoder encoderD;
//_______________________________________________

//____________INTERNET_________________
const char* ssid = "Latitude3490";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)

const uint32_t agent_port = 8888;
//___________________________

//_____________________________________IMU_________________________________________________________________________
#include <Wire.h>
#include "mpu9250.h"

bfs::Mpu9250 imu;

// Offsets calibrados
float accX_offset = 0.062685;
float accY_offset = 0.164525;
float accZ_offset = 1.304075;

float gyroX_offset = -0.024333;
float gyroY_offset = -0.012607;
float gyroZ_offset = 0.019727;

const int samples = 500;

float ax_uf, ay_uf, az_uf;
float gx_uf, gy_uf, gz_uf;
//_________________________________________________________________________________________________________________

//___________Parametros del robot_____________
float L = 0.196; // distancia entre ruedas metros
float D = 0.0853; // Diametro de la rueda en metros
//____________________________________________

//___________ESTRUCTURA PARA DATOS MOTORES__________
struct DatosMotores {
  volatile long encoderValor = 0;
  volatile byte ant = 0;
  volatile byte act = 0;
  double w = 0.0;  // Revoluciones por minuto calculadas.
  double wAnterior = 0.0;
  double wRef = 0.0;  // Velocidad angular en rad/s.
  int outValue = 0; //Variable de control (pwm)
  double wSinFilter = 0.0;     // Velocidad angular en rad/s.
  double v = 0.0;
};

DatosMotores Izq;
DatosMotores Der;
//_________________________________________________

//_____________MOTOR IZQUIERDO___________
const int IN3 = 19; //REAL IN3 = 18
const int IN4 = 18; //REAL IN4 = 19
const int ENB = 23;

int canalMI = 0;
//_______________________________________

//_______________MOTOR DERECHO___________
const int IN1 = 17; // REAL IN1 = 16
const int IN2 = 16; // REAL IN2 = 17
const int ENA = 4;

int canalMD = 1;
//_______________________________________

//________CONFIGURACION ENCODER__________
unsigned long lastTime = 0;      // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo milisegundos
unsigned int ppr = 1500;         // Número de muescas que tiene el disco del encoder.
//_______________________________________

//___________ENCODER DERECHO_____________
#define DerechoC1 33
#define DerechoC2 32
//_______________________________________

//__________ENCODER IZQUIERDO____________
#define IzquierdoC1 35
#define IzquierdoC2 34
//_______________________________________

//_________SENSOR ULTRASONICO_______
const int trig = 25;
const int echo = 26;

double distancia = 0;
//_________________________________________

//__________PWM__________________________
const int frecuencia = 10000;
const int resolucion = 8;
//_______________________________________

//____________CONFIGURACION PID__________________
motorControl motorD(sampleTime);
motorControl motorI(sampleTime);

//FILTRO DIGITAL
double alpha = 0.8;
//_______________________________________________

//______________VARIABLES A ENVIAR_________________
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float roll = 0, pitch = 0, yaw = 0;
float v = 0, w = 0;

float x_pos = 0.0, delta_x = 0.0;
float y_pos = 0.0,  delta_y = 0.0;
float yaw_odom = 0.0, delta_yaw_odom = 0.0;
float yaw_imu = 0.0, delta_yaw_imu = 0.0;
unsigned long lastTime_imu = 0, dt_imu = 0;
//__________________________________________

//___________FUNCIONES A UTILIZAR_________________
void updateEncoderRight();
void updateEncoderLeft();
void giroHorario(int canal, int I1, int I2, int cv);
void giroAntihorario(int canal, int I1, int I2, int cv);
void pararMotor(int canal, int I1, int I2);
double ping();
//___________________________________________

void setup(){
  pinMode(led_error, OUTPUT);
  digitalWrite(led_error, HIGH);
  Serial.begin(115200);  // Inicializa el puerto serial a 9600 bps
  Serial.println("ESP32 iniciada y lista para recibir comandos");

  //delay(1000);

  //______MICROROS_________
  //set_microros_transports();
  //delay(2000);

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
  delay(1000);


  allocator = rcl_get_default_allocator();

  //create init_options, node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "car","robot1", &support));

  // 4. Crear suscriptor para el servicio
  RCCHECK(rclc_subscription_init_default(
            &reset_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "reset/request_srv" 
          ));

  // 5. Crear publicador para el booleano
  RCCHECK(rclc_publisher_init_default(
            &reset_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "reset/response_srv" 
          ));

  // subscritor for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"));

  //create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
            &odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "odom/unfiltered"));

  //create a imu publisher
  RCCHECK(rclc_publisher_init_default(
            &imu_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu/unfiltered"));

  //create a range publishe
  RCCHECK(rclc_publisher_init_default(
            &range_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            "range/unfiltered"));

  //timer function for controlling the odom
  RCCHECK(rclc_timer_init_default(
            &odom_timer,
            &support,
            RCL_MS_TO_NS(80),
            odom_callback));

  //timer function for controlling the imu
  RCCHECK(rclc_timer_init_default(
            &imu_timer,
            &support,
            RCL_MS_TO_NS(1000 / 30),
            imu_callback));

  //timer function for controlling the range
  RCCHECK(rclc_timer_init_default(
            &range_timer,
            &support,
            RCL_MS_TO_NS(160),
            range_callback));

  //timer function for controlling the range
  RCCHECK(rclc_timer_init_default(
            &sync_timer,
            &support,
            RCL_MS_TO_NS(120000),
            sync_timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&reset_subscriber, &sub_msg, &reset_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &range_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));

  delay(1000);
  syncTime();

  //_____________________MOTOR______________________
  ledcSetup(canalMI, frecuencia, resolucion);
  ledcAttachPin(ENB, canalMI);
  //ledcAttachChannel(ENB, frecuencia, resolucion, canalMD);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(canalMD, frecuencia, resolucion);
  ledcAttachPin(ENA, canalMD);
  //ledcAttachChannel(ENA, frecuencia, resolucion, canalMI);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //_____________________ENCODER______________________
  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Opcional

  encoderD.attachFullQuad(DerechoC1, DerechoC2);  // A (C1), B (C2)
  encoderD.setCount(0);            // Iniciar en cero

  encoderI.attachFullQuad(IzquierdoC1, IzquierdoC2);  // A (C1), B (C2)
  encoderI.setCount(0);            // Iniciar en cero

  //_____________CONTROL PID_______________
  motorI.setGains (0.7, 0.11, 0.03); // (Kc,Ti,Td)
  motorD.setGains (0.4, 0.11, 0.03); // (Kc,Ti,Td);

  motorI.setCvLimits(255, 130);  //130
  motorD.setCvLimits(255, 130); //130

  motorI.setPvLimits(16.9, 0); //izquierdo 16.4
  motorD.setPvLimits(16.9, 0); //derecho
  //__________________________________________

  //____________IMU______________
  while (!Serial) {}

  if (!IMU_begin()) {
    Serial.println("Error al inicializar el IMU");
    while (1);
  }
  //_______________________________

  //_________________SENSOR ULTRASONICO____________
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //_______________________________

  //________mensaje fijo______________
  // Mensaje fijo
  range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view = 0.52;
  range_msg.min_range = 0.01;
  range_msg.max_range = 2;
  range_msg.header.frame_id.data = (char*)"robot1/range_link";
  range_msg.header.frame_id.size = strlen("robot1/range_link");
  range_msg.header.frame_id.capacity = range_msg.header.frame_id.size + 1;
  //_________________________________________________________________

  // Tarea para control de velocidad, en Core 0
  xTaskCreatePinnedToCore(
    ControlLoop,        // Función
    "ControlLoopTask",  // Nombre
    4096,               // Stack size
    NULL,               // Parametros
    1,                  // Prioridad
    NULL,               // Handle
    0                   // Core 0
  );

  Serial.println("CORRECTO Y FUNCIONANDO");
  digitalWrite(led_error, LOW);

  lastTime = millis();
  lastTime_imu = millis();

  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
}

void loop() {
  //Serial.print("loop() running on core ");
  //Serial.println(xPortGetCoreID());
  //delay(10);
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}

//___________MOVIMIENTOS DE AVANCE _________
void giroHorario(int canal, int I1, int I2, int cv) {
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  ledcWrite(canal, cv);
  //ledcWriteChannel(canal, cv);
}

void giroAntihorario(int canal, int I1, int I2, int cv) {
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  ledcWrite(canal, cv);
  //ledcWriteChannel(canal, cv);
}

void pararMotor(int canal, int I1, int I2) {
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  ledcWrite(canal, 0);
  //ledcWriteChannel(canal, 0);
}
//______________________________________________

//_____________SENSOR ULTRASONICO____________
double ping() {
  long duration = 0;
  const unsigned long timeout = 25000UL; // 25 ms (seguro para 4m + margen)

  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  //duration = pulseIn(echo,HIGH); //Duracion del puso echo en us
  duration = pulseIn(echo, HIGH, timeout);

  //Serial.println(duration);
  double dis = duration * 0.017;
  return dis;
}
//__________________________________________________________

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

void odom_callback(rcl_timer_t* timer, int64_t last_call_time) {
  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  //Serial.println("publicando");

  odometry.update(
    v,
    w,
    x_pos,
    y_pos,
    yaw_odom);

  odom_msg = odometry.getData();

  struct timespec time_stamp = getTime();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void imu_callback(rcl_timer_t * timer, int64_t last_call_time) {
  imu_pub.update(
    ax, ay, az,
    gx, gy, gz,
    roll, pitch, yaw
  );

  imu_msg = imu_pub.getData();

  struct timespec time_stamp = getTime();
  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void range_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    double distancia = ping() / 100.0;  // Convertir a metros

    struct timespec time_stamp = getTime();
    range_msg.header.stamp.sec = time_stamp.tv_sec;
    range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    range_msg.range = distancia;

    RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL));
  }
}

void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
  float linear = msg.linear.x;
  float angular = msg.angular.z;;

  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vI = linear - (angular * L) / 2;
  float vD = linear + (angular * L) / 2;

  Izq.wRef = (2 * vI) / D;
  Der.wRef = (2 * vD) / D;
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}

// Callback del servicio
void reset_callback(const void *msg_in)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
   // Imprimir el valor real
  Serial.printf("Booleano recibido: %s\n", msg->data ? "true" : "false");
  if(msg->data){
    ax = 0, ay = 0, az = 0;
    gx = 0, gy = 0, gz = 0;
    roll = 0, pitch = 0, yaw = 0;
    v = 0, w = 0;
    
    x_pos = 0.0;
    y_pos = 0.0;
    yaw_odom = 0.0;
    yaw_imu = 0.0;
    publish_bool(true);
  }else{
    Serial.println("Error al resetear: valor falso recibido");
    publish_bool(false);
  }
}


// Función para publicar el booleano
void publish_bool(bool value) {
  pub_msg.data = value;
  //RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  rcl_publish(&reset_publisher, &pub_msg, NULL);
  Serial.printf("Booleano publicado: %s\n", value ? "true" : "false");
}

//_________________________________________________

//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {
  Serial.print("ControlLoop() running on core ");
  Serial.println(xPortGetCoreID());
  
  while (1) {
    

    if (millis() - lastTime_imu >= 100) {
      dt_imu = millis() - lastTime_imu;
      lastTime_imu = millis();
      IMU_update();

      roll = 0;
      pitch = 0;      

      delta_yaw_imu = gz * dt_imu / 1000;
      yaw_imu += delta_yaw_imu;

      yaw = yaw_imu;
    }

    if (millis() - lastTime >= sampleTime) {// Se actualiza cada tiempo de muestreo
      unsigned long dt = millis() - lastTime; //milisegundos
      lastTime = millis();  // Almacenamos el tiempo actual.

      Izq.encoderValor = encoderI.getCount();
      Der.encoderValor = encoderD.getCount();

      Izq.wSinFilter = Izq.encoderValor * (2 * 3.14159 * 1000) / ( dt * ppr );
      Der.wSinFilter = Der.encoderValor * (2 * 3.14159 * 1000) / ( dt * ppr );

      encoderI.setCount(0);
      encoderD.setCount(0);

      Izq.w = alpha * Izq.wSinFilter + (1.0 - alpha) * Izq.wAnterior;
      Izq.wAnterior = Izq.w;

      Der.w = alpha * Der.wSinFilter + (1.0 - alpha) * Der.wAnterior;
      Der.wAnterior = Der.w;

      Izq.v = (Izq.w * D) / 2 ;
      Der.v = (Der.w * D) / 2 ;
      v = (Der.v + Izq.v) / 2;
      w = (Der.v - Izq.v) / L ; // rad/s

      delta_yaw_odom = w * dt / 1000; //radians
      float cos_h = cos(yaw_imu);
      float sin_h = sin(yaw_imu);
      delta_x = v * cos_h * dt / 1000; //m
      delta_y = v * sin_h * dt / 1000; //m

      //calculate current position of the robot
      x_pos += delta_x;//*0.85;
      y_pos += delta_y;//*1.1;
      yaw_odom += delta_yaw_odom;

      Izq.outValue = motorI.compute(Izq.wRef, Izq.w); //Control PID
      Der.outValue = motorD.compute(Der.wRef, Der.w); //Control PID

      if (Izq.outValue > 0) giroAntihorario(canalMI, IN3, IN4, Izq.outValue); else giroHorario(canalMI, IN3, IN4, abs(Izq.outValue));
      if (Der.outValue > 0) giroHorario(canalMD, IN1, IN2, Der.outValue); else giroAntihorario(canalMD, IN1, IN2, abs(Der.outValue));
    }
    delay(10);

  }
}
