/*

Robot controller
----------------

https://github.com/turingbirds/robot-driver

Copyright 2023
Apache License 2.0

*/

// warning!! Enabling serial debug will increase response latency
#define SERIAL_DEBUG 0

// demo mode: no wifi and MQTT, just pulse motors one after the other
#define DEMO_MODE 0
#define DEMO_MODE_ADC_READING 0

// use feedback encoders/servo mode for main motors
#define USE_ENCODERS 0

// control small "normal" PWM servo motors (pins shared with encoder inputs--select only one at a time)
#define AUX_SERVO_CONTROLLER 1

#define MIN(x, y) ((x)<(y)?(x):(y))
#define MAX(x, y) ((x)>(y)?(x):(y))


#include <stdint.h>

#include <Esp.h>
#if !DEMO_MODE
#include <WiFi.h>
#include <PubSubClient.h>
#endif
#include "esp_system.h"
#include "esp_sleep.h"

#if AUX_SERVO_CONTROLLER
#include <ESP32Servo.h>
#endif

// ----------------------------------------------------------------------------
// definitions

#define PWM_RESOLUTION 8
#define PWM_FREQ  25000

#define PWM_L_PIN_A 22
#define PWM_L_PIN_B 23
#define PWM_R_PIN_A 19
#define PWM_R_PIN_B 18
#define PWM_C_PIN_A 17
#define PWM_C_PIN_B 16

#define PWM_EN_PIN 2

#define MOTOR_L_PWM_CHANNEL_A 6
#define MOTOR_L_PWM_CHANNEL_B 7
#define MOTOR_R_PWM_CHANNEL_A 2
#define MOTOR_R_PWM_CHANNEL_B 3
#define MOTOR_C_PWM_CHANNEL_A 4
#define MOTOR_C_PWM_CHANNEL_B 5

#if USE_ENCODERS
#define ENCODER_L_CLK_PIN 27
#define ENCODER_L_DT_PIN 26
#define ENCODER_R_CLK_PIN 34
#define ENCODER_R_DT_PIN 21
#define ENCODER_C_CLK_PIN 33
#define ENCODER_C_DT_PIN 32
#endif

#if AUX_SERVO_CONTROLLER
#define AUX_SERVO_PWM_FREQ     50
#define AUX_SERVO_PWM_RESOLUTION  8

#define AUX_SERVO_1_PIN 21
#define AUX_SERVO_2_PIN 33

#define AUX_SERVO_PWM_CHANNEL_1 0
#define AUX_SERVO_PWM_CHANNEL_2 1
#endif

#define BAT_VOLT_ADC_PIN 36

const unsigned int min_dutycycle = 30;  // allow transistor to turn fully on before turning it off on heavy loads (transistor spends too much time in linear range below this value)
const unsigned int max_dutycycle = 211;  // 14.4V supply for a 12V motor -> 83% max duty = 211/255

// ----------------------------------------------------------------------------
// Global state

#if DEMO_MODE
volatile uint8_t foo = 0;
volatile uint8_t motor_idx = 0;
#endif

volatile uint8_t serial_delay_counter = 0;
volatile uint32_t bat_volt_adc_raw = 0;

volatile int vel_L = 0;
volatile int vel_R = 0;
volatile int vel_C = 0;

#if AUX_SERVO_CONTROLLER
// initial servo positions
volatile unsigned int aux_servo_pos_1 = 90;
volatile unsigned int aux_servo_pos_2 = 90;

Servo aux_servo_1;
Servo aux_servo_2;
#endif

hw_timer_t * timer = NULL;

// ----------------------------------------------------------------------------
// wifi connection and MQTT server

const char* ssid = "v2vr";
const char* password = "1098217356521888";
const char* mqtt_server = "192.168.1.11";
// const char* ssid = "Qeske Open";
// const char* password = "OpenWifi*";
// const char* mqtt_server = "10.15.2.73";


// ----------------------------------------------------------------------------
// our own IDs

const char* hostname = "v2vr_esp32_client";

// !! if the MQTT_PREFIX changes, change the optimized conditionals in the callback function too!!
#define MQTT_PREFIX "vr/"

// ----------------------------------------------------------------------------
#if !DEMO_MODE
WiFiClient espClient;
PubSubClient client(espClient);
#endif

long lastMsg = 0;
char msg[50];
int value = 0;

// ----------------------------------------------------------------------------
// timer interrupt handler to read out encoder

#if USE_ENCODERS

void IRAM_ATTR onTimer() {
  uint8_t CLK_L_current_state = digitalRead(ENCODER_L_CLK_PIN);

  if (CLK_L_current_state != CLK_L_previous_state) {
    // If the inputDT state is different than the inputCLK state then
    // the encoder is rotating counterclockwise
    if (digitalRead(ENCODER_L_DT_PIN) != CLK_L_current_state) {
      --pos_L;
      // direction = DIRECTION_COUNTER_CLOCKWISE;
    }
    else {
      ++pos_L;
      // direction = DIRECTION_CLOCKWISE;
    }
  }
  CLK_L_previous_state = CLK_L_current_state;

  uint8_t CLK_R_current_state = digitalRead(ENCODER_R_CLK_PIN);
  if (CLK_R_current_state != CLK_R_previous_state) {
    // If the inputDT state is different than the inputCLK state then
    // the encoder is rotating counterclockwise
    if (digitalRead(ENCODER_R_DT_PIN) != CLK_R_current_state) {
      --pos_R;
      // direction = DIRECTION_COUNTER_CLOCKWISE;
    }
    else {
      ++pos_R;
      // direction = DIRECTION_CLOCKWISE;
    }
  }
  CLK_R_previous_state = CLK_R_current_state;

  uint8_t CLK_C_current_state = digitalRead(ENCODER_C_CLK_PIN);
  if (CLK_C_current_state != CLK_C_previous_state) {
    // If the inputDT state is different than the inputCLK state then
    // the encoder is rotating counterclockwise
    if (digitalRead(ENCODER_C_DT_PIN) != CLK_C_current_state) {
      --pos_C;
      // direction = DIRECTION_COUNTER_CLOCKWISE;
    }
    else {
      ++pos_C;
      // direction = DIRECTION_CLOCKWISE;
    }
  }
  CLK_C_previous_state = CLK_C_current_state;
}
#endif

// ----------------------------------------------------------------------------

#if !DEMO_MODE
void setup_wifi() {
  delay(1000);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  WiFi.setSleep(false);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  #if SERIAL_DEBUG
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
  #endif

  // const String topic_str = String(topic); // optimized away (see conditionals below)
  char s[length+1];
  for (unsigned int i = 0; i < length; ++i) {
    s[i] = message[i];
  }
  s[length] = '\0';

  // optimization: if the topic string is shorter than 8 characters, don't consider it at all
  if (topic[0] == '\0' || topic[1] == '\0' || topic[2] == '\0' || topic[3] == '\0' || topic[4] == '\0' || topic[5] == '\0' || topic[6] == '\0' || topic[7] == '\0') {
    return;
  }

  //if (topic_str == MQTT_PREFIX "vel_R") {
  if (topic[7] == 'R') { // optimized version
    vel_R = atoi(s);
  }
  // else if (topic_str == MQTT_PREFIX "vel_L") {
  else if (topic[7] == 'L') { // optimized version
    vel_L = atoi(s);
  }
  // else if (topic_str == MQTT_PREFIX "vel_C") {
  else if (topic[7] == 'C') { // optimized version
    vel_C = atoi(s);
  }
#if AUX_SERVO_CONTROLLER
  // optimization: if the topic string is shorter than 12 characters, don't consider it at all
  if (topic[8] == '\0' || topic[9] == '\0' || topic[10] == '\0' || topic[11] == '\0') {
    return;
  }

  // if (topic_str == MQTT_PREFIX "aux_pos_1") {
 if (topic[11] == '1') { // optimized version
    aux_servo_pos_1 = atoi(s);
    aux_servo_pos_1 = MIN(140, aux_servo_pos_1);
    aux_servo_pos_1 = MAX(60, aux_servo_pos_1);
  }
  // else if (topic_str == MQTT_PREFIX "aux_pos_2") {
  else if (topic[11] == '2') { // optimized version
    aux_servo_pos_2 = atoi(s);
  }
#endif
}
#endif
// ----------------------------------------------------------------------------

void init_servos() {
  ledcSetup(MOTOR_L_PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_L_PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_C_PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_C_PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PWM_L_PIN_A, MOTOR_L_PWM_CHANNEL_A);
  ledcAttachPin(PWM_L_PIN_B, MOTOR_L_PWM_CHANNEL_B);
  ledcAttachPin(PWM_R_PIN_A, MOTOR_R_PWM_CHANNEL_A);
  ledcAttachPin(PWM_R_PIN_B, MOTOR_R_PWM_CHANNEL_B);
  ledcAttachPin(PWM_C_PIN_A, MOTOR_C_PWM_CHANNEL_A);
  ledcAttachPin(PWM_C_PIN_B, MOTOR_C_PWM_CHANNEL_B);

  // enable
  pinMode(PWM_EN_PIN, OUTPUT);
  digitalWrite(PWM_EN_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("In setup(): ");

  adcAttachPin(BAT_VOLT_ADC_PIN);
  analogSetPinAttenuation(BAT_VOLT_ADC_PIN, ADC_2_5db);    // The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1100 mV. (1V input = ADC reading of 3722)

#if !DEMO_MODE
  setup_wifi();

  Serial.println("client.setServer");
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
#else
  foo = min_dutycycle;
  motor_idx = 0;
#endif

  init_servos();

// #if AUX_SERVO_CONTROLLER
//   ledcSetup(AUX_SERVO_PWM_CHANNEL_1, AUX_SERVO_PWM_FREQ, AUX_SERVO_PWM_RESOLUTION);
//   ledcSetup(AUX_SERVO_PWM_CHANNEL_2, AUX_SERVO_PWM_FREQ, AUX_SERVO_PWM_RESOLUTION);
//
//   ledcAttachPin(AUX_SERVO_1_PIN, AUX_SERVO_PWM_CHANNEL_1);
//   ledcAttachPin(AUX_SERVO_2_PIN, AUX_SERVO_PWM_CHANNEL_2);
// #endif

#if AUX_SERVO_CONTROLLER
  ESP32PWM::allocateTimer(AUX_SERVO_PWM_CHANNEL_1);
  ESP32PWM::allocateTimer(AUX_SERVO_PWM_CHANNEL_2);
  aux_servo_1.setPeriodHertz(50); // [Hz]
  aux_servo_2.setPeriodHertz(50); // [Hz]
  aux_servo_1.attach(AUX_SERVO_1_PIN, 500, 2400);
  aux_servo_2.attach(AUX_SERVO_2_PIN, 500, 2400);
  #endif
}


void loop() {
#if DEMO_MODE
  ++foo;
  if (foo > max_dutycycle) {
    foo = min_dutycycle;
    ++motor_idx;
    if (motor_idx >= 12) {
      motor_idx = 0;
    }

#if DEMO_MODE_ADC_READING
    bat_volt_adc_raw = analogRead(BAT_VOLT_ADC_PIN);
    const float bat_volt_adc = bat_volt_adc_raw * 0.0067;
    const float bat_volt_adc_corrected = bat_volt_adc + (20. - bat_volt_adc) / 20.; // some ad-hoc correction of the horrible ADC nonlinearity

    Serial.print(bat_volt_adc_raw);
    Serial.println();
    Serial.printf("%f\n", bat_volt_adc_corrected);
#endif
  }

  // one motor after the other
  if (motor_idx == 0) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, foo);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 1) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, foo);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 2) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, foo);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 3) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, foo);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 4) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, foo);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 5) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, foo);
  }
  else if (motor_idx == 6) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, foo);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, foo);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, foo);
  }
  else if (motor_idx == 7) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, foo);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, foo);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, foo);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 8) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, max_dutycycle-foo);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, max_dutycycle-foo);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, max_dutycycle-foo);
  }
  else if (motor_idx == 9) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, max_dutycycle-foo);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, max_dutycycle-foo);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, max_dutycycle-foo);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else if (motor_idx == 10 || motor_idx == 11) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, max_dutycycle);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, max_dutycycle);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, max_dutycycle);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  delay(10);

#else

  // attempt to connect to MQTT broker
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");\
    if (client.connect(hostname)) {
      Serial.println("connected");
      client.subscribe(MQTT_PREFIX "vel_L");
      client.subscribe(MQTT_PREFIX "vel_R");
      client.subscribe(MQTT_PREFIX "vel_C");
#if AUX_SERVO_CONTROLLER
      client.subscribe(MQTT_PREFIX "aux_pos_1");
      client.subscribe(MQTT_PREFIX "aux_pos_2");
#endif
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println();
      Serial.print("Attempting Wifi connection...");\
      WiFi.reconnect();
      delay(1000);
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting Wifi connection...");\
        WiFi.reconnect();
        delay(1000);
      }
      delay(1000);
      return;
    }
  }
  client.loop();
  // delay(1);


  // ----------------------------------------------------------------------------
  // report on battery voltage

  bat_volt_adc_raw = analogRead(BAT_VOLT_ADC_PIN);
  const float bat_volt_adc = bat_volt_adc_raw * 0.0067;
  const float bat_volt_adc_corrected = bat_volt_adc + (20. - bat_volt_adc) / 20.; // some ad-hoc correction of the horrible ADC nonlinearity

  char bat_volt_adc_corrected_str[80];
  sprintf(bat_volt_adc_corrected_str, "%f", bat_volt_adc_corrected);
  client.publish(MQTT_PREFIX "battery_voltage", bat_volt_adc_corrected_str, true);

  // ----------------------------------------------------------------------------
  // set motors

  if (vel_L > 0) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, vel_L);
  }
  else if (vel_L < 0) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, -vel_L);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
  }
  else {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
  }

  if (vel_R > 0) {
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, vel_R);
  }
  else if (vel_R < 0) {
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, -vel_R);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
  }
  else {
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
  }

  if (vel_C > 0) {
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, vel_C);
  }
  else if (vel_C < 0) {
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, -vel_C);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else {
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }

// #if AUX_SERVO_CONTROLLER
//   ledcWrite(AUX_SERVO_PWM_CHANNEL_1, aux_servo_pos_1);
//   ledcWrite(AUX_SERVO_PWM_CHANNEL_2, aux_servo_pos_2);
// #endif
#ifdef AUX_SERVO_CONTROLLER
  aux_servo_1.write(aux_servo_pos_1);
  aux_servo_2.write(aux_servo_pos_2);
#endif

// #if SERIAL_DEBUG
//   // enabling this renders everything very, very slow
//   Serial.print("vel_R = ");
//   Serial.print(vel_R);
//   Serial.print(". vel_L = ");
//   Serial.print(vel_L);
//   Serial.print(", vel_C = ");
//   Serial.print(vel_C);
//   Serial.println();
//   #if AUX_SERVO_CONTROLLER
//   Serial.print("aux_servo_pos_1 = ");
//   Serial.print(aux_servo_pos_1);
//   Serial.print(", aux_servo_pos_2 = ");
//   Serial.print(aux_servo_pos_2);
//   Serial.println();
// Serial.print(bat_volt_adc_raw);
// Serial.println();
// Serial.printf("%f\n", bat_volt_adc_corrected);
//   #endif
#endif

}
