// warning!! Enabling serial debug will increase response latency
#define SERIAL_DEBUG 1

// demo mode: no wifi and MQTT, just pulse motors one after the other
#define DEMO_MODE 1

// servo mode
//#define USE_ENCODERS 1

#include <stdint.h>

#include <Esp.h>
#if !DEMO_MODE
#include <WiFi.h>
#include <PubSubClient.h>
#endif
#include "esp_system.h"
#include "esp_sleep.h"


volatile uint8_t serial_delay_counter = 0;
volatile uint32_t bat_volt_adc_raw = 0;

#if DEMO_MODE
volatile uint8_t foo = 0;
volatile uint8_t motor_idx = 0;
#endif

// ----------------------------------------------------------------------------
// definitions for trigger control output

// const int ledPin = 33;

// ----------------------------------------------------------------------------
// definitions for servo control
// #define DIRECTION_COUNTER_CLOCKWISE 0
// #define DIRECTION_CLOCKWISE 1

#define PWM_RESOLUTION 8
#define PWM_FREQ  25000

#define PWM_L_PIN_A 22
#define PWM_L_PIN_B 23
#define PWM_R_PIN_A 19
#define PWM_R_PIN_B 18
#define PWM_C_PIN_A 17
#define PWM_C_PIN_B 16

#define PWM_EN_PIN 2

#define MOTOR_L_PWM_CHANNEL_A 0
#define MOTOR_L_PWM_CHANNEL_B 1
#define MOTOR_R_PWM_CHANNEL_A 2
#define MOTOR_R_PWM_CHANNEL_B 3
#define MOTOR_C_PWM_CHANNEL_A 4
#define MOTOR_C_PWM_CHANNEL_B 5

#define ENCODER_L_CLK_PIN 27
#define ENCODER_L_DT_PIN 26
#define ENCODER_R_CLK_PIN 34
#define ENCODER_R_DT_PIN 21
#define ENCODER_C_CLK_PIN 33
#define ENCODER_C_DT_PIN 32

#define BAT_VOLT_ADC_PIN 15


const unsigned int min_dutycycle = 30;  // allow transistor to turn fully on before turning it off on heavy loads (transistor spends too much time in linear range below this value)
const unsigned int max_dutycycle = 211;  // 14.4V supply for a 12V motor -> 83% max duty = 211/255olatile int delta_pos_L = 0;
volatile int delta_pos_R = 0;
volatile int delta_pos_C = 0;

hw_timer_t * timer = NULL;

// ----------------------------------------------------------------------------
// wifi connection and MQTT server

const char* ssid = "v2vr";
const char* password = "1098217356521888";

const char* hostname = "v2vr_esp32_client2";
// N.B. change also MQTT prefix, e.g. "vr2/" or "vr2/" !!!

const char* mqtt_server = "192.168.1.11";

#if !DEMO_MODE
WiFiClient espClient;
PubSubClient client(espClient);
#endif

long lastMsg = 0;
char msg[50];
int value = 0;

// ----------------------------------------------------------------------------
// timer interrupt handler to read out encoder

#ifdef USE_ENCODERS

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
  // Serial.print("Message arrived on topic: ");
  // Serial.print(topic);
  // Serial.print(". Message: ");
#endif
   char s[length+1];
   for (unsigned int i = 0; i < length; ++i) {
     s[i] = message[i];
   }
   s[length] = '\0';
  if (String(topic) == "vr2/right_controller_delta") {
    delta_pos_R = atoi(s);
  }
  else if (String(topic) == "vr2/left_controller_delta") {
    delta_pos_L = atoi(s);
  }
  else if (String(topic) == "vr2/head_mounted_position_delta") {
    delta_pos_C = atoi(s);
  }
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

#ifndef DEMO_MODE
  setup_wifi();

  Serial.println("client.setServer");
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
#else
  foo = min_dutycycle;
  motor_idx = 0;
#endif

  init_servos();
}


void loop() {
#ifdef DEMO_MODE
  ++foo;
  if (foo > max_dutycycle) {
    foo = min_dutycycle;
    ++motor_idx;
    if (motor_idx >= 12) {
      motor_idx = 0;
    }
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



  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");\
    // Attempt to connect
    if (client.connect(hostname)) {
      Serial.println("connected");
      client.subscribe("vr2/left_controller_delta");
      client.subscribe("vr2/right_controller_delta");
      client.subscribe("vr2/head_mounted_position_delta");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println();
      delay(1000);
      return;
    }
  }
  client.loop();
  //
//  Serial.print("delta_pos_R = ");
  // Serial.print(delta_pos_R);
  // Serial.print(". delta_pos_L = ");
  // Serial.print(delta_pos_L);
  // Serial.print(", delta_pos_C = ");
  // Serial.print(delta_pos_C);
  //   Serial.println();

  bat_volt_adc_raw = analogRead(BAT_VOLT_ADC_PIN);

  if (delta_pos_L > 0) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, delta_pos_L);
  }
  else if (delta_pos_L < 0) {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, -delta_pos_L);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
  }
  else {
    ledcWrite(MOTOR_L_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_L_PWM_CHANNEL_B, 0);
  }

  if (delta_pos_R > 0) {
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, delta_pos_R);
  }
  else if (delta_pos_R < 0) {
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, -delta_pos_R);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
  }
  else {
    ledcWrite(MOTOR_R_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_R_PWM_CHANNEL_B, 0);
  }

  if (delta_pos_C > 0) {
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, delta_pos_C);
  }
  else if (delta_pos_C < 0) {
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, -delta_pos_C);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
  else {
    ledcWrite(MOTOR_C_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_C_PWM_CHANNEL_B, 0);
  }
#endif
}
