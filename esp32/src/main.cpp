/*

Robot controller
----------------

https://github.com/turingbirds/robot-driver

Copyright 2023
Apache License 2.0

*/


/*
 * Serial debugging
**/

// warning!! Enabling serial debug will increase response latency
#define SERIAL_DEBUG 1
#define SERIAL_DEBUG_DETAILED 0

#include <esp_task_wdt.h>


/*
 * Wifi
**/

// const char* ssid = "Qeske Open";
// const char* password = "OpenWifi*";
// const char* ssid = "v2vr";
// const char* password = "1098217356521888";

#define WIFI_SSID "v2vr"
#define WIFI_PSK  "1098217356521888"
// #define WIFI_SSID "Qeske Open"
// #define WIFI_PSK  "OpenWifi*"

#include <Esp.h>
#include "esp_system.h"
#include "esp_sleep.h"

#include <WiFi.h>
#include <ArduinoJson.h>
#include "AsyncUDP.h"

// demo mode: no wifi and MQTT, just pulse motors one after the other
#define DEMO_MODE 0
#define DEMO_MODE_ADC_READING 0

// use feedback encoders/servo mode for main motors
#define USE_ENCODERS 0

// control small "normal" PWM servo motors (pins shared with encoder inputs--select only one at a time)
#define AUX_SERVO_CONTROLLER 1

#define MIN(x, y) ((x)<(y)?(x):(y))
#define MAX(x, y) ((x)>(y)?(x):(y))

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
const unsigned int max_dutycycle = 211;  // 14.4V supply for a 12V motor -> 83% max duty = 211/255. In the future this could be made dependent on measured battery voltage

#if AUX_SERVO_CONTROLLER
#include <ESP32Servo.h>
#endif

#if !DEMO_MODE
WiFiClient espClient;
AsyncUDP udp;

volatile unsigned long prev_millis = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 5000; // in milliseconds

unsigned int udp_comms_timeout = 500; // timeout in milliseconds -- stop robot if no packets received for this interval or more
unsigned int udp_time_since_last_packet = 0;
#endif

// ----------------------------------------------------------------------------
// Global state

#if DEMO_MODE
volatile uint8_t foo = 0;
volatile uint8_t motor_idx = 0;
#endif

volatile unsigned int message_counter = 0;

volatile unsigned int frame = 0;

volatile uint32_t serial_delay_counter = 0;
// volatile uint32_t serial_delay_counter2 = 0;
volatile uint32_t bat_volt_adc_raw = 0;
volatile float bat_volt_adc_corrected = -1.;

volatile int vel_L = 0;
volatile int vel_R = 0;
volatile int vel_C = 0;

#if AUX_SERVO_CONTROLLER
// initial servo positions
volatile unsigned int aux_pos_1 = 90;
volatile unsigned int aux_pos_2 = 90;

const unsigned int MAX_AUX_POS_1 = 140;
const unsigned int MIN_AUX_POS_1 = 60;

Servo aux_servo_1;
Servo aux_servo_2;
#endif








void init_udp_listener() {
	if (udp.listen(1234)) {
		Serial.print("UDP Listening on IP: ");
		Serial.println(WiFi.localIP());
		udp.onPacket([](AsyncUDPPacket packet) {
#if SERIAL_DEBUG
			Serial.print("[host -> esp32] UDP Packet Type: ");
			Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
			Serial.print(", From: ");
			Serial.print(packet.remoteIP());
			Serial.print(":");
			Serial.print(packet.remotePort());
			Serial.print(", To: ");
			Serial.print(packet.localIP());
			Serial.print(":");
			Serial.print(packet.localPort());
			Serial.print(", Length: ");
			Serial.print(packet.length()); //dlzka packetu
			Serial.print(", Data: ");
			Serial.write(packet.data(), packet.length());
			Serial.println();
#endif
			// reset packet timeout
			udp_time_since_last_packet = 0;

			String data_as_string = (const char*)packet.data();
			DynamicJsonDocument doc(1024);
			deserializeJson(doc, data_as_string);
#if SERIAL_DEBUG
			// Serial.write("[[[]]]%f]]", doc["vel_L"]);


						// message_counter += 1;

				    // char s[256];
			//			sprintf(s, "{\"vel_L\": \"%d\", \"vel_C\": \"%d\", \"vel_R\": \"%d\", \"aux_pos_1\": \"%d\", \"aux_pos_2\": \"%d\", \"cnt\": \"%d\", \"battery_voltage\": \"%f\"}\0", vel_L, vel_C, vel_R, aux_pos_1, aux_pos_2, message_counter, bat_volt_adc_corrected);
						// sprintf(s, "{\"vel_L\": \"%d\", \"vel_C\": \"%d\", \"vel_R\": \"%d\", \"aux_pos_1\": \"%d\", \"aux_pos_2\": \"%d\", \"cnt\": \"%d\", \"battery_voltage\": \"%f\"}\0", vel_L, vel_C, vel_R, aux_pos_1, aux_pos_2, message_counter, bat_volt_adc_corrected);



#endif
			vel_L = doc["vel_L"];
			vel_C = doc["vel_C"];
			vel_R = doc["vel_R"];
			#if AUX_SERVO_CONTROLLER
			unsigned int aux_pos_1_new = doc["aux_pos_1"];
			aux_pos_1_new = MIN(MAX_AUX_POS_1, aux_pos_1_new);
			aux_pos_1_new = MAX(MIN_AUX_POS_1, aux_pos_1_new);
			aux_pos_1 = aux_pos_1_new;
			aux_pos_2 = doc["aux_pos_2"];
			#endif

			// packet.printf("[esp32 -> host] received %u bytes of data", packet.length());
			packet.printf("{\"battery_voltage\": %f, \"free_heap\": %d}", bat_volt_adc_corrected, ESP.getFreeHeap());
		});
	}
}


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
#if SERIAL_DEBUG
	delay(1000);
	// For logging
	Serial.begin(115200);

	Serial.println("Setting up WDT");
// 3 seconds WDT
#define WDT_TIMEOUT 3
	esp_task_wdt_init(WDT_TIMEOUT, true);
	esp_task_wdt_add(NULL);

	// Connect to WiFi
	Serial.println("Setting up WiFi");
#endif

		WiFi.setSleep(false);
	  WiFi.begin(WIFI_SSID, WIFI_PSK);
	  while (WiFi.status() != WL_CONNECTED) {
			esp_task_wdt_reset();
	    Serial.print(".");
	    delay(500);
	  }
	  Serial.print("Connected. IP=");
	  Serial.println(WiFi.localIP());
		WiFi.setSleep(false);
		Serial.print("CPU frequency: ");
		Serial.println(getCpuFrequencyMhz());



	  adcAttachPin(BAT_VOLT_ADC_PIN);
	  analogSetPinAttenuation(BAT_VOLT_ADC_PIN, ADC_2_5db);    // The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1100 mV. (1V input = ADC reading of 3722)

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


	init_udp_listener();
}

#if DEMO_MODE
void demo_mode_loop() {
  ++foo;
  if (foo > max_dutycycle) {
    foo = min_dutycycle;
    ++motor_idx;
    if (motor_idx >= 12) {
      motor_idx = 0;
    }

#if DEMO_MODE_ADC_READING5
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
}
#endif

void loop() {
	// reset watchdog timer
	esp_task_wdt_reset();

	unsigned long current_millis = millis();

#if DEMO_MODE
	demo_mode_loop();
#else

	if (WiFi.status() != WL_CONNECTED) {
		vel_L = 0;
		vel_C = 0;
		vel_R = 0;

		Serial.println("Wifi got disconnected! Attempting to reconnect...");

		WiFi.disconnect();
		WiFi.setSleep(false);
	  WiFi.begin(WIFI_SSID, WIFI_PSK);
	  while (WiFi.status() != WL_CONNECTED) {
			esp_task_wdt_reset();
	    Serial.print(".");
	    delay(500);
	  }
	  Serial.print("Connected. IP=");
	  Serial.println(WiFi.localIP());
		WiFi.setSleep(false);

		udp_time_since_last_packet = 0;/home/charl/prj/v2/robot_controller/manual_control/http_requester.py
	}

	// network timeout handling
	delay(10);
	udp_time_since_last_packet += 10;

	if (udp_time_since_last_packet > udp_comms_timeout) {
		vel_L = 0;
		vel_C = 0;
		vel_R = 0;
	}

  // report on battery voltage
	++serial_delay_counter;
	if (serial_delay_counter >= 10) { // every 100 ms
		serial_delay_counter = 0;

#if SERIAL_DEBUG
		// Serial.print("event sent: ");
		// Serial.println(s);
		//
		Serial.print("Free heap: ");
		Serial.println(ESP.getFreeHeap());
#endif

    const float bat_volt_adc_raw = analogRead(BAT_VOLT_ADC_PIN);
    const float bat_volt_adc = bat_volt_adc_raw * 0.0067;
    bat_volt_adc_corrected = bat_volt_adc + (20. - bat_volt_adc) / 20.; // some ad-hoc correction of the horrible ADC nonlinearity
  }

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

#ifdef AUX_SERVO_CONTROLLER
	// set servo controllers
  aux_servo_1.write(aux_pos_1);
  aux_servo_2.write(aux_pos_2);
#endif


#if SERIAL_DEBUG_DETAILED
  // enabling this renders everything very, very slow
  Serial.print("vel_R = ");
  Serial.print(vel_R);
  Serial.print(". vel_L = ");
  Serial.print(vel_L);
  Serial.print(", vel_C = ");
  Serial.print(vel_C);
  Serial.println();
#if AUX_SERVO_CONTROLLER
  Serial.print("aux_servo_pos_1 = ");
  Serial.print(aux_pos_1);
  Serial.print(", aux_servo_pos_2 = ");
  Serial.print(aux_pos_2);
  Serial.println();
	Serial.print(", bat_volt_raw = ");
	Serial.print(bat_volt_adc_raw);
	Serial.println();
	Serial.printf("%f\n", bat_volt_adc_corrected);
#endif
#endif
#endif
}
