#include <Arduino.h>

#define ARDUINO_UNO_R3

#ifdef ARDUINO_MEGA_2560 
#define PIN_BUZZER 9        // PWM
#define LED_RED 11          // PWM
#define LED_GREEN 12        // PWM
#define LED_BOARD 13
#define PIN_TRIGGER 16      // DigitalIo
#define PIN_ECHO 17         // DigitalIo
#elif defined(ARDUINO_UNO_R3) 
#define PIN_BUZZER 9
#define LED_RED 10
#define LED_GREEN 11
#define LED_BOARD 13
#define PIN_TRIGGER 2
#define PIN_ECHO 4
#endif


void initiateTrigger();
float calculatedistance(unsigned long time);
const float sound = 34300.0;  // Sound speech in cm/s
void pwm_output(float fraction);
float percentage(float in, float min, float max);
char print_buffer[256];
float delta = 0.1;
float distance;
unsigned char pwm_table[] = { 0b0, 0b10, 0b100, 0b111, 0b1011, 0b10010, 0b11110, 0b1000001, 0b01010000, 0b01100100, 0b01111101, 0b10100000, 0b11001000, 0b11111111 };
const char* line_graph = "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$";

float led_pwm_from_fraction(float percentage) {
  int index = percentage / (1. / 16.);
  pwm_table[index] / 255.;
}

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  initiateTrigger();
  unsigned long time = pulseIn(PIN_ECHO, HIGH, 150000);
  float distance = calculatedistance(time);
  pwm_output(percentage(distance, 0., 100.));
  Serial.print(String(distance) + "|"+ millis());
}

void initiateTrigger() {
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
}

float calculatedistance(unsigned long time) {
  float measurement = (time * 0.000001 * sound / 2.0);
  distance = measurement;
  return distance;
}

void pwm_output(float fraction) {

  int pos = strlen(line_graph) * (1. - fraction);
  Serial.println(&line_graph[pos]);
  analogWrite(LED_RED, (1 - fraction) * 255);
  analogWrite(LED_BOARD, (1 - fraction) * 255);
  analogWrite(LED_GREEN, fraction * 255);
  analogWrite(PIN_BUZZER,(1 - fraction) * 255);
}

float percentage(float in, float min, float max) {
  if (in < min) return 0;
  if (in > max) return 1.;
  return (in - min) / (max - min);
}
