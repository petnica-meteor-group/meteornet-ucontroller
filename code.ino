#include <DHT.h>
#include <Servo.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "serial_com.h"
#include "com_protocol.h"
}

const char* UCONTROLLER_NAME = "Camera controller & PSU v1.1";

#define DHTTYPE DHT22
const int DHTPIN = A4;
const char* DHT_HUM_STRING = "Humidity:";
const char* DHT_HUM_UNIT = "%";
const char* DHT_TEMP_STRING = "Temperature:";
const char* DHT_TEMP_UNIT = "C";

const int SHUTTER_SERVO_PIN = A5;
const int SHUTTER_ANGLE_OPEN = 0;
const int SHUTTER_ANGLE_CLOSED = 180;
const int SHUTTER_ANGLE_DELTA = 2;
const int SHUTTER_ANGLE_DELAY = 20;

const int CAMERA_SWITCH_PIN = A3;
const int CAMERA_SWITCH_DELAY = 300;

const int CAMERA_VOLTAGE_PIN = A1;
const float CAMERA_VOLTAGE_CONVERSION_FACTOR = 12.06 / 692.0;
const char* CAMERA_VOLTAGE_STRING = "Camera voltage:";
const char* CAMERA_VOLTAGE_UNIT = "V";

const int PSU_STATUS_PIN = A0;
const char* PSU_STATUS_STRING = "PSU:";
const char* PSU_STATUS_ON = "on";
const char* PSU_STATUS_OFF = "off";

const char* END_STRING = "END";
char BUFFER_STRING[64];

DHT dht(DHTPIN, DHTTYPE);

Servo shutter_servo;
int shutter_servo_angle;

void string_send(const char* string) {
    unsigned i, j;
    uint32_t msg;
    uint8_t *current_char, done = 0;
    for (i = 0;; i += sizeof(msg)) {
        for (j = 0, current_char = (uint8_t*)&msg; j < sizeof(msg); j++, current_char++) {
            if (done || string[i + j] == 0) {
                *current_char = 0;
                done = 1;
            } else {
                *current_char = string[i + j];
            }
        }
        serial_send(0, msg);
        if (done) break;
    }
}

void shutter_move_to(int target_angle) {
    while (shutter_servo_angle != target_angle) {
        if (fabs(shutter_servo_angle - target_angle) > SHUTTER_ANGLE_DELTA) {
            shutter_servo_angle += SHUTTER_ANGLE_DELTA * (target_angle > shutter_servo_angle ? 1 : -1);
        } else {
            shutter_servo_angle = target_angle;
        }

        shutter_servo.write(shutter_servo_angle);
        delay(SHUTTER_ANGLE_DELAY);
    }
}

inline void shutter_open() {
    shutter_move_to(SHUTTER_ANGLE_OPEN);
}

inline void shutter_close() {
    shutter_move_to(SHUTTER_ANGLE_CLOSED);
}

inline void camera_turn_on() {
    digitalWrite(CAMERA_SWITCH_PIN, HIGH);
}

inline void camera_turn_off() {
    digitalWrite(CAMERA_SWITCH_PIN, LOW);
}

inline void dht_info_send() {
    float hum = dht.readHumidity();
    strcpy(BUFFER_STRING, DHT_HUM_STRING);
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%d.%04d", (int)hum, (int)((hum - (int)hum) * 10000));
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%s", DHT_HUM_UNIT);
    string_send(BUFFER_STRING);

    float temp = dht.readTemperature();
    strcpy(BUFFER_STRING, DHT_TEMP_STRING);
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%d.%04d", (int)temp, (int)((temp - (int)temp) * 10000));
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%s", DHT_TEMP_UNIT);
    string_send(BUFFER_STRING);
}

inline void psu_status_send() {
    int status = (digitalRead(PSU_STATUS_PIN) == HIGH);
    strcpy(BUFFER_STRING, PSU_STATUS_STRING);
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%s", status ? PSU_STATUS_ON : PSU_STATUS_OFF);
    string_send(BUFFER_STRING);
}

inline void camera_voltage_send() {
    int pin_read = analogRead(CAMERA_VOLTAGE_PIN);
    float voltage = pin_read * CAMERA_VOLTAGE_CONVERSION_FACTOR;
    strcpy(BUFFER_STRING, CAMERA_VOLTAGE_STRING);
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%d.%04d", (int)voltage, (int)((voltage - (int)voltage) * 10000));
    sprintf(BUFFER_STRING + strlen(BUFFER_STRING), "%s", CAMERA_VOLTAGE_UNIT);
    string_send(BUFFER_STRING);
}

void setup() {
    pinMode(CAMERA_SWITCH_PIN, OUTPUT);
    pinMode(CAMERA_VOLTAGE_PIN, INPUT);
    pinMode(PSU_STATUS_PIN, INPUT);

    dht.begin();

    shutter_servo.attach(SHUTTER_SERVO_PIN);
    shutter_servo_angle = shutter_servo.read();

    serial_init();

    shutter_open();
    camera_turn_on();
}

void loop() {
    uint32_t command;
    serial_receive(0, &command);
    switch (command) {
        case NIGHT: {
            string_send(END_STRING);
            shutter_open();
            delay(CAMERA_SWITCH_DELAY);
            camera_turn_on();
            break;
        }
        case DAY: {
            string_send(END_STRING);
            camera_turn_off();
            delay(CAMERA_SWITCH_DELAY);
            shutter_close();
            break;
        }
        case NAME_GET: {
            string_send(UCONTROLLER_NAME);
            string_send(END_STRING);
            break;
        }
        case MEASUREMENTS_GET: {
            dht_info_send();
            camera_voltage_send();
            psu_status_send();
            string_send(END_STRING);
            break;
        }
    }
}
