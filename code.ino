#include <DHT.h>
#include <Servo.h>
#include <math.h>

extern "C" {
#include "serial_com.h"
#include "com_protocol.h"
}

#define DHTTYPE DHT22
const int DHTPIN = 14;

const int SHUTTER_SERVO_PIN = 16;
const int SHUTTER_ANGLE_OPEN = 0;
const int SHUTTER_ANGLE_CLOSED = 180;
const int SHUTTER_ANGLE_DELTA = 2;
const int SHUTTER_ANGLE_DELAY = 20;

const int CAMERA_SWITCH_PIN = 18;

DHT dht(DHTPIN, DHTTYPE);
Servo shutter_servo;
int shutter_servo_angle;

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

inline void dht_info_get(float *hum, float* temp) {
    *hum = dht.readHumidity();
    *temp = dht.readTemperature();
}

inline void dht_info_send(float hum, float temp) {
    serial_send(*(uint32_t*)&hum);
    serial_send(*(uint32_t*)&temp);
}

void setup() {
    pinMode(CAMERA_SWITCH_PIN, OUTPUT);

    dht.begin();

    shutter_servo.attach(SHUTTER_SERVO_PIN);
    shutter_servo_angle = shutter_servo.read();

    serial_init();
}

void loop() {
    uint32_t command;
    serial_receive(&command);
    switch (command) {
        case SHUTTER_OPEN: {
            shutter_open();
            break;
        }
        case SHUTTER_CLOSE: {
            shutter_close();
            break;
        }
        case CAMERA_TURN_ON: {
            camera_turn_on();
            break;
        }
        case CAMERA_TURN_OFF: {
            camera_turn_off();
            break;
        }
        case DHT_INFO_GET: {
            float hum;
            float temp;
            dht_info_get(&hum, &temp);
            dht_info_send(hum, temp);
            break;
        }
    }
}
