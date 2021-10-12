/*
 * TR CAN Shield example sketch
 */

#include <TR_CAN_Shield.h>

TR_CAN_Shield canShield();

int sensor_value;

void setup() {

}

void loop() {
    sensor_value = canShield.analogRead(1);
    Serial.print("Sensor reading: ");
    Serial.print(sensor_value)
}
