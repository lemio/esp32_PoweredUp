#include <PoweredUp.h>

PoweredUp hub(nullptr);

void setup() {
  Serial.begin(115200);
  hub.connect();
  hub.port('A').onTiltChanged([](int8_t x, int8_t y){
    Serial.printf("Tilt angle A: x=%d y=%d\n", x, y);
  });
  hub.port('B').onTiltChanged([](int8_t x, int8_t y){
    Serial.printf("Tilt angle B: x=%d y=%d\n", x, y);
  });
  hub.onDistanceChanged([](int8_t distance){
    Serial.printf("Distance: %d\n", distance);
  });
}

void loop() {
  hub.handleConnection();
}