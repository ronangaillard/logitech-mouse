# logitech-mouse
Use an arduino as a logitech wireless 🐭

We managed to make an Arduino act like a Logitech wireless mouse using a NRF24. This work is based on our retroengineering work done in December 2016 and January 2017 during our school project @Supelec.

## Example

This code makes the mouse move in circle on the screen.

```cpp
#include "logitech-mouse.h"

logiMouse myMouse;

float mouseSpeed = 10.0f;
float degreestofloat = 2.0f*3.14f/360.0f;

void setup() {
  Serial.begin(115200);

  Serial.println("Starting");

  myMouse.begin();

  Serial.println("Pairing");

  myMouse.pair();

  Serial.println("Done :)");
}

void loop() {
  int x, y = 0;

  for(x = 0; x < 360; x+=5) {
    myMouse.move( (uint16_t)(mouseSpeed * cos( ((float)x) * degreestofloat ) ),  (uint16_t)(mouseSpeed * sin( ((float)x) * degreestofloat ) ));
    delay(10);
  }
}
```

## Quick doc

The class we created is called `logiMouse`.

### Methods

- `bool begin()` : inits NRF24 module for wireless communication
- `void pair()` : pairs Arduino with Logitech dongle (don't forget to put the dongle into pairing mode using the unifying desktop app)
- `void move(uint16_t x, uint16_t y)` : moves the mouse on the screen according to specified velocity (x,y)
- `void move(uint16_t x, uint16_t y, bool leftClick, bool rightClick)` : moves the mouse on the screen according to specified velocity (x,y), and click according to booleans
- `void click(bool leftClick, bool rightClick)` : clicks according to booleans

## Copyright

The NRF24 lib included in this repo was written by Coliz <maniacbug@ymail.com>, all rights reserved to him.

We are the author of both files : `logitech-mouse.cpp` and `logitech-mouse.h`

