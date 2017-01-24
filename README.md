# logitech-mouse
Use an arduino as a logitech wireless 🐭

We managed to make an Arduino act like a Logitech wireless mouse using a NRF24. This work is based on our retroengineering work done in December 2016 and January 2017 during our school project @Supelec.

## Example

This code makes the mouse move in circle on the screen.

```cpp
#include "logitech-mouse.h"

logiMouse myMouse;

float mouseSpeed = 10.0f;
float degreesToRadians = 2.0f*3.14f/360.0f;

void setup() {
  Serial.begin(115200);

  Serial.println("Starting");

  myMouse.begin();

  Serial.println("Trying to reconnect");

  if(myMouse.reconnect())
      Serial.println("Reconnected using previous configuration !");
  else
  {
      Serial.println("Unable to reconnect to dongle... Pairing with any dongle...");
      myMouse.pair();
  }
    
  Serial.println("Ready :)");
}

void loop() {
  int x, y = 0;

  for(x = 0; x < 360; x+=5) {
    myMouse.move( (uint16_t)(mouseSpeed * cos( ((float)x) * degreesToRadians ) ),  
      (uint16_t)(mouseSpeed * sin( ((float)x) * degreesToRadians ) ));
      
    delay(10);
  }
}
```

## Quick doc

The class we created is called `logiMouse`.

### Methods

#### General methods

- `bool begin()` : inits NRF24 module for wireless communication
- `bool pair()` : pairs Arduino with Logitech dongle (don't forget to put the dongle into pairing mode using the unifying desktop app), returns false if pairing failed
- `bool pair(uint8_t)` : pairs with timeout (exits if pairing takes too much time)
- `bool reconnect()` : reconnects to previously connected dongle (exits if dongle is not found), returns true if success

#### Move methods

- `void move(uint16_t x_move, uint16_t y_move)` : moves the mouse on the screen according to specified velocity (x,y)
- `void move(uint16_t x_move, uint16_t y_move, bool leftClick, bool rightClick)` : moves the mouse on the screen according to specified velocity (x,y), and click according to booleans
- `void move(uint16_t x_move, uint16_t y_move, uint8_t scroll_v, uint8_t scroll_h)` : moves and scrolls (see scrolls methods for more info)
- `void move(uint16_t x_move, uint16_t y_move, uint8_t scroll_v, uint8_t scroll_h, bool leftClick, bool rightClick)` : moves, scrolls and clicks (see related methods for more info)

#### Scroll methods

- `void scroll(uint8_t scroll_v, uint8_t scroll_h)` : scrolls horizontally and vertically (_scroll_v_ and _scroll_h_ can be negative)
- `void scroll(uint8_t scroll_v)` : scrolls vertically (_scroll_v_ can be negative)

#### Click methods

- `void click(bool leftClick, bool rightClick)` : clicks according to booleans


## Copyright

The NRF24 lib included in this repo was written by Coliz <maniacbug@ymail.com>, all rights reserved to him.

We are the author of both files : `logitech-mouse.cpp` and `logitech-mouse.h`

