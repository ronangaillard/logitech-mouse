/*
 Copyright (C) 2017 Ronan Gaillard <ronan.gaillard@live.fr>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/

#ifndef LOGITECH_MOUSE
#define LOGITECH_MOUSE

#include "nRF24L01.h"
#include "RF24.h"

#define DEFAULT_CE_PIN 8
#define DEFAULT_CS_PIN 7
#define CHANNEL 5
#define PAYLOAD_SIZE 22
#define PAIRING_MAC_ADDRESS 0xBB0ADCA575LL
#define EEPROM_SUPPORT
#define MAC_ADDRESS_EEPROM_ADDRESS 0

#ifdef EEPROM_SUPPORT
#include <EEPROM.h>
#endif

class logiMouse
{
  private : 

  RF24 radio;

  void setChecksum(uint8_t *payload, uint8_t len);
  bool pairingStep(uint8_t *pairing_packet, uint8_t *pairing_packet_small, uint8_t *ack_payload, uint8_t timeout);
  void setAddress(uint8_t *address);
  void setAddress(uint64_t address);

  /* Pre-defined pairing packets */
  uint8_t pairing_packet_1[22] = {0x15, 0x5F, 0x01, 0x84, 0x5E, 0x3A, 0xA2, 0x57, 0x08, 0x10, 0x25, 0x04, 0x00, 0x01, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xEC};
  uint8_t pairing_packet_1_bis[5] = {0x15, 0x40, 0x01, 0x84, 0x26};
  uint8_t pairing_packet_2[22] = {0x00, 0x5F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x58, 0x8A, 0x51, 0xEA, 0x01, 0x07, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  uint8_t pairing_packet_2_bis[5] = {0x00, 0x40, 0x02, 0x01, 0xbd};
  uint8_t pairing_packet_3[22] = {0x00, 0x5F, 0x03, 0x01, 0x00, 0x04, 0x4D, 0x35, 0x31, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB6};
  uint8_t pairing_packet_3_bis[5] = {0x00, 0x5F, 0x03, 0x01, 0x0f};
  uint8_t pairing_packet_4[10] = {0x00, 0x0F, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEA};
  /* Enf of pre-defined pairing packets */

  public :

  logiMouse(uint8_t _cepin, uint8_t _cspin);
  logiMouse();

  bool begin();

  bool pair();
  bool pair(uint8_t timeout);
  bool reconnect();

  void move(uint16_t x_move, uint16_t y_move);
  void move(uint16_t x_move, uint16_t y_move, bool leftClick, bool rightClick);
  void move(uint16_t x_move, uint16_t y_move, uint8_t scroll_v, uint8_t scroll_h);
  void move(uint16_t x_move, uint16_t y_move, uint8_t scroll_v, uint8_t scroll_h, bool leftClick, bool rightClick);
  void click(bool leftClick, bool rightClick);
  void scroll(uint8_t scroll_v, uint8_t scroll_h);
  void scroll(uint8_t scroll_v);
};

#endif
