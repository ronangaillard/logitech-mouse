/*
 Copyright (C) 2017 Ronan Gaillard <ronan.gaillard@live.fr>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/
 
#include "logitech-mouse.h"


#ifdef EEPROM_SUPPORT
#include <EEPROM.h>
#endif

logiMouse::logiMouse(uint8_t _cepin, uint8_t _cspin) : radio(_cepin, _cspin)
{

}

logiMouse::logiMouse() : logiMouse(DEFAULT_CE_PIN, DEFAULT_CS_PIN)
{

}

void logiMouse::setAddress(uint64_t address)
{
    setAddress((uint8_t *)&address);
}

void logiMouse::setAddress(uint8_t *address)
{
    uint8_t address_dongle[5];

    memcpy(address_dongle, address, 4);
    address_dongle[0] = 0;
    
    radio.stopListening();
    radio.openReadingPipe(2, address_dongle);
    radio.openReadingPipe(1, address);
    radio.openWritingPipe(address);
}

bool logiMouse::begin()
{
    uint8_t init_status = radio.begin();

    if (init_status == 0 || init_status == 0xff)
    {
        return false;
    }

    radio.stopListening();
    radio.setAutoAck(1);
    radio.setRetries(3, 1);
    radio.setPayloadSize(PAYLOAD_SIZE);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.enableDynamicAck();
    radio.openWritingPipe(PAIRING_MAC_ADDRESS);
    radio.openReadingPipe(1, PAIRING_MAC_ADDRESS);
    radio.setChannel(CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.stopListening();

    return true;
}

void logiMouse::setChecksum(uint8_t *payload, uint8_t len)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < (len - 1); i++)
        checksum += payload[i];

    payload[len - 1] = -checksum;
}

bool logiMouse::pairingStep(uint8_t *pairing_packet, uint8_t *pairing_packet_small, uint8_t *ack_payload, uint8_t timeout)
{
    bool keep_going = true;
    uint8_t loop_counter = timeout;

    while (keep_going && (timeout == 0 || loop_counter > 0))
    {
        if (radio.available())
            break;

        while (!radio.write(pairing_packet, 22, 0) && (timeout == 0 || loop_counter > 0))
            loop_counter--;

        if (radio.available())
            break;

        while (keep_going && (timeout == 0 || loop_counter > 0))
        {
            if (radio.write(pairing_packet_small, 5, 0))
                if (radio.available())
                    keep_going = false;

            loop_counter--;
        }
    }

    radio.read(ack_payload, 22);

    return (timeout == 0 || loop_counter > 0);
}

bool logiMouse::pair()
{
    return pair(0);
}

bool logiMouse::pair(uint8_t timeout)
{
    uint8_t buffer[22];
    uint8_t loop_counter = timeout;

    /* Pairing step 1 */
    if(!pairingStep(pairing_packet_1, pairing_packet_1_bis, buffer, timeout))
        return false;

    /* Generate dongle and device address */
    uint8_t new_add[5];

    for (int i = 0; i < 5; i++)
        new_add[i] = buffer[3 + (4 - i)];

    setAddress(new_add);

    /* Pairing step 2 */
    if(!pairingStep(pairing_packet_2, pairing_packet_2_bis, buffer, timeout))
        return false;
    
    /* Pairing step 3 */
    if(!pairingStep(pairing_packet_3, pairing_packet_3_bis, buffer, timeout))
        return false;

    /* Pairing step 4 */
    while (!radio.write(pairing_packet_4, 22, 0) && (timeout == 0 || loop_counter > 0) )
        loop_counter--;

    if(!(timeout == 0 || loop_counter > 0))
        return false;

    #ifdef EEPROM_SUPPORT
    /* Save address to eeprom */
    for (int i = 0; i < 5; i++)
    {
        EEPROM.write(MAC_ADDRESS_EEPROM_ADDRESS + i, new_add[i]);
    }
    #endif
}

bool logiMouse::reconnect()
{
  #ifndef EEPROM_SUPPORT
  #warning "EEPROM support is not enabled"
  return false;
  #else
  uint8_t new_add[5];

  for (int i = 0; i < 5; i++)
      new_add[i] = EEPROM.read(MAC_ADDRESS_EEPROM_ADDRESS + i);
      
  setAddress(new_add);

  if(pair(255))
      return true;
  else {
      setAddress(PAIRING_MAC_ADDRESS);
      return false;
  }

  #endif
}

void logiMouse::move(uint16_t x_move, uint16_t y_move)
{
    move(x_move, y_move, false, false);
}

void logiMouse::move(uint16_t x_move, uint16_t y_move, bool leftClick, bool rightClick)
{
    move(x_move, y_move, 0, 0, false, false);
}

void logiMouse::move(uint16_t x_move, uint16_t y_move, uint8_t scroll_v, uint8_t scroll_h)
{
    move(x_move, y_move, scroll_v, scroll_h, false, false);
}

void logiMouse::move(uint16_t x_move, uint16_t y_move, uint8_t scroll_v, uint8_t scroll_h, bool leftClick, bool rightClick)
{
    uint8_t mouse_payload[] = {0x00, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint32_t cursor_velocity;

    cursor_velocity = ((uint32_t)y_move & 0xFFF) << 12 | (x_move & 0xFFF);

    memcpy(mouse_payload + 4, &cursor_velocity, 3);

    if(leftClick)
        mouse_payload[2] = 1;

    if(rightClick)
        mouse_payload[2] |= 1 << 1;

    mouse_payload[7] = scroll_v;
    mouse_payload[8] = scroll_h;

    setChecksum(mouse_payload, 10);
    while (!radio.write(mouse_payload, 10, 0))
    {
    }
    

    radio.flush_rx();
}

void logiMouse::click(bool leftClick, bool rightClick) {
    move(0, 0, leftClick, rightClick);
}

void logiMouse::scroll(uint8_t scroll_v, uint8_t scroll_h)
{
     move(0, 0, scroll_v, scroll_h, false, false);
}

void logiMouse::scroll(uint8_t scroll_v)
{
    scroll(scroll_v, 0);
}
