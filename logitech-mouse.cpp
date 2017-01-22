/*
 Copyright (C) 2017 Ronan Gaillard <ronan.gaillard@live.fr>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/
 
#include "logitech-mouse.h"

logiMouse::logiMouse(uint8_t _cepin, uint8_t _cspin) : radio(_cepin, _cspin)
{

}

logiMouse::logiMouse() : logiMouse(DEFAULT_CE_PIN, DEFAULT_CS_PIN)
{

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
    radio.setRetries(3, 3);
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

void logiMouse::pairingStep(uint8_t *pairing_packet, uint8_t *pairing_packet_small, uint8_t *ack_payload)
{
    bool keep_going = true;

    while (keep_going)
    {
        if (radio.available())
            break;

        while (!radio.write(pairing_packet, 22, 0))
        {
        }

        if (radio.available())
            break;

        while (keep_going)
        {
            if (radio.write(pairing_packet_small, 5, 0))
                if (radio.available())
                    keep_going = false;
        }
    }

    radio.read(ack_payload, 22);
}

void logiMouse::pair()
{
    uint8_t buffer[22];

    /* Pairing step 1 */
    pairingStep(pairing_packet_1, pairing_packet_1_bis, buffer);

    /* Generate dongle and device address */
    uint8_t new_add[5];
    uint8_t new_add_dongle[5];

    for (int i = 0; i < 5; i++)
    {
        new_add[i] = buffer[3 + (4 - i)];
        new_add_dongle[i] = buffer[3 + (4 - i)];
    }

    new_add_dongle[0] = 0;

    /* Switch addresses */
    radio.stopListening();
    radio.openReadingPipe(2, new_add_dongle);
    radio.openReadingPipe(1, new_add);
    radio.openWritingPipe(new_add);

    /* Pairing step 2 */
    pairingStep(pairing_packet_2, pairing_packet_2_bis, buffer);
    
    /* Pairing step 3 */
    pairingStep(pairing_packet_3, pairing_packet_3_bis, buffer);

    /* Pairing step 4 */
    while (!radio.write(pairing_packet_4, 22, 0))
    {
    }
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

    // Send keepalive
    byte keepalive_payload[] = {0x00, 0x40, 0xff, 0xff, 0xc2};
    while (!radio.write(keepalive_payload, 5, 0))
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