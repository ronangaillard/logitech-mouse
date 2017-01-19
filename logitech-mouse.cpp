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

void logiMouse::pair()
{
    uint8_t buffer[22];

    /* Pairing step 1 */
    while (1)
    {
        delay(10);
        radio.write(pairing_packet_1, 22, 1);
        if (radio.write(pairing_packet_1_bis, 5, 0))
            if (radio.available())
                break;
    }

    radio.read(buffer, 22);

    /* Generate dongle and device address */
    byte new_add[5];
    byte new_add_dongle[5];

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

    bool keep_going = true;

    /* Pairing step 2 */
    while (keep_going)
    {

        if (radio.available())
            break;

        while (!radio.write(pairing_packet_2, 22, 0))
        {
        }

        if (radio.available())
            break;

        while (keep_going)
        {
            if (radio.write(pairing_packet_2_bis, 5, 0))
                if (radio.available())
                    keep_going = false;
        }
    }

    radio.read(buffer, 22);

    keep_going = true;

    /* Pairing step 3 */
    while (keep_going)
    {

        if (radio.available())
            break;

        while (!radio.write(pairing_packet_3, 22, 0))
        {
        }

        if (radio.available())
            break;

        while (keep_going)
        {
            if (radio.write(pairing_packet_3_bis, 5, 0))
                if (radio.available())
                    keep_going = false;
        }
    }

    radio.read(buffer, 22);

    /* Pairing step 4 */
    while (!radio.write(pairing_packet_4, 22, 0))
    {
    }
}

void logiMouse::move(uint16_t x_move, uint16_t y_move)
{
    byte mouse_payload[] = {0x00, 0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint32_t cursor_velocity;

    cursor_velocity = ((uint32_t)y_move & 0xFFF) << 12 | (x_move & 0xFFF);

    memcpy(mouse_payload + 4, &cursor_velocity, 3);

    setChecksum(mouse_payload, 10);

    while (!radio.write(mouse_payload, 10, 0))
    {
    }

    radio.flush_rx();
}