#include "bleData.h"
// #include "bleByte.h"
#include "globals.h"

static const char TAG[] = __FILE__;

BleDataConvert::BleDataConvert()
{
    cursor = 0;
}

BleDataConvert::~BleDataConvert(void)
{
}

void BleDataConvert::reset(void)
{
    cursor = 0;
}

uint8_t BleDataConvert::getSize(void)
{
    return cursor;
}

uint8_t *BleDataConvert::getBuffer(void)
{
    return buffer;
}

void BleDataConvert::addFactory()
{
    buffer[cursor++] = (0x59);
    buffer[cursor++] = (0x00);
}

void BleDataConvert::addByte(uint8_t value)
{
    buffer[cursor++] = (value);
}

void BleDataConvert::addInt16(int16_t value)
{
    buffer[cursor++] = (value >> 8) & 0xFF;
    buffer[cursor++] = value & 0xFF;
}

void BleDataConvert::addInt16Array(int16_t array[], uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[cursor++] = (array[i] >> 8)  & 0xFF;
        buffer[cursor++] = array[i] & 0xFF;
    }
}

void BleDataConvert::addString(String string, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[cursor++] = (string[i]);
    }
}

void BleDataConvert::addflag(uint8_t flag)
{
    uint8_t value = flag;
    value |= (((byte)REV_CODE) << 3);
    value |= (((byte)FV_CODE) << 1);
    buffer[cursor++] = (value);
}

void BleDataConvert::addmac(const uint8_t *value, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[cursor++] = (value[i]);
    }
}

void BleDataConvert::addidV(uint8_t *value, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[cursor++] = (value[i]);
    }
}

void BleDataConvert::addthingname(char *thingname, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[cursor++] = (thingname[i]);
    }
}

void BleDataConvert::addhead(uint8_t value)
{
    buffer[cursor++] = value;
}