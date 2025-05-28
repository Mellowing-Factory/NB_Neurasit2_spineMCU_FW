#ifndef _BLE_DATA_H
#define _BLE_DATA_H

#include "globals.h"

class BleDataConvert {
    public:
        BleDataConvert();
        ~BleDataConvert();

        void reset(void);
        uint8_t getSize(void);
        uint8_t *getBuffer(void);
        void addFactory();
        void addByte(uint8_t value);
        void addInt16(int16_t value);
        void addInt16Array(int16_t array[], uint8_t size);
        void addflag(uint8_t flag);
        void addString(String string, uint8_t size);
        void addmac(const uint8_t *value, uint8_t size);
        void addidV(uint8_t *value, uint8_t size);
        void addthingname(char *thingname, uint8_t size);
        void addssid(char *ssid, uint8_t size);
        void addpass(char *pass, uint8_t size);
        void addhead(uint8_t value);
        void addidvEncrypted(uint8_t idv1, uint8_t idv2);
        
        /* not used
        // void addmac(uint8_t *value, uint8_t size);
        //*/

    private:
        uint8_t buffer[BLE_DATA_MAXIMUM_SIZE];
        uint8_t cursor;
};

extern BleDataConvert bledata;
#endif