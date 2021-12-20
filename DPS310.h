#ifndef DPS310_H
#define DPS310_H

#include "DPS310.h"
#include "Arduino.h"
#include "Wire.h"

class DPS310 
{
    public:

        float altitude, temperature;
        float pressure;
        DPS310() {};

        bool init();

        void set_pressure_configs( byte config ) { write(119, 0x06, config); };
        void set_temp_configs ( byte config ) { write(119, 0x07, config); };
        void set_measure_configs ( byte config ) { write(119, 0x08, config); };
        void set_int_configs ( byte config ) { write(119, 0x09, config); };

        byte read_int_status() { read(119, 0x0a, 1); return(Wire.read()); };
        byte read_FIFO_status() { read(119, 0x0b, 1); return(Wire.read()); };

        void soft_reset();

        void read_alt();
        void read_alt_fast(); // uses optimized powf function to reduce runtime (and accuracy) by A LOT 

    private:
        int32_t _c0, _c1, _c00, _c10; // calibration values
        int16_t _c01, _c11, _c20, _c21, _c30;

        int32_t twosComplement(int32_t val, uint8_t bits);

        void read_calib();

        void write( byte addr, byte reg, byte data );
        void read( byte addr, byte reg, byte bytesToRead );

};

#endif
