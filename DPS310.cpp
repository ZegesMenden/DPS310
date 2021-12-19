#include "DPS310.h"
#include "Arduino.h"
#include "Wire.h"

// thank you elvin from BPS

float altLUT[] = {0.        , 0.41629385, 0.47499169, 0.5130931 , 0.54196597,
       0.56547576, 0.58543972, 0.6028679 , 0.61838369, 0.63240071,
       0.64520838, 0.65701764, 0.66798728, 0.67824003, 0.68787284,
       0.69696371, 0.70557638, 0.71376365, 0.72156982, 0.72903237,
       0.73618338, 0.7430505 , 0.74965775, 0.75602613, 0.76217412,
       0.76811807, 0.77387253, 0.77945047, 0.78486357, 0.79012234,
       0.79523626, 0.80021397, 0.80506332, 0.80979148, 0.81440501,
       0.81890995, 0.82331185, 0.82761583, 0.83182663, 0.83594864,
       0.83998594, 0.84394232, 0.84782133, 0.85162627, 0.85536021,
       0.85902606, 0.86262654, 0.86616419, 0.8696414 , 0.87306045,
       0.87642346, 0.87973244, 0.88298929, 0.88619582, 0.88935373,
       0.89246464, 0.89553009, 0.89855152, 0.90153034, 0.90446786,
       0.90736533, 0.91022397, 0.91304491, 0.91582924, 0.91857802,
       0.92129224, 0.92397285, 0.92662078, 0.9292369 , 0.93182205,
       0.93437704, 0.93690264, 0.93939961, 0.94186865, 0.94431045,
       0.94672568, 0.94911498, 0.95147895, 0.9538182 , 0.95613328,
       0.95842476, 0.96069316, 0.962939  , 0.96516277, 0.96736495,
       0.96954601, 0.97170638, 0.97384651, 0.97596682, 0.9780677 ,
       0.98014956, 0.98221277, 0.98425771, 0.98628472, 0.98829416,
       0.99028637, 0.99226167, 0.99422038, 0.99616281, 0.99808925,
       1.        , 1.00189534, 1.00377555, 1.00564089, 1.00749162,
       1.00932801, 1.01115028, 1.01295869, 1.01475346, 1.01653483,
       1.01830301};

float fastPow(float P) {
  int idx = int(P * 100.f);
  float diff = P * 100.f - idx;
  if (idx >= 110) {
    return 1.01830301;
  }
  return altLUT[idx] * (1 - diff) + diff * altLUT[idx+1];
}

void DPS310::write( byte addr, byte reg, byte data ) 
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission(true);
}   

void DPS310::read ( byte addr, byte reg, byte bytesToRead )
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(119, bytesToRead, true);
}

// ty adafruit
int32_t DPS310::twosComplement(int32_t val, uint8_t bits) {
  if (val & ((uint32_t)1 << (bits - 1))) {
    val -= (uint32_t)1 << bits;
  }
  return val;
}

bool DPS310::init()
{

    Wire.beginTransmission(119);
    Wire.write(0x0d);
    Wire.endTransmission(false);
    Wire.requestFrom(119, 1, true);
    if ( Wire.read() != 0x10 ) { return false; }

    soft_reset();

    delay(100);

    read_calib();

    return true;

}

// ty adafruit (pt2)

void DPS310::read_calib() {

    byte coeffs[18];

    read(119, 0x10, 18);

    coeffs[0] = Wire.read();
    coeffs[1] = Wire.read();
    coeffs[2] = Wire.read();
    coeffs[3] = Wire.read();
    coeffs[4] = Wire.read();
    coeffs[5] = Wire.read();
    coeffs[6] = Wire.read();
    coeffs[7] = Wire.read();
    coeffs[8] = Wire.read();
    coeffs[9] = Wire.read();
    coeffs[10] = Wire.read();
    coeffs[11] = Wire.read();
    coeffs[12] = Wire.read();
    coeffs[13] = Wire.read();
    coeffs[14] = Wire.read();
    coeffs[15] = Wire.read();
    coeffs[16] = Wire.read();
    coeffs[17] = Wire.read();

    _c0 = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
    _c0 = twosComplement(_c0, 12);

    _c1 = twosComplement((((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2], 12);

    _c00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
         (((uint32_t)coeffs[5] >> 4) & 0x0F);
    _c00 = twosComplement(_c00, 20);

    _c10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
            (uint32_t)coeffs[7];
    _c10 = twosComplement(_c10, 20);

    _c01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
    _c11 = twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
    _c20 = twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
    _c21 = twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
    _c30 = twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);

}

void DPS310::soft_reset() 
{ 
    write(119, 0x0c, 0b00001001); 
    delay(12);

    while (1) { // wait for sensor initialization to finish

        Wire.beginTransmission(119);
        Wire.write(0x08);
        Wire.endTransmission(false);
        Wire.requestFrom(119, 1, true);
        
        byte meas_cfg = Wire.read();

        if ( bitRead(meas_cfg, 6) == 1 ) { break; }

    }

}

void DPS310::read_alt()
{

    read(119, 0x00, 6);

    uint32_t pressure_uncomp_int = twosComplement((uint32_t)(Wire.read()|Wire.read()<<8|Wire.read()<<16), 24);
    uint32_t temp_uncomp_int = twosComplement((uint32_t)(Wire.read()|Wire.read()<<8|Wire.read()<<16), 24);

    float pressure_uncomp = (float)pressure_uncomp_int / 1040384.0f;
    float temp_uncomp = (float)temp_uncomp_int / 1040384.0f;

    float pressure_comp =
      (int32_t)_c00 +
      pressure_uncomp * ((int32_t)_c10 +
                   pressure_uncomp * ((int32_t)_c20 + pressure_uncomp * (int32_t)_c30)) +
      temp_uncomp *
          ((int32_t)_c01 +
           pressure_uncomp * ((int32_t)_c11 + pressure_uncomp * (int32_t)_c21));
    temperature = _c0 * (float)0.5 + _c1 * temp_uncomp;

    altitude = 44330 * (1.0f - powf((pressure_comp / 100) / 1013.529f, 0.1903f));
}

void DPS310::read_alt_fast()
{
    read(119, 0x00, 6);

    uint32_t pressure_uncomp_int = twosComplement((uint32_t)(Wire.read()|Wire.read()<<8|Wire.read()<<16), 24);
    uint32_t temp_uncomp_int = twosComplement((uint32_t)(Wire.read()|Wire.read()<<8|Wire.read()<<16), 24);

    float pressure_uncomp = (float)pressure_uncomp_int / 1040384.0f;
    float temp_uncomp = (float)temp_uncomp_int / 1040384.0f;

    float pressure_comp =
      (int32_t)_c00 +
      pressure_uncomp * ((int32_t)_c10 +
                   pressure_uncomp * ((int32_t)_c20 + pressure_uncomp * (int32_t)_c30)) +
      temp_uncomp *
          ((int32_t)_c01 +
           pressure_uncomp * ((int32_t)_c11 + pressure_uncomp * (int32_t)_c21));
    temperature = _c0 * (float)0.5 + _c1 * temp_uncomp;

    altitude = 44330 * (1.0f - fastPow((pressure_comp / 100) / 1013.529f));
}