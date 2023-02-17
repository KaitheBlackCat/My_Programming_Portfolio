//-------------------------------------------------------------------------------------
// CRC16 support class
// Based on various examples found on the web
// Copyright (C) 2014 Vincenzo Mennella (see license.txt)
// History
//  0.1.0 31/05/2014:   First public code release
//  0.1.1 17/12/2014:   Minor revision and commented code
//
// License
// "MIT Open Source Software License":
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------
#ifndef CRC16_H
#define CRC16_H
#define LIBRARY_VERSION_CRC16_H   "0.1.1"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Crc16 {
   private:
        //Crc parameters
        uint16_t _msbMask;
        uint16_t _mask;
        uint16_t _xorIn;
        uint16_t _xorOut;
        uint16_t _polynomial;
        uint8_t _reflectIn;
        uint8_t _reflectOut;
        //Crc value
    uint16_t _crc;
    uint8_t reflect(uint8_t data, uint8_t bits = 32);

   public:
        inline Crc16()
        {
            //Default to XModem parameters  //엑스모뎀 파라미터?
            _reflectIn = false;
            _reflectOut = false;
            _polynomial = 0x1021;
            _xorIn = 0x0000;
            _xorOut = 0x0000;
            _msbMask = 0x8000;
            _mask = 0xFFFF;
        }
        inline Crc16(uint8_t reflectIn, uint8_t reflectOut, uint16_t polynomial, uint16_t xorIn, uint16_t xorOut, uint16_t msbMask, uint16_t mask)
        {
            _reflectIn = reflectIn;
            _reflectOut = reflectOut;
            _polynomial = polynomial;
            _xorIn = xorIn;
            _xorOut = xorOut;
            _msbMask = msbMask;
            _mask = mask;
        }
    void clearCrc();
    void updateCrc(uint8_t data);
    uint16_t getCrc();
    unsigned int fastCrc(uint8_t data[], uint8_t start, uint16_t length, uint8_t reflectIn, uint8_t reflectOut, uint16_t polynomial, uint16_t xorIn, uint16_t xorOut, uint16_t msbMask, uint16_t mask);
    inline unsigned int XModemCrc(uint8_t data[], uint8_t start, uint16_t length)
    {
            //  XModem parameters: poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000
            return fastCrc(data, start, length, false, false, 0x1021, 0x0000, 0x0000, 0x8000, 0xffff);
    }
};

//---------------------------------------------------
// Initialize crc calculation
//---------------------------------------------------
void Crc16::clearCrc()
{
  _crc = _xorIn;
}
//---------------------------------------------------
// Update crc with new data
//---------------------------------------------------
void Crc16::updateCrc(uint8_t data)
{
  if (_reflectIn != 0)
    data = (uint8_t) reflect(data, 8);

  int j = 0x80;

  while (j > 0)
  {
    uint16_t bit = (uint16_t)(_crc & _msbMask);
    
    _crc <<= 1;

    if ((data & j) != 0)
    {
      bit = (uint16_t)(bit ^ _msbMask);
    }

    if (bit != 0)
    {
      _crc ^= _polynomial;
    }

    j >>= 1;
  }
}

//---------------------------------------------------
// Get final crc value
//---------------------------------------------------
uint16_t Crc16::getCrc()
{
  if (_reflectOut != 0)
    _crc = (unsigned int)((reflect(_crc) ^ _xorOut) & _mask);

  return _crc;
}

//---------------------------------------------------
// Calculate generic crc code on data array
// Examples of crc 16:
// Kermit:    width=16 poly=0x1021 init=0x0000 refin=true  refout=true  xorout=0x0000 check=0x2189
// Modbus:    width=16 poly=0x8005 init=0xffff refin=true  refout=true  xorout=0x0000 check=0x4b37
// XModem:    width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000 check=0x31c3
// CCITT-False: width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1
//---------------------------------------------------
//return fastCrc(data, start, length, false, false, 0x1021, 0x0000, 0x0000, 0x8000, 0xffff);
unsigned int Crc16::fastCrc(uint8_t data[], uint8_t start, uint16_t length, uint8_t reflectIn, uint8_t reflectOut, uint16_t polynomial, uint16_t xorIn, uint16_t xorOut, uint16_t msbMask, uint16_t mask)
{
  unsigned int crc = xorIn;      // xor in 변수를 부호가 없는 정수 crc로 지정합니다.

  int j;
  uint8_t c;
  unsigned int bit;             // 변수를 지정합니다.

  if (length == 0) return crc;   // 만약 문자열이 들어오지 않으면 xor in 값을 그대로 내보냅니다.

  for (int i = start; i < (start + length); i++)  //만약 i가 start라면, i가 start + length 값을 넘길 때까지 1씩 더합니다.
  {
    c = data[i];  // data의 i-1번째 문자열에 c를 집어넣습니다.

    if (reflectIn != 0)  // 만약 reflect in 값이 0이 아니라면 
      c = (uint8_t) reflect(c, 8); // c는 reflect 에 c와 8을 넣은 것과 같습니다.  -> 

    j = 0x80; //j값을 0x80으로 정합니다.

    while (j > 0) // j가 0보다 큰 와중에
    {
      bit = (unsigned int)(crc & msbMask); // bit 값을 crc와 msbMask 값이 같은지, 같지 않은지에 대한 값을(True, False) unsigned int로 임시 지정하여(1, 0) 나온 값으로 정합니다.
      crc <<= 1; //crc는 crc의 2진수 값을 왼쪽으로 1번 움직인 값으로 합니다.(2씩 커집니다)

      if ((c & j) != 0) // 만약 c, j가 둘 다 0이 아니라면
      {
        bit = (unsigned int)(bit ^ msbMask); //bit 값은 unsigned int로 임시지정된, bit와 msbMask값의 ^연산으로 정합니다.
      }

      if (bit != 0) // 만약 bit가 0이 아니라면
      {
        crc ^= polynomial; //crc는 crc와 polynomial의 ^연산한 결과로 합니다.
      }

      j >>= 1; //j에 2를 나눕니다.
    }
  }

  if (reflectOut != 0)
    crc = (unsigned int)((reflect(crc) ^ xorOut) & mask);

  return crc;
}

//-------------------------------------------------------
// Reflects bit in a uint8_t
//-------------------------------------------------------
uint8_t Crc16::reflect(uint8_t data, uint8_t bits)  // <- reflect에 c, 8 대입
{
  unsigned long reflection = 0x00000000;  // long 값 reflection에 0을 넣습니다.
  // Reflect the data about the center bit.
  for (uint8_t bit = 0; bit < bits; bit++) // 만약 bit값이 0이라면, bit값이 들어온 bits(여기선 8)을 넘을 때 까지 bit에 1씩 더합니다.
  {
    // If the LSB bit is set, set the reflection of it.
    if ((data & 0x01) != 0) // 만약 data 값과 0x01을 비교하여 같은 것은 1로, 다른 것은 0으로 한 값이 0이 아니면
    {
      reflection |= (unsigned long)(1 << ((bits - 1) - bit)); // reflection에 unsigned long으로 임시 지정된, bits에 1을 뺀 값과 bit의 차를 구하고, 1의 2진수 값을 왼쪽으로 차 만큼 움직입니다.
    }

    data = (uint8_t)(data >> 1);//데이터의 값을 unit8_t값으로 임시 지정된, data의 2진수를 오른쪽으로 1만큼 움직인 값으로 정합니다. (2씩 곱합니다)
  }

  return reflection;  //reflection을 반환합니다.
}
#endif
