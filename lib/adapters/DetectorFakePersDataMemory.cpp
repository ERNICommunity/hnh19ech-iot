/*
 * DetectorIvmMemory.cpp
 *
 *  Created on: 27.08.2018
 *      Author: niklaudi
 */

#include <Adafruit_FRAM_I2C.h>
#include <LoRaWanDriver.h>
#include <DetectorFakePersDataMemory.h>

const unsigned int DetectorFakePersDataMemory::s_numDevices  = 10;
const unsigned int DetectorFakePersDataMemory::s_numMaxChars = 32;

//-----------------------------------------------------------------------------
DetectorFakePersDataMemory::DetectorFakePersDataMemory(Adafruit_FRAM_I2C* fram, LoRaWanDriver* loraWanDriver /*= 0*/)
: m_deviceSerialNr(0)
, m_fram(fram)
, m_loraWanDriver(loraWanDriver)
{ }

DetectorFakePersDataMemory::~DetectorFakePersDataMemory()
{ }

void DetectorFakePersDataMemory::assignLoRaWanDriver(LoRaWanDriver* loraWanDriver)
{
  m_loraWanDriver = loraWanDriver;
}

void DetectorFakePersDataMemory::write(unsigned int address, unsigned char data)
{
  // Dummy write - no action
}

char DetectorFakePersDataMemory::read(unsigned int address)
{
  char value = 0;
  if (0 != m_fram)
  {
	value = m_fram->read8(address);
//    value = defaultChar(address);
  }
  return value;
}

const char DetectorFakePersDataMemory::defaultChar(unsigned int address) const
{
  if (m_deviceSerialNr < s_numDevices)
  {
    const char deviceSpecificKeys[s_numDevices][KT_NumKeys][s_numMaxChars+1] =
    {
      { // EDDCH1901 LoRaWan Session - Node #0
        { "edd1901-node-00009876b6106f1e" },        // deviceId
        { "26011870" },                             // devAddr
        { "A7077A8D6EECAFC4ADAF47D1EADCCF16" },     // nwkSKey
        { "337992C476DC574EDBEADAB7D80E5B19" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #1
        { "edd1901-node-00009876b6106f27" },        // deviceId
        { "26011BD0" },                             // devAddr
        { "750250E8708AF60E6981329E6DE411F1" },     // nwkSKey
        { "25604D593C2A49F8375E88A49D1ED362" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #2
        { "edd1901-node-00009876b610bdd2" },        // deviceId
        { "2601187B" },                             // devAddr
        { "A22B318917667D8F8408C6BE58688620" },     // nwkSKey
        { "9934CE8575A43B2FF7B841EF5021D360" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #3
        { "edd1901-node-00009876b610bf1f" },        // deviceId
        { "260115D8" },                             // devAddr
        { "7BF9177A68A0F136443A321EC8F4586C" },     // nwkSKey
        { "0CD311388C74902586C97FC5857AABA6" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #4
        { "edd1901-node-00009876b610bf99" },        // deviceId
        { "26011ED7" },                             // devAddr
        { "9A6FE68A02DD53A3E4D0CA9551619E62" },     // nwkSKey
        { "A8055F943ECBBF777428D40922F44C80" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #5
        { "edd1901-node-00009876b610ffd6" },        // deviceId
        { "26011B35" },                             // devAddr
        { "63305D9B09BFB07A4B929377A67F17DD" },     // nwkSKey
        { "23F9802ED6A8F1BC8D23BE0A2A001087" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #6
        { "edd1901-node-00009876b61108ff" },        // deviceId
        { "26011ECF" },                             // devAddr
        { "BEF79FD4BB54C59013A6C1340E7964CF" },     // nwkSKey
        { "964A097B22F27987181F18554E2E54F1" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #7
        { "edd1901-node-00009876b6110978" },        // deviceId
        { "26011C6A" },                             // devAddr
        { "9F8A48047DA3E7C267E63E872949A0EE" },     // nwkSKey
        { "A0C1CA68BB47B59437FE2EDB0BDB914D" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #8
        { "edd1901-node-00009876b61109d7" },        // deviceId
        { "26011447" },                             // devAddr
        { "6FBBECACCED2B0CDF9FE14B361830B3F" },     // nwkSKey
        { "1F2112E9278F85EFDC0DC9ED184587FA" }      // appSKey
      },
      { // EDDCH1901 LoRaWan Session - Node #9
        { "edd1901-node-00009876b6110a1a" },        // deviceId
        { "2601147C" },                             // devAddr
        { "B3AC92FAB637698F06555E0771963D4B" },     // nwkSKey
        { "72CCA339009E9E686C3BEB6750FA659E" }      // appSKey
      }
    };
    unsigned int key = address / s_numMaxChars;
    unsigned int ch  = address % s_numMaxChars;
    return deviceSpecificKeys[this->m_deviceSerialNr][key][ch];
  }
  else
  {
    return 0;
  }
}

void DetectorFakePersDataMemory::initPersStorage()
{
  if (0 != m_fram)
  {
    const unsigned int maxChars = s_numDevices * KT_NumKeys * (s_numMaxChars + 1);

    for (unsigned int addr = 0; addr < maxChars; addr++)
    {
      m_fram->write8(addr, 0);
    }

    for (unsigned int addr = 0; addr < maxChars; addr++)
    {
      m_fram->write8(addr, defaultChar(addr));
    }

    if (0 != m_loraWanDriver)
    {
    	m_loraWanDriver->configure(true);
    }
  }
}

void DetectorFakePersDataMemory::setDeviceSerialNr(unsigned long int deviceSerialNr)
{
  m_deviceSerialNr = deviceSerialNr;
  initPersStorage();
}

unsigned int DetectorFakePersDataMemory::sizeOfDeviceKeyStorage()
{
  return s_numDevices * KT_NumKeys * s_numMaxChars;
}
