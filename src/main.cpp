/*
 * wiring-skeleton.cpp
 *
 *  Created on: 15.03.2017
 *      Author: niklausd
 */

#include <Arduino.h>
#include <Wire.h>

// PlatformIO libraries
#include <SerialCommand.h>  // pio lib install 173, lib details see https://github.com/kroimon/Arduino-SerialCommand
#include <SpinTimer.h>      // pio lib install 11599, lib details see https://github.com/dniklaus/spin-timer
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_FRAM_I2C.h>

// private libraries
#include <ProductDebug.h>
#include <Assets.h>
#include <MyDeviceSerialNrAdapter.h>
#include <DetectorFakePersDataMemory.h>
#include <Battery.h>
#include <MyBatteryAdapter.h>
#include <LoraWanAbp.h>
#include <LoRaWanDriver.h>
#include <MyLoRaWanConfigAdapter.h>
#include <Indicator.h>
#include <MyBuiltinLedIndicatorAdapter.h>
#include <LoRaWanRxDataToStatusLedAdapter.h>
#include <ILoraWanTxDataEventAdapter.h>
#include <ILoraWanRxDataEventAdapter.h>
#include <LoraMessage.h>
#include <DbgTracePort.h>
#include <DbgTraceLevel.h>

LoRaWanDriver* loRaWanInterface = 0;
// Pin mapping
//#if defined(ARDUINO_SAMD_FEATHER_M0)
const lmic_pinmap lmic_pins = LmicPinMap_AdafruitFeatherM0();
//#elif defined (__arm__) && defined (__SAM3X8E__)              // Arduino Due => Dragino Shield
//const lmic_pinmap lmic_pins = LmicPinMap_DraginoShield();
//#elif defined (__avr__)                                       // Arduino Uno or Mega 2560 => Dragino Shield
//const lmic_pinmap lmic_pins = LmicPinMap_DraginoShield();
//#endif

#ifndef BUILTIN_LED
#define BUILTIN_LED 13
#endif

#define DHTPIN      12        // Pin which is connected to the DHT sensor.
#define DHTTYPE     DHT22     // DHT 22 (AM2302)

SerialCommand* sCmd = 0;
Assets* assets = 0;
Battery* battery = 0;
Indicator* statusLed = 0;

//-----------------------------------------------------------------------------

class LoRaWanTxDataRequester : public ILoraWanTxDataEventAdapter
{
private:
  LoRaWanDriver*  m_loraDriver;
  DHT_Unified*    m_dht;

public:
  LoRaWanTxDataRequester(LoRaWanDriver* loRaWanDriver)
  : m_loraDriver(loRaWanDriver)
  , m_dht(new DHT_Unified(DHTPIN, DHTTYPE))
  { }

  virtual ~LoRaWanTxDataRequester()
  {
    delete m_dht;
    m_dht = 0;
  }

  void messageTransmitted()
  {
    sensors_event_t event;
    LoraMessage loRaMessage;


    // Get temperature event and pass its value to the LoraMessage.
    m_dht->temperature().getEvent(&event);
    if (!isnan(event.temperature))
    {
      loRaMessage.addTemperature(event.temperature);
    }

    // Get humidity event and pass its value to the LoraMessage.
    m_dht->humidity().getEvent(&event);
    if (!isnan(event.relative_humidity))
    {
      loRaMessage.addHumidity(event.relative_humidity);
    }

    TR_PRINTF(m_loraDriver->trPort(), DbgTrace_Level::debug, "Temperature: %f *C, Humidity: %f", event.temperature, event.relative_humidity);

    m_loraDriver->setPeriodicMessageData(loRaMessage.getBytes(), loRaMessage.getLength());
  }
};

//-----------------------------------------------------------------------------

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 0);

  delay(5000);

  setupProdDebugEnv();

  if (fram.begin())
  {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println("Found I2C FRAM");
  }
  else
  {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }
 
  //-----------------------------------------------------------------------------
  // Assets (inventory and persistent data)
  //-----------------------------------------------------------------------------
  DetectorFakePersDataMemory* detectorPersDataMemory = new DetectorFakePersDataMemory(&fram);
  assets = new Assets(new MyDeviceSerialNrAdapter(), detectorPersDataMemory);

  //-----------------------------------------------------------------------------
  // Battery Voltage Surveillance
  //-----------------------------------------------------------------------------
  BatteryThresholdConfig battCfg = { 3.6, // BATT_WARN_THRSHD [V]
                                     3.4, // BATT_STOP_THRSHD [V]
                                     3.2, // BATT_SHUT_THRSHD [V]
                                     0.1  // BATT_HYST        [V]
                                   };
  battery = new Battery(new MyBatteryAdapter(), battCfg);

  //---------------------------------------------------------------------------
  // Status LED (ToggleButton)
  //---------------------------------------------------------------------------
  statusLed = new Indicator("led", "Built in LED.");
  statusLed->assignAdapter(new MyBuiltinLedIndicatorAdapter());

  //-----------------------------------------------------------------------------
  // LoRaWan
  //-----------------------------------------------------------------------------
  loRaWanInterface = new LoraWanAbp(new MyLoRaWanConfigAdapter(assets));
  detectorPersDataMemory->assignLoRaWanDriver(loRaWanInterface);
  loRaWanInterface->setLoraWanRxDataEventAdapter(new LoRaWanRxDataToStatusLedAdapter(statusLed, loRaWanInterface));
  loRaWanInterface->setLoraWanTxDataEventAdapter(new LoRaWanTxDataRequester(loRaWanInterface));

  // trigger the first measurement and prepare data for TX with LoRaWan
  loRaWanInterface->loraWanTxDataEventAdapter()->messageTransmitted();
}

void loop()
{
  // file deepcode ignore CppSameEvalBinaryExpressionfalse: sCmd gets instantiated by setupProdDebugEnv()
  if (0 != sCmd)
  {
    sCmd->readSerial();     	  // process serial commands (Debug CLI)
  }
  scheduleTimers();         	  // process Timers
  loRaWanInterface->loopOnce(); // process LoRaWan driver
}
