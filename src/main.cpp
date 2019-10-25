/*
 * wiring-skeleton.cpp
 *
 *  Created on: 15.03.2017
 *      Author: niklausd
 */

#include <Arduino.h>

// PlatformIO libraries
#include <SerialCommand.h>  // pio lib install 173, lib details see https://github.com/kroimon/Arduino-SerialCommand
#include <Timer.h>          // pio lib install 1699, lib details see https://github.com/dniklaus/wiring-timer
#include <DHT.h>
#include <DHT_U.h>

// private libraries
#include <DbgCliNode.h>
#include <DbgCliTopic.h>
#include <DbgCliCommand.h>
#include <DbgTracePort.h>
#include <DbgTraceContext.h>
#include <DbgTraceOut.h>
#include <DbgPrintConsole.h>
#include <DbgTraceLevel.h>
#include <AppDebug.h>
#include <ProductDebug.h>
#include <RamUtils.h>
#include <Assets.h>
#include <MyDeviceSerialNrAdapter.h>
#include <DetectorFakePersDataMemory.h>
#include <Battery.h>
#include <MyBatteryAdapter.h>
#include <LoraWanAbp.h>
#include <LoRaWanDriver.h>
#include <MyLoRaWanConfigAdapter.h>
#include <ToggleButton.h>
#include <LoRaWanRxDataToStatusLedAdapter.h>
#include <ILoraWanTxDataEventAdapter.h>
#include <ILoraWanRxDataEventAdapter.h>
#include <LoraMessage.h>

LoRaWanDriver* m_LoraWanInterface = 0;
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
ToggleButton* statusLed = 0;

//-----------------------------------------------------------------------------

class LoRaWanTxDataRequester : public ILoraWanTxDataEventAdapter
{
private:
  LoRaWanDriver* m_loraDriver;
  DHT_Unified* m_dht;

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


    // Get temperature event and pass its value to the LoRaWan driver.
    m_dht->temperature().getEvent(&event);
    if (!isnan(event.temperature))
    {
//      int8_t tempInteger  = static_cast<int8_t>(event.temperature);
//      int8_t tempFraction = static_cast<int8_t>(static_cast<int16_t>(event.temperature*100.0)-static_cast<int16_t>(event.temperature)*100);
//
//      uint8_t temperature[] = { static_cast<uint8_t>(tempInteger), static_cast<uint8_t>(tempFraction) };
//      TR_PRINTF(m_loraDriver->trPort(), DbgTrace_Level::debug, "Temperature: %d.%02d *C", tempInteger, tempFraction);

      loRaMessage.addTemperature(event.temperature);
    }

    m_dht->humidity().getEvent(&event);
    if (!isnan(event.relative_humidity))
    {
      loRaMessage.addHumidity(event.relative_humidity);
    }

    TR_PRINTF(m_loraDriver->trPort(), DbgTrace_Level::debug, "Temperature: %f *C, Hunidity: %f", event.temperature, event.relative_humidity);

    m_loraDriver->setPeriodicMessageData(loRaMessage.getBytes(), loRaMessage.getLength());
  }
};

//-----------------------------------------------------------------------------

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 0);

  delay(5000);

  setupProdDebugEnv();

  //-----------------------------------------------------------------------------
  // Assets (inventory and persistent data)
  //-----------------------------------------------------------------------------
  assets = new Assets(new MyDeviceSerialNrAdapter(), new DetectorFakePersDataMemory());

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
  statusLed = new ToggleButton(ToggleButton::BTN_NC, BUILTIN_LED);

  //-----------------------------------------------------------------------------
  // LoRaWan
  //-----------------------------------------------------------------------------
  m_LoraWanInterface = new LoraWanAbp(new MyLoRaWanConfigAdapter(assets));
  m_LoraWanInterface->setLoraWanRxDataEventAdapter(new LoRaWanRxDataToStatusLedAdapter(statusLed, m_LoraWanInterface));
  m_LoraWanInterface->setLoraWanTxDataEventAdapter(new LoRaWanTxDataRequester(m_LoraWanInterface));

  // trigger the first measurement and prepare data for TX with LoRaWan
  m_LoraWanInterface->loraWanTxDataEventAdapter()->messageTransmitted();
}

void loop()
{
  if (0 != sCmd)
  {
    sCmd->readSerial();     // process serial commands
  }
  scheduleTimers();         // process Timers
  m_LoraWanInterface->loopOnce();
}
