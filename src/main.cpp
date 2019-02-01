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
#include <LoraWanAbp.hpp>
#include <LoRaWanDriver.hpp>
#include <MyLoRaWanConfigAdapter.h>
#include <LoraWanPriorityQueue.hpp>
#include <pb_encode.h>
#include <pb_decode.h>
#include <SerialCommand.h>

/* This is the buffer where we will store our message. */
bool setMessageOnce = true;
LoRaWanDriver* m_LoraWanInterface;
LoraWanPriorityQueue* m_LoraWanPriorityQueue;

#ifndef BUILTIN_LED
#define BUILTIN_LED 13
#endif

SerialCommand* sCmd = 0;
Assets* assets = 0;
Battery* battery = 0;

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 0);

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

  //-----------------------------------------------------------------------------
  // Sensors
  //-----------------------------------------------------------------------------
  pmProcess = new PM_Process(&Serial1, new MyPM_ProcessAdapter());
  pmProcess->init(9600);
  dhtProcess = new DHT_Process(new MyDHT_ProcessAdapter());

  //-----------------------------------------------------------------------------
  // LoRaWan
  //-----------------------------------------------------------------------------
  m_LoraWanInterface = new LoraWanAbp(new MyLoRaWanConfigAdapter(assets));
  m_LoraWanPriorityQueue = new LoraWanPriorityQueue(m_LoraWanInterface);
  // m_MeasurementFacade = new MeasurementFacade(m_LoraWanPriorityQueue);
  // m_SystemStatusFacade = new SystemStatusFacade(m_LoraWanPriorityQueue);
  m_LoraWanPriorityQueue->setUpdateCycleHighPriorityPerdioc(2);
  m_LoraWanPriorityQueue->start();
}

void loop()
{
  if (0 != sCmd)
  {
    sCmd->readSerial();     // process serial commands
  }
  yield();                  // process Timers
  m_LoraWanInterface->loopOnce();
  m_LoraWanPriorityQueue->update();
}
