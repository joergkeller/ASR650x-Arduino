#ifndef LoRaWan_APP_H
#define LoRaWan_APP_H

#include <stdio.h>
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "hw.h"
#include "Region.h"
#include "low_power.h"
#include "spi-board.h"
#include "rtc-board.h"
#include "asr_timer.h"
#include "sx126x.h"
#include "board-config.h"
#include "hw_conf.h"
#include <uart_port.h>
#include "AT_Command.h"
#include <HardwareSerial.h>

extern uint8_t devEui[];
extern uint8_t appEui[];
extern uint8_t appKey[];
extern uint8_t nwkSKey[];
extern uint8_t appSKey[];
extern uint32_t devAddr;
extern uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];
extern uint8_t appDataSize;
extern uint8_t appPort;
extern uint32_t txDutyCycleTime;
extern bool overTheAirActivation;
extern LoRaMacRegion_t loraWanRegion;
extern bool loraWanAdr;
extern bool isTxConfirmed;
extern uint32_t appTxDutyCycle;
extern DeviceClass_t loraWanClass;
extern bool passthroughMode;
extern uint8_t confirmedNbTrials;
extern bool modeLoraWan;
extern bool keepNet;
extern bool IsLoRaMacNetworkJoined;
extern uint16_t userChannelsMask[6];


/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

extern enum eDeviceState_LoraWan deviceState;

class LoRaWanClass{
public:
  void init(DeviceClass_t lorawanClass,LoRaMacRegion_t region);
  void join();
  void send();
  void cycle(uint32_t dutyCycle);
  void sleep();
  void ifskipjoin();

#if defined(CubeCell_BoardPlus)||defined(CubeCell_GPS)
  void displayJoining();
  void displayJoined();
  void displaySending();
  void displayAck();
  void displayMcuInit();
#endif
};

extern "C" uint16_t getBatteryVoltage(void);
extern "C" bool SendFrame( void );
extern "C" void turnOnRGB(uint32_t color,uint32_t time);
extern "C" void turnOffRGB(void);
extern "C" uint16_t getBatteryVoltage(void);
extern "C" bool checkUserAt(char * cmd, char * content);
extern "C" void downLinkDataHandle(McpsIndication_t *mcpsIndication);
extern "C" void lwan_dev_params_update( void );


extern LoRaWanClass LoRaWAN;

#endif
