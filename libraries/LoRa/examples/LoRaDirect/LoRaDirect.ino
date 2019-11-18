#include "Arduino.h"
#include "LoRaMac.h"
#include "Commissioning.h"

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
#define CONFIRMED_TRIALS 8

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE DR_5

/* the application data transmission duty cycle.  value in [ms]. */
uint32_t APP_TX_DUTYCYCLE = 15000;

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND 1000

/*!
 * Indicates if a new packet can be sent
 */
static bool pendingTx = false;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t nextPacketTimer;

/*!
 * Device states
 */
enum DeviceState_t {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} deviceState;

LoRaMacPrimitives_t mac_primitive;

LoRaMacCallback_t mac_callback;

uint8_t devEui[] = LORAWAN_DEVICE_EUI;
uint8_t appEui[] = LORAWAN_APPLICATION_EUI;
uint8_t appKey[] = LORAWAN_APPLICATION_KEY;

uint8_t nwkSessionKey[] = LORAWAN_NWKSKEY;
uint8_t appSessionKey[] = LORAWAN_APPSKEY;
uint32_t deviceAddress = LORAWAN_DEVICE_ADDRESS;


/*!
 * User application data size
 */
uint8_t messageSize = 4;

/*!
 * User application data
 */
uint8_t message[LORAWAN_APP_DATA_MAX_SIZE];


void setup() {
    BoardInitMcu();
    Serial.begin(115200);
    deviceState = DEVICE_STATE_INIT;
//    LoRaWAN.Ifskipjoin();
}

void loop() {
  switch( deviceState ) {
    case DEVICE_STATE_INIT: {
//      printDevParam();
      Serial.printf("LoRaWan Class%X start! \r\n",LORAWAN_CLASS+10);
      LoRaWAN_Init(LORAWAN_CLASS, ACTIVE_REGION);
      break;
    }
    case DEVICE_STATE_JOIN: {
      LoRaWAN_Join_OTAA();
      break;
    }
    case DEVICE_STATE_SEND: {
      PrepareTxFrame();
      LoRaWAN_Send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE: {
      // Schedule next packet transmission
      uint32_t delay = APP_TX_DUTYCYCLE + randr( 0, APP_TX_DUTYCYCLE_RND );
      TimerSetValue( &nextPacketTimer, delay );
      TimerStart( &nextPacketTimer );
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP: {
      LowPower_Handler();
      // Process Radio IRQ
      Radio.IrqProcess();
      break;
    }
    default: {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication ) {
  switch( mlmeIndication->MlmeIndication ) {
    case MLME_SCHEDULE_UPLINK: { // The MAC signals that we shall provide an uplink as soon as possible
      OnTxNextPacketTimerEvent( );
      break;
    }
    default: {
      break;
    }
  }
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication ) {
  if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK ) {
    return;
  }
  printf( "receive data: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,(int)mcpsIndication->RxDatarate);

#if (LoraWan_RGB==1)
  RGB_ON(COLOR_RECEIVED, 200);
  RGB_OFF();
#endif

  switch( mcpsIndication->McpsIndication ) {
    case MCPS_UNCONFIRMED: break;
    case MCPS_CONFIRMED: break;
    case MCPS_PROPRIETARY: break;
    case MCPS_MULTICAST: break;
    default: break;
  }

  // Check Multicast
  // Check Port
  // Check Datarate
  // Check FramePending
  if( mcpsIndication->FramePending ) {
    // The server signals that it has pending data to be sent.
    // We schedule an uplink as soon as possible to flush the server.
    OnTxNextPacketTimerEvent();
  }
  // Check Buffer
  // Check BufferSize
  // Check Rssi
  // Check Snr
  // Check RxSlot
  if( mcpsIndication->RxData )
  {
    //memset(temp,0,200);
    //memset(temp1,0,200);

    //HexToString((const char *)(mcpsIndication->Buffer),mcpsIndication->BufferSize,(char *)(temp1));
    //temp1[mcpsIndication->BufferSize * 2]='\0';

    printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
    printf("+REV DATA:");
    for(uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
      printf("%02X",mcpsIndication->Buffer[i]);
    }
    printf("\r\n");
  }
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent() {
  MibRequestConfirm_t mibReq;
  LoRaMacStatus_t status;

  TimerStop( &nextPacketTimer );

  mibReq.Type = MIB_NETWORK_JOINED;
  status = LoRaMacMibGetRequestConfirm( &mibReq );

  if( status == LORAMAC_STATUS_OK ) {
    if( mibReq.Param.IsNetworkJoined ) {
      deviceState = DEVICE_STATE_SEND;
      pendingTx = false;
    } else {
      // Network not joined yet. Try to join again
      MlmeReq_t mlmeReq;
      mlmeReq.Type = MLME_JOIN;
      mlmeReq.Req.Join.DevEui = devEui;
      mlmeReq.Req.Join.AppEui = appEui;
      mlmeReq.Req.Join.AppKey = appKey;

      if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK ) {
        deviceState = DEVICE_STATE_SLEEP;
      } else {
        deviceState = DEVICE_STATE_CYCLE;
      }
    }
  }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm ) {
  switch( mlmeConfirm->MlmeRequest ) {
    case MLME_JOIN: {
      if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
        printf("joined\r\n");
#if (LoraWan_RGB==1)
        RGB_ON(COLOR_JOINED,500);
        RGB_OFF();
#endif
      } else {
        uint32_t rejoin_delay = 30000;
        printf("join failed\r\n");
        TimerSetValue( &nextPacketTimer, rejoin_delay );
        TimerStart( &nextPacketTimer );
      }
      break;
    }
    case MLME_LINK_CHECK: {
      if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
        // Check DemodMargin
        // Check NbGateways
      }
      break;
    }
    default: {
        break;
    }
  }
  pendingTx = false;
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm ) {
  if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
    switch( mcpsConfirm->McpsRequest ) {
      case MCPS_UNCONFIRMED: {
        // Check Datarate
        // Check TxPower
        break;
      }
      case MCPS_CONFIRMED: {
        // Check Datarate
        // Check TxPower
        // Check AckReceived
        // Check NbTrials
        break;
      }
      case MCPS_PROPRIETARY: {
        break;
      }
      default:
        break;
    }
  }
  pendingTx = false;
}

void LoRaWAN_Init(DeviceClass_t CLASS,LoRaMacRegion_t REGION) {
  MibRequestConfirm_t mibReq;

  mac_primitive.MacMcpsConfirm = McpsConfirm;
  mac_primitive.MacMcpsIndication = McpsIndication;
  mac_primitive.MacMlmeConfirm = MlmeConfirm;
  mac_primitive.MacMlmeIndication = MlmeIndication;
  mac_callback.GetBatteryLevel = BoardGetBatteryLevel;
  mac_callback.GetTemperatureLevel = NULL;
  LoRaMacInitialization( &mac_primitive, &mac_callback, REGION);
  TimerStop( &nextPacketTimer );
  TimerInit( &nextPacketTimer, OnTxNextPacketTimerEvent );

  mibReq.Type = MIB_ADR;
  mibReq.Param.AdrEnable = LORAWAN_ADR;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
  LoRaMacMibSetRequestConfirm( &mibReq );

  lwan_dev_params_update();

  deviceState = DEVICE_STATE_JOIN;
}

void LoRaWAN_Join_OTAA() {
  Serial.println("joining...");
  MlmeReq_t mlmeReq;

  mlmeReq.Type = MLME_JOIN;

  mlmeReq.Req.Join.DevEui = devEui;
  mlmeReq.Req.Join.AppEui = appEui;
  mlmeReq.Req.Join.AppKey = appKey;
  if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK ) {
    deviceState = DEVICE_STATE_SLEEP;
  } else {
    deviceState = DEVICE_STATE_CYCLE;
  }
}

void LoRaWAN_Join_ABP() {
  MibRequestConfirm_t mibReq;

  mibReq.Type = MIB_NET_ID;
  mibReq.Param.NetID = LORAWAN_NETWORK_ID;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_DEV_ADDR;
  mibReq.Param.DevAddr = deviceAddress;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_NWK_SKEY;
  mibReq.Param.NwkSKey = nwkSessionKey;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_APP_SKEY;
  mibReq.Param.AppSKey = appSessionKey;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_NETWORK_JOINED;
  mibReq.Param.IsNetworkJoined = true;
  LoRaMacMibSetRequestConfirm( &mibReq );

  deviceState = DEVICE_STATE_SEND;
}

/* Prepares the payload of the frame */
static void PrepareTxFrame() {
    messageSize = 4;//messageSize max value is 64
    message[0] = 0x00;
    message[1] = 0x01;
    message[2] = 0x02;
    message[3] = 0x03;
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
LoRaMacStatus_t SendFrame(bool confirmReception, uint8_t applicationPort) {
  McpsReq_t mcpsReq;
  LoRaMacTxInfo_t txInfo;

  if( LoRaMacQueryTxPossible( messageSize, &txInfo ) != LORAMAC_STATUS_OK ) {
    // Send empty frame in order to flush MAC commands
    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fBuffer = NULL;
    mcpsReq.Req.Unconfirmed.fBufferSize = 0;
    mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
  } else {
    if( confirmReception ) {
      printf("confirmed uplink sending ...\r\n");
      mcpsReq.Type = MCPS_CONFIRMED;
      mcpsReq.Req.Confirmed.fPort = applicationPort;
      mcpsReq.Req.Confirmed.fBuffer = message;
      mcpsReq.Req.Confirmed.fBufferSize = messageSize;
      mcpsReq.Req.Confirmed.NbTrials = CONFIRMED_TRIALS;
      mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    } else {
      printf("unconfirmed uplink sending ...\r\n");
      mcpsReq.Type = MCPS_UNCONFIRMED;
      mcpsReq.Req.Unconfirmed.fPort = applicationPort;
      mcpsReq.Req.Unconfirmed.fBuffer = message;
      mcpsReq.Req.Unconfirmed.fBufferSize = messageSize;
      mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
  }
  return LoRaMacMcpsRequest( &mcpsReq );
}

void LoRaWAN_Send() {
  if( pendingTx ) return;

  MibRequestConfirm_t mibReq;
  mibReq.Type = MIB_DEVICE_CLASS;
  LoRaMacMibGetRequestConfirm( &mibReq );

  if( mibReq.Param.Class != CLASS_A ) {
    mibReq.Param.Class = CLASS_A;
    LoRaMacMibSetRequestConfirm( &mibReq );
  }

  if( SendFrame(true, 2) == LORAMAC_STATUS_OK ) {
    pendingTx = true;
  }
}

static void lwan_dev_params_update() {
  MibRequestConfirm_t mibReq;
  uint16_t channelsMaskTemp[6];
  channelsMaskTemp[0] = 0x00FF;
  channelsMaskTemp[1] = 0x0000;
  channelsMaskTemp[2] = 0x0000;
  channelsMaskTemp[3] = 0x0000;
  channelsMaskTemp[4] = 0x0000;
  channelsMaskTemp[5] = 0x0000;

  mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
  mibReq.Param.ChannelsMask = channelsMaskTemp;
  LoRaMacMibSetRequestConfirm(&mibReq);

  mibReq.Type = MIB_CHANNELS_MASK;
  mibReq.Param.ChannelsMask = channelsMaskTemp;
  LoRaMacMibSetRequestConfirm(&mibReq);
}

