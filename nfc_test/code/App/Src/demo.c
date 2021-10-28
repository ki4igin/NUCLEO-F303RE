/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file
 *
 *  \author 
 *
 *  \brief Demo application
 *
 *  This demo shows how to poll for several types of NFC cards/devices and how 
 *  to exchange data with these devices, using the RFAL library.
 *
 *  This demo does not fully implement the activities according to the standards,
 *  it performs the required to communicate with a card/device and retrieve 
 *  its UID. Also blocking methods are used for data exchange which may lead to
 *  long periods of blocking CPU/MCU.
 *  For standard compliant example please refer to the Examples provided
 *  with the RFAL library.
 * 
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "demo.h"
#include "utils.h"
#include "rfal_nfc.h"

#if defined(ST25R3916) && defined(RFAL_FEATURE_LISTEN_MODE)
  #include "demo_ce.h"
#endif

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* Definition of possible states the demo state machine could have */
#define DEMO_ST_NOTINIT         0 /*!< Demo State:  Not initialized        */
#define DEMO_ST_START_DISCOVERY 1 /*!< Demo State:  Start Discovery        */
#define DEMO_ST_DISCOVERY       2 /*!< Demo State:  Discovery              */

#define DEMO_NFCV_BLOCK_LEN 4 /*!< NFCV Block len                      */

#define DEMO_NFCV_USE_SELECT_MODE false /*!< NFCV demonstrate select mode        */
#define DEMO_NFCV_WRITE_TAG       true  /*!< NFCV demonstrate Write Single Block */

/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[]     = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

static struct
{
  uint8_t *pdata;
  uint32_t size;
} wrData;


static struct
{
  uint32_t *pdata;
  uint8_t   startBlock;
  uint32_t  size;
} rdData;

typedef __PACKED_STRUCT
{
  uint8_t  flags;
  uint32_t blockData;
  uint16_t crc;
}
Rxdata_t;

static rfalNfcDiscoverParam discParam;
static uint8_t              state = DEMO_ST_NOTINIT;

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

static void demoNotif(rfalNfcState st);
ReturnCode  demoTransceiveBlocking(uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxBuf, uint16_t **rcvLen, uint32_t fwt);

/*!
 *****************************************************************************
 * \brief Demo Notification
 *
 *  This function receives the event notifications from RFAL
 *****************************************************************************
 */
void demowrData(void *pdata, uint32_t size)
{
  wrData.pdata = (uint8_t *)pdata;
  wrData.size  = size;
}

void demordData(void *pdata, uint8_t startBlock, uint32_t size)
{
  rdData.pdata      = (uint32_t *)pdata;
  rdData.startBlock = startBlock;
  rdData.size       = size;
}

uint8_t demoGetStatus(void)
{
  return state;
}

static void demoNotif(rfalNfcState st)
{
  uint8_t        devCnt;
  rfalNfcDevice *dev;

  if (st == RFAL_NFC_STATE_WAKEUP_MODE)
  {
    platformLog("Wake Up mode started \r\n");
  }
  else if (st == RFAL_NFC_STATE_POLL_TECHDETECT)
  {
    platformLog("Wake Up mode terminated. Polling for devices \r\n");
  }
  else if (st == RFAL_NFC_STATE_POLL_SELECT)
  {
    /* Multiple devices were found, activate first of them */
    rfalNfcGetDevicesFound(&dev, &devCnt);
    rfalNfcSelect(0);

    platformLog("Multiple Tags detected: %d \r\n", devCnt);
  }
}

/*!
 *****************************************************************************
 * \brief Demo Ini
 *
 *  This function Initializes the required layers for the demo
 *
 * \return true  : Initialization ok
 * \return false : Initialization failed
 *****************************************************************************
 */
bool demoIni(void)
{
  ReturnCode err;

  err = rfalNfcInitialize();
  if (err == ERR_NONE)
  {
    discParam.compMode = RFAL_COMPLIANCE_MODE_NFC;
    discParam.devLimit = 1U;
    discParam.nfcfBR   = RFAL_BR_212;
    discParam.ap2pBR   = RFAL_BR_424;

    ST_MEMCPY(&discParam.nfcid3, NFCID3, sizeof(NFCID3));
    ST_MEMCPY(&discParam.GB, GB, sizeof(GB));
    discParam.GBLen = sizeof(GB);

    discParam.notifyCb            = demoNotif;
    discParam.totalDuration       = 1000U;
    discParam.wakeupEnabled       = false;
    discParam.wakeupConfigDefault = true;
    discParam.techs2Find          = (RFAL_NFC_POLL_TECH_A | RFAL_NFC_POLL_TECH_B | RFAL_NFC_POLL_TECH_F | RFAL_NFC_POLL_TECH_V | RFAL_NFC_POLL_TECH_ST25TB);

#if defined(ST25R3911) || defined(ST25R3916)
    discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
#endif /* ST25R95 */

#if defined(ST25R3916)

    /* Set configuration for NFC-A CE */
    ST_MEMCPY(discParam.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN); /* Set SENS_RES / ATQA */
    ST_MEMCPY(discParam.lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_NFCID2_LEN);            /* Set NFCID / UID */
    discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_07;                            /* Set NFCID length to 7 bytes */
    discParam.lmConfigPA.SEL_RES  = ceNFCA_SEL_RES;                                  /* Set SEL_RES / SAK */

    /* Set configuration for NFC-F CE */
    ST_MEMCPY(discParam.lmConfigPF.SC, ceNFCF_SC, RFAL_LM_SENSF_SC_LEN);                      /* Set System Code */
    ST_MEMCPY(&ceNFCF_SENSF_RES[RFAL_NFCF_LENGTH_LEN], ceNFCF_nfcid2, RFAL_LM_SENSF_RES_LEN); /* Load NFCID2 on SENSF_RES */
    ST_MEMCPY(discParam.lmConfigPF.SENSF_RES, ceNFCF_SENSF_RES, RFAL_LM_SENSF_RES_LEN);       /* Set SENSF_RES / Poll Response */

    discParam.techs2Find |= (RFAL_NFC_LISTEN_TECH_A | RFAL_NFC_LISTEN_TECH_F);

#endif /* ST25R95 */

    state = DEMO_ST_START_DISCOVERY;
    return true;
  }
  return false;
}

/*!
 *****************************************************************************
 * \brief Demo Cycle
 *
 *  This function executes the demo state machine. 
 *  It must be called periodically
 *****************************************************************************
 */

void demoStop(void)
{
  rfalNfcDeactivate(false);
}

ErrorStatus demoCheckCycle(void)
{
  static rfalNfcDevice *nfcDevice;

  ErrorStatus err = ERROR;

  rfalNfcWorker();

  switch (state)
  {
    /*******************************************************************************/
    case DEMO_ST_START_DISCOVERY:
      rfalNfcDeactivate(false);
      rfalNfcDiscover(&discParam);
      platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      state = DEMO_ST_DISCOVERY;
      break;

    /*******************************************************************************/
    case DEMO_ST_DISCOVERY:
    {
      uint8_t status = rfalNfcIsDevActivated(rfalNfcGetState());
      // platformLog("status %s\r\n", hex2Str(&status, 1));

      if (status)
      {
        rfalNfcGetActiveDevice(&nfcDevice);

        if (nfcDevice->type == RFAL_NFC_LISTEN_TYPE_NFCV)
        {
          uint8_t devUID[RFAL_NFCV_UID_LEN];

          ST_MEMCPY(devUID, nfcDevice->nfcid, nfcDevice->nfcidLen); /* Copy the UID into local var */
          REVERSE_BYTES(devUID, RFAL_NFCV_UID_LEN);                 /* Reverse the UID for display purposes */
          platformLog("ISO15693/NFC-V card found. UID: %s\r\n", hex2Str(devUID, RFAL_NFCV_UID_LEN));

          platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
          err = SUCCESS;
        }

        rfalNfcDeactivate(false);
        platformDelay(100);
        state = DEMO_ST_START_DISCOVERY;
      }
    }
    break;

    /*******************************************************************************/
    case DEMO_ST_NOTINIT:
    default:
      break;
  }
  return err;
}

ErrorStatus demoWriteCycle(void)
{
  static rfalNfcDevice *nfcDevice;

  ErrorStatus err = ERROR;

  rfalNfcWorker();

  switch (state)
  {
    /*******************************************************************************/
    case DEMO_ST_START_DISCOVERY:
      rfalNfcDeactivate(false);
      rfalNfcDiscover(&discParam);
      platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      state = DEMO_ST_DISCOVERY;
      break;

    /*******************************************************************************/
    case DEMO_ST_DISCOVERY:
    {
      uint8_t status = rfalNfcIsDevActivated(rfalNfcGetState());
      // platformLog("status %s\r\n", hex2Str(&status, 1));

      if (status)
      {
        rfalNfcGetActiveDevice(&nfcDevice);

        if (nfcDevice->type == RFAL_NFC_LISTEN_TYPE_NFCV)
        {
          uint8_t devUID[RFAL_NFCV_UID_LEN];

          ST_MEMCPY(devUID, nfcDevice->nfcid, nfcDevice->nfcidLen); /* Copy the UID into local var */
          REVERSE_BYTES(devUID, RFAL_NFCV_UID_LEN);                 /* Reverse the UID for display purposes */
          platformLog("ISO15693/NFC-V card found. UID: %s\r\n", hex2Str(devUID, RFAL_NFCV_UID_LEN));

          platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);

          err = SUCCESS;

          uint8_t *uid = nfcDevice->dev.nfcv.InvRes.UID;

          if (wrData.size == 0)
          {
            return err;
          }

          for (uint32_t i = 0; i < wrData.size / 4; i++)
          {
            ReturnCode rc = rfalNfcvPollerWriteSingleBlock(RFAL_NFCV_REQ_FLAG_DEFAULT, uid, i, &wrData.pdata[4 * i], DEMO_NFCV_BLOCK_LEN);
            platformLog(" Write Block: %s Data: %s\r\n", (rc != ERR_NONE) ? "FAIL" : "OK", hex2Str(&wrData.pdata[4 * i], DEMO_NFCV_BLOCK_LEN));
          }
        }
        rfalNfcDeactivate(false);
        platformDelay(200);
        state = DEMO_ST_START_DISCOVERY;
      }
    }
    break;

    /*******************************************************************************/
    case DEMO_ST_NOTINIT:
    default:
      break;
  }
  return err;
}

ErrorStatus demoReadCycle(void)
{
  static rfalNfcDevice *nfcDevice;

  ErrorStatus err = ERROR;

  rfalNfcWorker();

  switch (state)
  {
    /*******************************************************************************/
    case DEMO_ST_START_DISCOVERY:
      rfalNfcDeactivate(false);
      rfalNfcDiscover(&discParam);
      platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      state = DEMO_ST_DISCOVERY;
      break;

    /*******************************************************************************/
    case DEMO_ST_DISCOVERY:
    {
      uint8_t status = rfalNfcIsDevActivated(rfalNfcGetState());
      // platformLog("status %s\r\n", hex2Str(&status, 1));

      if (status)
      {
        rfalNfcGetActiveDevice(&nfcDevice);

        if (nfcDevice->type == RFAL_NFC_LISTEN_TYPE_NFCV)
        {
          uint8_t devUID[RFAL_NFCV_UID_LEN];

          ST_MEMCPY(devUID, nfcDevice->nfcid, nfcDevice->nfcidLen); /* Copy the UID into local var */
          REVERSE_BYTES(devUID, RFAL_NFCV_UID_LEN);                 /* Reverse the UID for display purposes */
          platformLog("ISO15693/NFC-V card found. UID: %s\r\n", hex2Str(devUID, RFAL_NFCV_UID_LEN));

          platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
          err = SUCCESS;

          uint16_t rcvLen;
          uint8_t *uid = nfcDevice->dev.nfcv.InvRes.UID;
          Rxdata_t rxdata;

          for (uint32_t i = 0; i < rdData.size / 4; i++)
          {
            ReturnCode rc = rfalNfcvPollerReadSingleBlock(RFAL_NFCV_REQ_FLAG_DEFAULT, uid, i + rdData.startBlock, (uint8_t *)&rxdata, sizeof(rxdata), &rcvLen);
            platformLog(" Read Block: %s %s\r\n",
                        (rc != ERR_NONE) ? "FAIL" : "OK Data:",
                        (rc != ERR_NONE) ? "" : hex2Str((uint8_t *)&rxdata.blockData, sizeof(rxdata.blockData)));
            rdData.pdata[i] = rxdata.blockData;
          }
        }

        rfalNfcDeactivate(false);
        platformDelay(100);
        state = DEMO_ST_START_DISCOVERY;
      }
    }
    break;

    /*******************************************************************************/
    case DEMO_ST_NOTINIT:
    default:
      break;
  }
  return err;
}

/*!
 *****************************************************************************
 * \brief Demo Blocking Transceive 
 *
 * Helper function to send data in a blocking manner via the rfalNfc module 
 *  
 * \warning A protocol transceive handles long timeouts (several seconds), 
 * transmission errors and retransmissions which may lead to a long period of 
 * time where the MCU/CPU is blocked in this method.
 * This is a demo implementation, for a non-blocking usage example please 
 * refer to the Examples available with RFAL
 *
 * \param[in]  txBuf      : data to be transmitted
 * \param[in]  txBufSize  : size of the data to be transmited
 * \param[out] rxData     : location where the received data has been placed
 * \param[out] rcvLen     : number of data bytes received
 * \param[in]  fwt        : FWT to be used (only for RF frame interface, 
 *                                          otherwise use RFAL_FWT_NONE)
 *
 * 
 *  \return ERR_PARAM     : Invalid parameters
 *  \return ERR_TIMEOUT   : Timeout error
 *  \return ERR_FRAMING   : Framing error detected
 *  \return ERR_PROTO     : Protocol error detected
 *  \return ERR_NONE      : No error, activation successful
 * 
 *****************************************************************************
 */
ReturnCode demoTransceiveBlocking(uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt)
{
  ReturnCode err;

  err = rfalNfcDataExchangeStart(txBuf, txBufSize, rxData, rcvLen, fwt);
  if (err == ERR_NONE)
  {
    do
    {
      rfalNfcWorker();
      err = rfalNfcDataExchangeGetStatus();
    } while (err == ERR_BUSY);
  }
  return err;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
