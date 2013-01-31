/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Airgo Networks, Inc proprietary. All rights reserved.
 * This file sirApi.h contains definitions exported by
 * Sirius software.
 * Author:        Chandra Modumudi
 * Date:          04/16/2002
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 */

#ifndef __SIR_API_H
#define __SIR_API_H

#include "sirTypes.h"
#include "sirMacProtDef.h"
#include "aniSystemDefs.h"
#include "sirParams.h"

#ifdef FEATURE_WLAN_CCX
#include "ccxGlobal.h"
#endif

/// Maximum number of STAs allowed in the BSS
#define SIR_MAX_NUM_STA                256

/// Maximum number of Neighbors reported by STA for LB feature
#define SIR_MAX_NUM_NEIGHBOR_BSS       3

/// Maximum number of Neighbors reported by STA for LB feature
#define SIR_MAX_NUM_ALTERNATE_RADIOS   5

/// Maximum size of SCAN_RSP message
#define SIR_MAX_SCAN_RSP_MSG_LENGTH    2600

/// Start of Sirius software/Host driver message types
#define SIR_HAL_HOST_MSG_START         0x1000

/// Power save level definitions
#define SIR_MAX_POWER_SAVE          3
#define SIR_INTERMEDIATE_POWER_SAVE 4
#define SIR_NO_POWER_SAVE           5

/// Max supported channel list
#define SIR_MAX_SUPPORTED_CHANNEL_LIST      96

/// Maximum DTIM Factor
#define SIR_MAX_DTIM_FACTOR         32

#define SIR_MDIE_SIZE               3



#define SIR_NUM_11B_RATES 4   //1,2,5.5,11
#define SIR_NUM_11A_RATES 8  //6,9,12,18,24,36,48,54
#define SIR_NUM_POLARIS_RATES 3 //72,96,108
#define SIR_NUM_TITAN_RATES 26
#define SIR_NUM_TAURUS_RATES 4 //136.5, 151.7,283.5,315
#define SIR_NUM_PROP_RATES  (SIR_NUM_TITAN_RATES + SIR_NUM_TAURUS_RATES)

#define SIR_11N_PROP_RATE_136_5 (1<<28)
#define SIR_11N_PROP_RATE_151_7 (1<<29)
#define SIR_11N_PROP_RATE_283_5 (1<<30)
#define SIR_11N_PROP_RATE_315     (1<<31)
#define SIR_11N_PROP_RATE_BITMAP 0x80000000 //only 315MBPS rate is supported today
//Taurus is going to support 26 Titan Rates(no ESF/concat Rates will be supported)
//First 26 bits are reserved for Titan and last 4 bits for Taurus, 2(27 and 28) bits are reserved.
//#define SIR_TITAN_PROP_RATE_BITMAP 0x03FFFFFF
//Disable all Titan rates
#define SIR_TITAN_PROP_RATE_BITMAP 0
#define SIR_CONVERT_2_U32_BITMAP(nRates) ((nRates + 31)/32)

/* #tANI_U32's needed for a bitmap representation for all prop rates */
#define SIR_NUM_U32_MAP_RATES    SIR_CONVERT_2_U32_BITMAP(SIR_NUM_PROP_RATES)


#define SIR_PM_SLEEP_MODE   0
#define SIR_PM_ACTIVE_MODE        1

// Used by various modules to load ALL CFG's
#define ANI_IGNORE_CFG_ID 0xFFFF

//hidden SSID options
#define SIR_SCAN_NO_HIDDEN_SSID                      0
#define SIR_SCAN_HIDDEN_SSID_PE_DECISION             1
#define SIR_SCAN_HIDDEN_SSID                         2

#define SIR_MAC_ADDR_LEN        6
#define SIR_IPV4_ADDR_LEN       4

typedef tANI_U8 tSirIpv4Addr[SIR_IPV4_ADDR_LEN];

#define SIR_VERSION_STRING_LEN 64
typedef tANI_U8 tSirVersionString[SIR_VERSION_STRING_LEN];

enum eSirHostMsgTypes
{
    SIR_HAL_APP_SETUP_NTF = SIR_HAL_HOST_MSG_START,
    SIR_HAL_INITIAL_CAL_FAILED_NTF,
    SIR_HAL_NIC_OPER_NTF,
    SIR_HAL_INIT_START_REQ,
    SIR_HAL_SHUTDOWN_REQ,
    SIR_HAL_SHUTDOWN_CNF,
    SIR_HAL_RESET_REQ,
    SIR_HAL_RADIO_ON_OFF_IND,    
    SIR_HAL_RESET_CNF,
    SIR_WRITE_TO_TD,
    SIR_HAL_HDD_ADDBA_REQ, // MAC -> HDD
    SIR_HAL_HDD_ADDBA_RSP, // HDD -> HAL        
    SIR_HAL_DELETEBA_IND, // MAC -> HDD
    SIR_HAL_BA_FAIL_IND, // HDD -> MAC
    SIR_TL_HAL_FLUSH_AC_REQ, 
    SIR_HAL_TL_FLUSH_AC_RSP
};



/**
 * Module ID definitions.
 */
enum {
    SIR_BOOT_MODULE_ID = 1,
    SIR_HAL_MODULE_ID  = 0x10,
    SIR_CFG_MODULE_ID,
    SIR_LIM_MODULE_ID,
    SIR_ARQ_MODULE_ID,
    SIR_SCH_MODULE_ID,
    SIR_PMM_MODULE_ID,
    SIR_MNT_MODULE_ID,
    SIR_DBG_MODULE_ID,
    SIR_DPH_MODULE_ID,
    SIR_SYS_MODULE_ID,
    SIR_SMS_MODULE_ID,

    SIR_PHY_MODULE_ID = 0x20,


    // Add any modules above this line
    SIR_DVT_MODULE_ID
};

#define SIR_WDA_MODULE_ID SIR_HAL_MODULE_ID

/**
 * First and last module definition for logging utility
 *
 * NOTE:  The following definitions need to be updated if
 *        the above list is changed.
 */
#define SIR_FIRST_MODULE_ID     SIR_HAL_MODULE_ID
#define SIR_LAST_MODULE_ID      SIR_DVT_MODULE_ID


// Type declarations used by Firmware and Host software

// Scan type enum used in scan request
typedef enum eSirScanType
{
    eSIR_PASSIVE_SCAN,
    eSIR_ACTIVE_SCAN,
} tSirScanType;

/// Result codes Firmware return to Host SW
typedef enum eSirResultCodes
{
    eSIR_SME_SUCCESS,

    eSIR_EOF_SOF_EXCEPTION,
    eSIR_BMU_EXCEPTION,
    eSIR_LOW_PDU_EXCEPTION,
    eSIR_USER_TRIG_RESET,
    eSIR_LOGP_EXCEPTION,
    eSIR_CP_EXCEPTION,
    eSIR_STOP_BSS,
    eSIR_AHB_HANG_EXCEPTION,
    eSIR_DPU_EXCEPTION,
    eSIR_RPE_EXCEPTION,
    eSIR_TPE_EXCEPTION,
    eSIR_DXE_EXCEPTION,
    eSIR_RXP_EXCEPTION,
    eSIR_MCPU_EXCEPTION,
    eSIR_MCU_EXCEPTION,
    eSIR_MTU_EXCEPTION,
    eSIR_MIF_EXCEPTION,
    eSIR_FW_EXCEPTION,
    eSIR_PS_MUTEX_READ_EXCEPTION,
    eSIR_PHY_HANG_EXCEPTION,
    eSIR_MAILBOX_SANITY_CHK_FAILED,
    eSIR_RADIO_HW_SWITCH_STATUS_IS_OFF, // Only where this switch is present
    eSIR_CFB_FLAG_STUCK_EXCEPTION,

    eSIR_SME_BASIC_RATES_NOT_SUPPORTED_STATUS=30,

    eSIR_SME_INVALID_PARAMETERS=500,
    eSIR_SME_UNEXPECTED_REQ_RESULT_CODE,
    eSIR_SME_RESOURCES_UNAVAILABLE,
    eSIR_SME_SCAN_FAILED,   // Unable to find a BssDescription
                            // matching requested scan criteria
    eSIR_SME_BSS_ALREADY_STARTED_OR_JOINED,
    eSIR_SME_LOST_LINK_WITH_PEER_RESULT_CODE,
    eSIR_SME_REFUSED,
    eSIR_SME_JOIN_TIMEOUT_RESULT_CODE,
    eSIR_SME_AUTH_TIMEOUT_RESULT_CODE,
    eSIR_SME_ASSOC_TIMEOUT_RESULT_CODE,
    eSIR_SME_REASSOC_TIMEOUT_RESULT_CODE,
    eSIR_SME_MAX_NUM_OF_PRE_AUTH_REACHED,
    eSIR_SME_AUTH_REFUSED,
    eSIR_SME_INVALID_WEP_DEFAULT_KEY,
    eSIR_SME_NO_KEY_MAPPING_KEY_FOR_PEER,
    eSIR_SME_ASSOC_REFUSED,
    eSIR_SME_REASSOC_REFUSED,
    eSIR_SME_DEAUTH_WHILE_JOIN, //Received Deauth while joining or pre-auhtentication.
    eSIR_SME_DISASSOC_WHILE_JOIN, //Received Disassociation while joining.
    eSIR_SME_DEAUTH_WHILE_REASSOC, //Received Deauth while ReAssociate.
    eSIR_SME_DISASSOC_WHILE_REASSOC, //Received Disassociation while ReAssociate
    eSIR_SME_STA_NOT_AUTHENTICATED,
    eSIR_SME_STA_NOT_ASSOCIATED,
    eSIR_SME_STA_DISASSOCIATED,
    eSIR_SME_ALREADY_JOINED_A_BSS,
    eSIR_ULA_COMPLETED,
    eSIR_ULA_FAILURE,
    eSIR_SME_LINK_ESTABLISHED,
    eSIR_SME_UNABLE_TO_PERFORM_MEASUREMENTS,
    eSIR_SME_UNABLE_TO_PERFORM_DFS,
    eSIR_SME_DFS_FAILED,
    eSIR_SME_TRANSFER_STA, // To be used when STA need to be LB'ed
    eSIR_SME_INVALID_LINK_TEST_PARAMETERS,// Given in LINK_TEST_START_RSP
    eSIR_SME_LINK_TEST_MAX_EXCEEDED,    // Given in LINK_TEST_START_RSP
    eSIR_SME_UNSUPPORTED_RATE,          // Given in LINK_TEST_RSP if peer does
                                        // support requested rate in
                                        // LINK_TEST_REQ
    eSIR_SME_LINK_TEST_TIMEOUT,         // Given in LINK_TEST_IND if peer does
                                        // not respond before next test packet
                                        // is sent
    eSIR_SME_LINK_TEST_COMPLETE,        // Given in LINK_TEST_IND at the end
                                        // of link test
    eSIR_SME_LINK_TEST_INVALID_STATE,   // Given in LINK_TEST_START_RSP
    eSIR_SME_LINK_TEST_TERMINATE,       // Given in LINK_TEST_START_RSP
    eSIR_SME_LINK_TEST_INVALID_ADDRESS, // Given in LINK_TEST_STOP_RSP
    eSIR_SME_POLARIS_RESET,             // Given in SME_STOP_BSS_REQ
    eSIR_SME_SETCONTEXT_FAILED,         // Given in SME_SETCONTEXT_REQ when
                                        // unable to plumb down keys
    eSIR_SME_BSS_RESTART,               // Given in SME_STOP_BSS_REQ

    eSIR_SME_MORE_SCAN_RESULTS_FOLLOW,  // Given in SME_SCAN_RSP message
                                        // that more SME_SCAN_RSP
                                        // messages are following.
                                        // SME_SCAN_RSP message with
                                        // eSIR_SME_SUCCESS status
                                        // code is the last one.
    eSIR_SME_INVALID_ASSOC_RSP_RXED,    // Sent in SME_JOIN/REASSOC_RSP
                                        // messages upon receiving
                                        // invalid Re/Assoc Rsp frame.
    eSIR_SME_MIC_COUNTER_MEASURES,      // STOP BSS triggered by MIC failures: MAC software to disassoc all stations
                                        // with MIC_FAILURE reason code and perform the stop bss operation
    eSIR_SME_ADDTS_RSP_TIMEOUT,         // didn't get response from peer within
                                        // timeout interval
    eSIR_SME_ADDTS_RSP_FAILED,          // didn't get success response from HAL
    eSIR_SME_RECEIVED,
    // TBA - TSPEC related Result Codes

    eSIR_SME_CHANNEL_SWITCH_FAIL,        // failed to send out Channel Switch Action Frame
    eSIR_SME_INVALID_STA_ROLE,
    eSIR_SME_INVALID_STATE,
#ifdef GEN4_SCAN
    eSIR_SME_CHANNEL_SWITCH_DISABLED,    // either 11h is disabled or channelSwitch is currently active
    eSIR_SME_HAL_SCAN_INIT_FAILED,       // SIR_HAL_SIR_HAL_INIT_SCAN_RSP returned failed status
    eSIR_SME_HAL_SCAN_START_FAILED,      // SIR_HAL_START_SCAN_RSP returned failed status
    eSIR_SME_HAL_SCAN_END_FAILED,        // SIR_HAL_END_SCAN_RSP returned failed status
    eSIR_SME_HAL_SCAN_FINISH_FAILED,     // SIR_HAL_FINISH_SCAN_RSP returned failed status
    eSIR_SME_HAL_SEND_MESSAGE_FAIL,      // Failed to send a message to HAL
#else // GEN4_SCAN
    eSIR_SME_CHANNEL_SWITCH_DISABLED,    // either 11h is disabled or channelSwitch is currently active
    eSIR_SME_HAL_SEND_MESSAGE_FAIL,      // Failed to send a message to HAL
#endif // GEN4_SCAN
#ifdef FEATURE_OEM_DATA_SUPPORT
    eSIR_SME_HAL_OEM_DATA_REQ_START_FAILED,
#endif
    eSIR_SME_STOP_BSS_FAILURE,           // Failed to stop the bss
    eSIR_SME_STA_ASSOCIATED,
    eSIR_SME_INVALID_PMM_STATE,
    eSIR_SME_CANNOT_ENTER_IMPS,
    eSIR_SME_IMPS_REQ_FAILED,
    eSIR_SME_BMPS_REQ_FAILED,
    eSIR_SME_BMPS_REQ_REJECT,
    eSIR_SME_UAPSD_REQ_FAILED,
    eSIR_SME_WOWL_ENTER_REQ_FAILED,
    eSIR_SME_WOWL_EXIT_REQ_FAILED,
#if defined WLAN_FEATURE_VOWIFI_11R
    eSIR_SME_FT_REASSOC_TIMEOUT_FAILURE,
    eSIR_SME_FT_REASSOC_FAILURE,
#endif
#ifdef WLAN_FEATURE_P2P
    eSIR_SME_SEND_ACTION_FAIL,
#endif
#ifdef WLAN_FEATURE_PACKET_FILTERING
    eSIR_SME_PC_FILTER_MATCH_COUNT_REQ_FAILED,
#endif // WLAN_FEATURE_PACKET_FILTERING
    
#ifdef WLAN_FEATURE_GTK_OFFLOAD
    eSIR_SME_GTK_OFFLOAD_GETINFO_REQ_FAILED,
#endif // WLAN_FEATURE_GTK_OFFLOAD
    eSIR_DONOT_USE_RESULT_CODE = SIR_MAX_ENUM_SIZE    
} tSirResultCodes;

/* each station added has a rate mode which specifies the sta attributes */
typedef enum eStaRateMode {
    eSTA_TAURUS = 0,
    eSTA_TITAN,
    eSTA_POLARIS,
    eSTA_11b,
    eSTA_11bg,
    eSTA_11a,
    eSTA_11n,
#ifdef WLAN_FEATURE_11AC
    eSTA_11ac,
#endif
    eSTA_INVALID_RATE_MODE
} tStaRateMode, *tpStaRateMode;

//although in tSirSupportedRates each IE is 16bit but PE only passes IEs in 8 bits with MSB=1 for basic rates.
//change the mask for bit0-7 only so HAL gets correct basic rates for setting response rates.
#define IERATE_BASICRATE_MASK     0x80
#define IERATE_RATE_MASK          0x7f
#define IERATE_IS_BASICRATE(x)   ((x) & IERATE_BASICRATE_MASK)
#define ANIENHANCED_TAURUS_RATEMAP_BITOFFSET_START  28

typedef struct sSirSupportedRates {
    /*
    * For Self STA Entry: this represents Self Mode.
    * For Peer Stations, this represents the mode of the peer.
    * On Station:
    * --this mode is updated when PE adds the Self Entry.
    * -- OR when PE sends 'ADD_BSS' message and station context in BSS is used to indicate the mode of the AP.
    * ON AP:
    * -- this mode is updated when PE sends 'ADD_BSS' and Sta entry for that BSS is used
    *     to indicate the self mode of the AP.
    * -- OR when a station is associated, PE sends 'ADD_STA' message with this mode updated.
    */

    tStaRateMode        opRateMode;
    // 11b, 11a and aniLegacyRates are IE rates which gives rate in unit of 500Kbps
    tANI_U16             llbRates[SIR_NUM_11B_RATES];
    tANI_U16             llaRates[SIR_NUM_11A_RATES];
    tANI_U16             aniLegacyRates[SIR_NUM_POLARIS_RATES];

    //Taurus only supports 26 Titan Rates(no ESF/concat Rates will be supported)
    //First 26 bits are reserved for those Titan rates and
    //the last 4 bits(bit28-31) for Taurus, 2(bit26-27) bits are reserved.
    tANI_U32             aniEnhancedRateBitmap; //Titan and Taurus Rates

    /*
    * 0-76 bits used, remaining reserved
    * bits 0-15 and 32 should be set.
    */
    tANI_U8 supportedMCSSet[SIR_MAC_MAX_SUPPORTED_MCS_SET];

    /*
     * RX Highest Supported Data Rate defines the highest data
     * rate that the STA is able to receive, in unites of 1Mbps.
     * This value is derived from "Supported MCS Set field" inside
     * the HT capability element.
     */
    tANI_U16 rxHighestDataRate;

#ifdef WLAN_FEATURE_11AC
   /*Indicates the Maximum MCS that can be received for each number
        of spacial streams */
    tANI_U16 vhtRxMCSMap;
   /*Indicate the highest VHT data rate that the STA is able to receive*/
    tANI_U16 vhtRxHighestDataRate;
   /*Indicates the Maximum MCS that can be transmitted	for each number
        of spacial streams */
    tANI_U16 vhtTxMCSMap;
   /*Indicate the highest VHT data rate that the STA is able to transmit*/
    tANI_U16 vhtTxHighestDataRate;
#endif
} tSirSupportedRates, *tpSirSupportedRates;


typedef enum eSirRFBand
{
    SIR_BAND_UNKNOWN,
    SIR_BAND_2_4_GHZ,
    SIR_BAND_5_GHZ,
} tSirRFBand;


/*
* Specifies which beacons are to be indicated upto the host driver when
* Station is in power save mode.
*/
typedef enum eBeaconForwarding
{
    ePM_BEACON_FWD_NTH,
    ePM_BEACON_FWD_TIM,
    ePM_BEACON_FWD_DTIM,
    ePM_BEACON_FWD_NONE
} tBeaconForwarding;


#ifdef WLAN_FEATURE_P2P
typedef struct sSirRemainOnChnReq
{
    tANI_U16 messageType;
    tANI_U16 length;
    tANI_U8 sessionId;
    tSirMacAddr selfMacAddr;
    tANI_U8  chnNum;
    tANI_U8  phyMode;
    tANI_U32 duration;
    tANI_U8  probeRspIe[1];
}tSirRemainOnChnReq, *tpSirRemainOnChnReq;

typedef struct sSirRegisterMgmtFrame
{
    tANI_U16 messageType;
    tANI_U16 length;
    tANI_U8 sessionId;
    tANI_BOOLEAN registerFrame;
    tANI_U16 frameType;
    tANI_U16 matchLen;
    tANI_U8  matchData[1];
}tSirRegisterMgmtFrame, *tpSirRegisterMgmtFrame;
#endif

//
// Identifies the neighbor BSS' that was(were) detected
// by an STA and reported to the AP
//
typedef struct sAniTitanCBNeighborInfo
{
  // A BSS was found on the Primary
  tANI_U8 cbBssFoundPri;

  // A BSS was found on the adjacent Upper Secondary
  tANI_U8 cbBssFoundSecUp;

  // A BSS was found on the adjacent Lower Secondary
  tANI_U8 cbBssFoundSecDown;

} tAniTitanCBNeighborInfo, *tpAniTitanCBNeighborInfo;

/// Generic type for sending a response message
/// with result code to host software
typedef struct sSirSmeRsp
{
    tANI_U16             messageType; // eWNI_SME_*_RSP
    tANI_U16             length;
    tANI_U8              sessionId;  // To support BT-AMP
    tANI_U16             transactionId;   // To support BT-AMP
    tSirResultCodes statusCode;
} tSirSmeRsp, *tpSirSmeRsp;

/// Definition for kick starting Firmware on STA
typedef struct sSirSmeStartReq
{
    tANI_U16   messageType;      // eWNI_SME_START_REQ
    tANI_U16   length;    
    tANI_U8      sessionId;      //Added for BT-AMP Support
    tANI_U16     transcationId;  //Added for BT-AMP Support
    tSirMacAddr  bssId;          //Added For BT-AMP Support   
    tANI_U32   roamingAtPolaris;
#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA) || defined(ANI_AP_CLIENT_SDK)
    tANI_U32   sendNewBssInd;
#endif
} tSirSmeStartReq, *tpSirSmeStartReq;

/// Definition for indicating all modules ready on STA
typedef struct sSirSmeReadyReq
{
    tANI_U16   messageType; // eWNI_SME_SYS_READY_IND
    tANI_U16   length;
    tANI_U16   transactionId;     
} tSirSmeReadyReq, *tpSirSmeReadyReq;

/// Definition for response message to previously issued start request
typedef struct sSirSmeStartRsp
{
    tANI_U16             messageType; // eWNI_SME_START_RSP
    tANI_U16             length;
    tSirResultCodes statusCode;
    tANI_U16             transactionId;     
} tSirSmeStartRsp, *tpSirSmeStartRsp;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
/**
 * Trigger Type
 */
typedef enum {
    // During Initialization and NM triggers only
    eSIR_TRIGGER_INIT_NM,

    // During Initialization, NM triggers and Periodic evaluation
    eSIR_TRIGGER_INIT_NM_PERIODIC
}tSirDfsTrigType;
#endif

/// Definition for Load structure
typedef struct sSirLoad
{
    tANI_U16             numStas;
    tANI_U16             channelUtilization;
} tSirLoad, *tpSirLoad;

/// BSS type enum used in while scanning/joining etc
typedef enum eSirBssType
{
    eSIR_INFRASTRUCTURE_MODE,
#ifdef WLAN_SOFTAP_FEATURE
    eSIR_INFRA_AP_MODE,                    //Added for softAP support
#endif
    eSIR_IBSS_MODE,
    eSIR_BTAMP_STA_MODE,                     //Added for BT-AMP support
    eSIR_BTAMP_AP_MODE,                     //Added for BT-AMP support
    eSIR_AUTO_MODE,
    eSIR_DONOT_USE_BSS_TYPE = SIR_MAX_ENUM_SIZE
} tSirBssType;

/// Definition for WDS Information
typedef struct sSirWdsInfo
{
    tANI_U16                wdsLength;
    tANI_U8                 wdsBytes[ANI_WDS_INFO_MAX_LENGTH];
} tSirWdsInfo, *tpSirWdsInfo;

/// Power Capability info used in 11H
typedef struct sSirMacPowerCapInfo
{
    tANI_U8              minTxPower;
    tANI_U8              maxTxPower;
} tSirMacPowerCapInfo, *tpSirMacPowerCapInfo;

/// Supported Channel info used in 11H
typedef struct sSirSupChnl
{
    tANI_U8              numChnl;
    tANI_U8              channelList[SIR_MAX_SUPPORTED_CHANNEL_LIST];
} tSirSupChnl, *tpSirSupChnl;

typedef enum eSirNwType
{
    eSIR_11A_NW_TYPE,
    eSIR_11B_NW_TYPE,
    eSIR_11G_NW_TYPE,
    eSIR_11N_NW_TYPE,
#ifdef WLAN_FEATURE_11AC
    eSIR_11AC_NW_TYPE,
#endif
    eSIR_DONOT_USE_NW_TYPE = SIR_MAX_ENUM_SIZE
} tSirNwType;

/// Definition for new iBss peer info
typedef struct sSirNewIbssPeerInfo
{
    tSirMacAddr    peerAddr;
    tANI_U16            aid;
} tSirNewIbssPeerInfo, *tpSirNewIbssPeerInfo;

/// Definition for Alternate BSS info
typedef struct sSirAlternateRadioInfo
{
    tSirMacAddr    bssId;
    tANI_U8             channelId;
} tSirAlternateRadioInfo, *tpSirAlternateRadioInfo;

/// Definition for Alternate BSS list
typedef struct sSirAlternateRadioList
{
    tANI_U8                       numBss;
    tSirAlternateRadioInfo   alternateRadio[1];
} tSirAlternateRadioList, *tpSirAlternateRadioList;

/// Definition for kick starting BSS
/// ---> MAC
/**
 * Usage of ssId, numSSID & ssIdList:
 * ---------------------------------
 * 1. ssId.length of zero indicates that Broadcast/Suppress SSID
 *    feature is enabled.
 * 2. If ssId.length is zero, MAC SW will advertise NULL SSID
 *    and interpret the SSID list from numSSID & ssIdList.
 * 3. If ssId.length is non-zero, MAC SW will advertise the SSID
 *    specified in the ssId field and it is expected that
 *    application will set numSSID to one (only one SSID present
 *    in the list) and SSID in the list is same as ssId field.
 * 4. Application will always set numSSID >= 1.
 */
//*****NOTE: Please make sure all codes are updated if inserting field into this structure..**********
typedef struct sSirSmeStartBssReq
{
    tANI_U16                messageType;       // eWNI_SME_START_BSS_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;       //Added for BT-AMP Support
    tANI_U16                transactionId;   //Added for BT-AMP Support
    tSirMacAddr             bssId;           //Added for BT-AMP Support
    tSirMacAddr             selfMacAddr;     //Added for BT-AMP Support
    tANI_U16                beaconInterval;  //Added for BT-AMP Support
    tANI_U8                 dot11mode;
    tSirBssType             bssType;
    tSirMacSSid             ssId;
    tANI_U8                 channelId;
    ePhyChanBondState       cbMode;
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
    tSirAlternateRadioList  alternateRadioList;
    tANI_S8                 powerLevel;
    tSirWdsInfo             wdsInfo;
#endif
    
#ifdef WLAN_SOFTAP_FEATURE
    tANI_U8                 privacy;
    tANI_U8                 apUapsdEnable;
    tANI_U8                 ssidHidden;
    tANI_BOOLEAN            fwdWPSPBCProbeReq;
    tANI_BOOLEAN            protEnabled;
    tANI_BOOLEAN            obssProtEnabled;
    tANI_U16                ht_capab;
    tAniAuthType            authType;
    tANI_U32                dtimPeriod;
    tANI_U8                 wps_state;
#endif
    tVOS_CON_MODE           bssPersona;

    tSirRSNie               rsnIE;             // RSN IE to be sent in
                                               // Beacon and Probe
                                               // Response frames
    tSirNwType              nwType;            // Indicates 11a/b/g
    tSirMacRateSet          operationalRateSet;// Has 11a or 11b rates
    tSirMacRateSet          extendedRateSet;    // Has 11g rates

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U8                 numSSID;
    tSirMacSSid             ssIdList[ANI_MAX_NUM_OF_SSIDS];
#endif
} tSirSmeStartBssReq, *tpSirSmeStartBssReq;

#define GET_IE_LEN_IN_BSS(lenInBss) ( lenInBss + sizeof(lenInBss) - \
              ((int) OFFSET_OF( tSirBssDescription, ieFields)))

#define WSCIE_PROBE_RSP_LEN (317 + 2)

typedef struct sSirBssDescription
{
    //offset of the ieFields from bssId.
    tANI_U16             length;
    tSirMacAddr          bssId;
    v_TIME_t             scanSysTimeMsec;
    tANI_U32             timeStamp[2];
    tANI_U16             beaconInterval;
    tANI_U16             capabilityInfo;
    tSirNwType           nwType; // Indicates 11a/b/g
    tANI_U8              aniIndicator;
    tANI_S8              rssi;
    tANI_S8              sinr;
    //channelId what peer sent in beacon/probersp.
    tANI_U8              channelId;
    //channelId on which we are parked at.
    //used only in scan case.
    tANI_U8              channelIdSelf;
    tANI_U8              sSirBssDescriptionRsvd[3];
    tANI_TIMESTAMP nReceivedTime;     //base on a tick count. It is a time stamp, not a relative time.
#if defined WLAN_FEATURE_VOWIFI
    tANI_U32       parentTSF;
    tANI_U32       startTSF[2];
#endif
#ifdef WLAN_FEATURE_VOWIFI_11R
    tANI_U8              mdiePresent;
    tANI_U8              mdie[SIR_MDIE_SIZE];                // MDIE for 11r, picked from the beacons
#endif
#ifdef FEATURE_WLAN_CCX
    tANI_U16             QBSSLoad_present;
    tANI_U16             QBSSLoad_avail; 
#endif
    // Please keep the structure 4 bytes aligned above the ieFields

    tANI_U8              fProbeRsp; //whether it is from a probe rsp
    tANI_U8              reservedPadding1;
    tANI_U8              reservedPadding2;
    tANI_U8              reservedPadding3;
    tANI_U32             WscIeLen;
    tANI_U8              WscIeProbeRsp[WSCIE_PROBE_RSP_LEN];
    tANI_U8              reservedPadding4;

    tANI_U32             ieFields[1];
} tSirBssDescription, *tpSirBssDescription;

/// Definition for response message to previously
/// issued start BSS request
/// MAC --->
typedef struct sSirSmeStartBssRsp
{
    tANI_U16            messageType; // eWNI_SME_START_BSS_RSP
    tANI_U16            length;
    tANI_U8             sessionId;
    tANI_U16            transactionId;//transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirBssType         bssType;//Add new type for WDS mode
    tANI_U16            beaconInterval;//Beacon Interval for both type
    tANI_U32            staId;//Staion ID for Self  
    tSirBssDescription  bssDescription;//Peer BSS description
} tSirSmeStartBssRsp, *tpSirSmeStartBssRsp;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
typedef struct sSirMeasControl
{
    // Periodic Measurements enabled flag
    tAniBool             periodicMeasEnabled;
    // Indicates whether to involve STAs in measurements or not
    tAniBool             involveSTAs;
    // Basic or enhanced measurement metrics
    // 0 - for basic, 1 - for enhanced
    tANI_U8                   metricsType;
    // Indicates active or passive scanning
    tSirScanType         scanType;
    // Following indicates how often measurements
    // on each channel are made for long-scan-duration
    tANI_U8                   longChannelScanPeriodicity;
    //
    // Channel Bonding plus 11H related scan control
    // 0 - CB and/or 11H is disabled
    // 1 - CB and 11H is enabled
    //
    tANI_BOOLEAN    cb11hEnabled;
} tSirMeasControl, *tpSirMeasControl;

typedef struct sSirMeasDuration
{
    // Following indicates time duration over which
    // all channels in the channelList to be measured once
    tANI_U32                  shortTermPeriod;
    // Following indicates time duration over which
    // cached DFS measurements are averaged
    tANI_U32                  averagingPeriod;
    // Following indicates time duration for making
    // DFS measurements on each channel
    tANI_U32                  shortChannelScanDuration;
    // Following indicates time duration for making
    // DFS measurements on each channel for long term measurements
    tANI_U32                  longChannelScanDuration;
} tSirMeasDuration, *tpSirMeasDuration;
#endif

typedef struct sSirChannelList
{
    tANI_U8          numChannels;
    tANI_U8          channelNumber[1];
} tSirChannelList, *tpSirChannelList;

#ifdef FEATURE_WLAN_CCX
typedef struct sTspecInfo {
    tANI_U8         valid;
    tSirMacTspecIE  tspec;
} tTspecInfo;

#define SIR_CCX_MAX_TSPEC_IES   4
typedef struct sCCXTspecTspecInfo {
    tANI_U8 numTspecs;
    tTspecInfo tspec[SIR_CCX_MAX_TSPEC_IES];
} tCCXTspecInfo;
#endif

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
/// Definition for Neighbor BSS info
typedef struct sSirNeighborBssInfo
{
    tSirMacAddr             bssId;
    tANI_U8                 channelId;
    tAniBool                wniIndicator;
    tSirBssType             bssType;
    tANI_U8                 sinr;
    tANI_S8                 rssi;
    tSirLoad                load;
    tAniSSID                ssId;
    tAniApName              apName;
    tSirRSNie               rsnIE;
#if 0    
    tDot11fIEHTCaps         HTCaps;
    tDot11fIEHTInfo         HTInfo;
#endif
    tSirNwType              nwType;             // Indicates 11a/b/g
    tANI_U16                capabilityInfo;
    tSirMacRateSet          operationalRateSet; // Has 11a or 11b rates
    tSirMacRateSet          extendedRateSet;    // Has 11g rates
    tANI_U16                beaconInterval;
    tANI_U8                 dtimPeriod;
    tANI_U8                 HTCapsPresent;
    tANI_U8                 HTInfoPresent;
    tANI_U8                 wmeInfoPresent;
    tANI_U8                 wmeEdcaPresent;
    tANI_U8                 wsmCapablePresent;
    tANI_U8                 hcfEnabled;
    tANI_U16                propIECapability;
    tANI_U32                localPowerConstraints;
    tANI_S32                aggrRssi;
    tANI_U32                dataCount;
    tANI_U32                totalPackets;
} tSirNeighborBssInfo, *tpSirNeighborBssInfo;


/// Definition for Neighbor BSS with WDS info
typedef struct sSirNeighborBssWdsInfo
{
    tSirNeighborBssInfo    neighborBssInfo;
    tSirWdsInfo            wdsInfo;
} tSirNeighborBssWdsInfo, *tpSirNeighborBssWdsInfo;

/// Definition for Neighbor BSS list
typedef struct sSirNeighborBssList
{
    tANI_U32                  numBss;
    tSirNeighborBssInfo  bssList[1];
} tSirNeighborBssList, *tpSirNeighborBssList;

/// Definition for Neighbor BSS list with WDS info
typedef struct sSirNeighborBssWdsList
{
    tANI_U32                     numBssWds;
    tSirNeighborBssWdsInfo  bssWdsList[1];
} tSirNeighborBssWdsList, *tpSirNeighborBssWdsList;

#if (WNI_POLARIS_FW_PRODUCT == AP)
/// Definition for kick starting measurements
/// ---> MAC
typedef struct sSirSmeMeasurementReq
{
    tANI_U16                  messageType; // eWNI_SME_MEASUREMENT_REQ
    tANI_U16                  length;
    tSirMeasControl      measControl;
    tSirMeasDuration     measDuration;
    // This indicates how often SME_MEASUREMENT_IND message
    // is sent to host
    tANI_U32                  measIndPeriod;
    // This channel list will have current channel also
    tSirChannelList      channelList;
} tSirSmeMeasurementReq, *tpSirSmeMeasurementReq;

/// Definition for response message to previously
/// issued Measurement request
/// MAC --->
typedef struct sSirSmeMeasurementRsp
{
    tANI_U16                    messageType; // eWNI_SME_MEASUREMENT_RSP
    tANI_U16                    length;
    tSirResultCodes        statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeMeasurementRsp, *tpSirSmeMeasurementRsp;

/// Definition for Measurement Matrix info
// NOTE: This should include current channel measurements
typedef struct sSirMeasMatrixInfo
{
    tANI_U8          channelNumber;
    tANI_S8          compositeRssi;
    tANI_S32         aggrRssi;     // Due to all packets in this channel
    tANI_U32         totalPackets;
} tSirMeasMatrixInfo, *tpSirMeasMatrixInfo;

/// Definition for Measurement Matrix List
typedef struct sSirMeasMatrixList
{
    tANI_U8                  numChannels;
    tSirMeasMatrixInfo  measMatrixList[1];
} tSirMeasMatrixList, *tpSirMeasMatrixList;
/// Definition for Measurement indication
/// MAC --->
/// Note : This is only sent when there was prior
///        SME_MEASUREMENT_REQ received by MAC
typedef struct sSirSmeMeasurementInd
{
    tANI_U16                    messageType; // eWNI_SME_MEASUREMENT_IND
    tANI_U16                    length;
    tANI_U32               duration;
    tSirLoad               currentLoad; // on this AP radio
    tSirMeasMatrixList     measMatrixList;
    tSirNeighborBssWdsList neighborBssWdsList;
} tSirSmeMeasurementInd, *tpSirSmeMeasurementInd;

/// Definition for Backhaul Link status change Indication
/// MAC --->
typedef struct sSirSmeWdsInfoInd
{
    tANI_U16                messageType; // eWNI_SME_WDS_INFO_IND
    tANI_U16                length;
    tSirWdsInfo        wdsInfo;
} tSirSmeWdsInfoInd, *tpSirSmeWdsInfoInd;

/// Definition for Backhaul Link Info Set Request
/// ---> MAC
typedef struct sSirSmeSetWdsInfoReq
{
    tANI_U16                messageType; // eWNI_SME_SET_WDS_INFO_REQ
    tANI_U16                length;
    tANI_U16                transactionId;     
    tSirWdsInfo        wdsInfo;
} tSirSmeSetWdsInfoReq, *tpSirSmeSetWdsInfoReq;

/// Definition for Backhaul Link Lnfo Set Response
/// MAC --->
typedef struct sSirSmeSetWdsInfoRsp
{
    tANI_U16                messageType; // eWNI_SME_SET_WDS_INFO_RSP
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tANI_U16                transactionId;     
} tSirSmeSetWdsInfoRsp, *tpSirSmeSetWdsInfoRsp;
#endif
#endif

/// Definition for Radar Info
typedef struct sSirRadarInfo
{
    tANI_U8          channelNumber;
    tANI_U16         radarPulseWidth; // in usecond
    tANI_U16         numRadarPulse;
} tSirRadarInfo, *tpSirRadarInfo;

#define SIR_RADAR_INFO_SIZE                (sizeof(tANI_U8) + 2 *sizeof(tANI_U16))

/// Two Background Scan mode
typedef enum eSirBackgroundScanMode
{
    eSIR_AGGRESSIVE_BACKGROUND_SCAN = 0,
    eSIR_NORMAL_BACKGROUND_SCAN = 1,
    eSIR_ROAMING_SCAN = 2,
} tSirBackgroundScanMode;

/// Two types of traffic check
typedef enum eSirLinkTrafficCheck
{
    eSIR_DONT_CHECK_LINK_TRAFFIC_BEFORE_SCAN = 0,
    eSIR_CHECK_LINK_TRAFFIC_BEFORE_SCAN = 1,
    eSIR_CHECK_ROAMING_SCAN = 2,
} tSirLinkTrafficCheck;

#define SIR_BG_SCAN_RETURN_CACHED_RESULTS              0x0
#define SIR_BG_SCAN_PURGE_RESUTLS                      0x80
#define SIR_BG_SCAN_RETURN_FRESH_RESULTS               0x01
#define SIR_SCAN_MAX_NUM_SSID                          0x09 

/// Definition for scan request
typedef struct sSirSmeScanReq
{
    tANI_U16        messageType; // eWNI_SME_SCAN_REQ
    tANI_U16        length;
    tANI_U8         sessionId;         // Session ID
    tANI_U16        transactionId;     // Transaction ID for cmd
    tSirMacAddr     bssId;
    tSirMacSSid     ssId[SIR_SCAN_MAX_NUM_SSID];
    tSirMacAddr     selfMacAddr; //Added For BT-AMP Support
    tSirBssType     bssType;
    tANI_U8         dot11mode;
    tSirScanType    scanType;
    /**
     * minChannelTime. Not used if scanType is passive.
     * 0x0 - Dont Use min channel timer. Only max channel timeout will used.
     *       11k measurements set this to zero to user only single duration for scan.
     * <valid timeout> - Timeout value used for min channel timeout.
     */
    tANI_U32        minChannelTime;
    /**
     * maxChannelTime.
     * 0x0 - Invalid. In case of active scan.
     * In case of passive scan, MAX( maxChannelTime, WNI_CFG_PASSIVE_MAXIMUM_CHANNEL_TIME) is used. 
     *
     */
    tANI_U32        maxChannelTime;
    /**
     * returnAfterFirstMatch can take following values:
     * 0x00 - Return SCAN_RSP message after complete channel scan
     * 0x01 -  Return SCAN_RSP message after collecting BSS description
     *        that matches scan criteria.
     * 0xC0 - Return after collecting first 11d IE from 2.4 GHz &
     *        5 GHz band channels
     * 0x80 - Return after collecting first 11d IE from 5 GHz band
     *        channels
     * 0x40 - Return after collecting first 11d IE from 2.4 GHz
     *        band channels
     *
     * Values of 0xC0, 0x80 & 0x40 are to be used by
     * Roaming/application when 11d is enabled.
     */
    tANI_U8              returnAfterFirstMatch;

    /**
     * returnUniqueResults can take following values:
     * 0 - Collect & report all received BSS descriptions from same BSS.
     * 1 - Collect & report unique BSS description from same BSS.
     */
    tANI_U8              returnUniqueResults;

    /**
     * returnFreshResults can take following values:
     * 0x00 - Return background scan results.
     * 0x80 - Return & purge background scan results
     * 0x01 - Trigger fresh scan instead of returning background scan
     *        results.
     * 0x81 - Trigger fresh scan instead of returning background scan
     *        results and purge background scan results.
     */
    tANI_U8              returnFreshResults;

    /*  backgroundScanMode can take following values:
     *  0x0 - agressive scan
     *  0x1 - normal scan where HAL will check for link traffic 
     *        prior to proceeding with the scan
     */
    tSirBackgroundScanMode   backgroundScanMode;

    tANI_U8              hiddenSsid;

    /* Number of SSIDs to scan */
    tANI_U8             numSsid;
    
    //channelList has to be the last member of this structure. Check tSirChannelList for the reason.
    /* This MUST be the last field of the structure */
    
 
#ifdef WLAN_FEATURE_P2P
    tANI_BOOLEAN         p2pSearch;
    tANI_BOOLEAN         skipDfsChnlInP2pSearch;
#endif
    tANI_U16             uIEFieldLen;
    tANI_U16             uIEFieldOffset;

    //channelList MUST be the last field of this structure
    tSirChannelList channelList;
    /*-----------------------------
      tSirSmeScanReq....
      -----------------------------
      uIEFiledLen 
      -----------------------------
      uIEFiledOffset               ----+
      -----------------------------    |
      channelList.numChannels          |
      -----------------------------    |
      ... variable size up to          |
      channelNumber[numChannels-1]     |
      This can be zero, if             |
      numChannel is zero.              |
      ----------------------------- <--+
      ... variable size uIEFiled 
      up to uIEFieldLen (can be 0)
      -----------------------------*/
} tSirSmeScanReq, *tpSirSmeScanReq;

#ifdef FEATURE_OEM_DATA_SUPPORT

#ifndef OEM_DATA_REQ_SIZE
#define OEM_DATA_REQ_SIZE 70
#endif
#ifndef OEM_DATA_RSP_SIZE
#define OEM_DATA_RSP_SIZE 968
#endif

typedef struct sSirOemDataReq
{
    tANI_U16              messageType; //eWNI_SME_OEM_DATA_REQ
    tSirMacAddr           selfMacAddr;
    tANI_U8               oemDataReq[OEM_DATA_REQ_SIZE];
} tSirOemDataReq, *tpSirOemDataReq;

typedef struct sSirOemDataRsp
{
    tANI_U16             messageType;
    tANI_U16             length;
    tANI_U8              oemDataRsp[OEM_DATA_RSP_SIZE];
} tSirOemDataRsp, *tpSirOemDataRsp;
    
#endif //FEATURE_OEM_DATA_SUPPORT

/// Definition for response message to previously issued scan request
typedef struct sSirSmeScanRsp
{
    tANI_U16           messageType; // eWNI_SME_SCAN_RSP
    tANI_U16           length;
    tANI_U8            sessionId;     
    tSirResultCodes    statusCode;
    tANI_U16           transcationId; 
    tSirBssDescription bssDescription[1];
} tSirSmeScanRsp, *tpSirSmeScanRsp;

/// Sme Req message to set the Background Scan mode
typedef struct sSirSmeBackgroundScanModeReq
{
    tANI_U16                      messageType; // eWNI_SME_BACKGROUND_SCAN_MODE_REQ
    tANI_U16                      length;
    tSirBackgroundScanMode   mode;
} tSirSmeBackgroundScanModeReq, *tpSirSmeBackgroundScanModeReq;

/// Background Scan Statisics
typedef struct sSirBackgroundScanInfo {
    tANI_U32        numOfScanSuccess;
    tANI_U32        numOfScanFailure;
    tANI_U32        reserved;
} tSirBackgroundScanInfo, *tpSirBackgroundScanInfo;

#define SIR_BACKGROUND_SCAN_INFO_SIZE        (3 * sizeof(tANI_U32))

/// Definition for Authentication request
typedef struct sSirSmeAuthReq
{
    tANI_U16           messageType; // eWNI_SME_AUTH_REQ
    tANI_U16           length;
    tANI_U8            sessionId;        // Session ID
    tANI_U16           transactionId;    // Transaction ID for cmd
    tSirMacAddr        bssId;            // Self BSSID
    tSirMacAddr        peerMacAddr;
    tAniAuthType       authType;
    tANI_U8            channelNumber;
} tSirSmeAuthReq, *tpSirSmeAuthReq;

/// Definition for reponse message to previously issued Auth request
typedef struct sSirSmeAuthRsp
{
    tANI_U16           messageType; // eWNI_SME_AUTH_RSP
    tANI_U16           length;
    tANI_U8            sessionId;      // Session ID
    tANI_U16           transactionId;  // Transaction ID for cmd
    tSirMacAddr        peerMacAddr;
    tAniAuthType       authType;
    tSirResultCodes    statusCode;
    tANI_U16           protStatusCode; //It holds reasonCode when Pre-Auth fails due to deauth frame.
                                       //Otherwise it holds status code.
} tSirSmeAuthRsp, *tpSirSmeAuthRsp;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
/// Association type
typedef enum eSirAssocType
{
    eSIR_NORMAL,
    eSIR_TRANSFERRED,
    eSIR_DONOT_USE_ASSOC_TYPE = SIR_MAX_ENUM_SIZE
} tSirAssocType;
#endif

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) 
typedef enum eSirBpIndicatorType
{
    eSIR_WIRELESS_BP,
    eSIR_WIRED_BP
} tSirBpIndicatorType;

#endif

#ifdef FEATURE_WLAN_INTEGRATED_SOC
/// Definition for Join/Reassoc info - Reshmi: need to check if this is a def which moved from elsehwere.
typedef struct sJoinReassocInfo
{
    tAniTitanCBNeighborInfo cbNeighbors;
    tAniBool            spectrumMgtIndicator;
    tSirMacPowerCapInfo powerCap;
    tSirSupChnl         supportedChannels;
} tJoinReassocInfo, *tpJoinReassocInfo;
#endif

/// Definition for join request
/// ---> MAC
/// WARNING! If you add a field in JOIN REQ. 
///         Make sure to add it in REASSOC REQ 
/// The Serdes function is the same and its 
/// shared with REASSOC. So if we add a field
//  here and dont add it in REASSOC REQ. It will BREAK!!! REASSOC.
typedef struct sSirSmeJoinReq
{
    tANI_U16            messageType;            // eWNI_SME_JOIN_REQ
    tANI_U16            length;
    tANI_U8             sessionId;
    tANI_U16            transactionId;  
    tSirMacSSid         ssId;
    tSirMacAddr         selfMacAddr;            // self Mac address
    tSirBssType         bsstype;                // add new type for BT -AMP STA and AP Modules
    tANI_U8             dot11mode;              // to support BT-AMP     
    tVOS_CON_MODE       staPersona;             //Persona
    ePhyChanBondState   cbMode;                 // Pass CB mode value in Join.

    /*This contains the UAPSD Flag for all 4 AC
     * B0: AC_VO UAPSD FLAG
     * B1: AC_VI UAPSD FLAG
     * B2: AC_BK UAPSD FLAG
     * B3: AC_BE UASPD FLAG
     */
    tANI_U8                 uapsdPerAcBitmask;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
    tSirAssocType       assocType;              // Indicates whether STA is
                                                // sending (Re) Association
                                                // due to load balance or not
#endif
    tSirMacRateSet      operationalRateSet;// Has 11a or 11b rates
    tSirMacRateSet      extendedRateSet;    // Has 11g rates
    tSirRSNie           rsnIE;                  // RSN IE to be sent in
                                                // (Re) Association Request
#ifdef FEATURE_WLAN_CCX
    tSirCCKMie          cckmIE;             // CCMK IE to be included as handler for join and reassoc is 
                                            // the same. The join will never carry cckm, but will be set to
                                            // 0. 
#endif

    tSirAddie           addIEScan;              // Additional IE to be sent in
                                                // (unicast) Probe Request at the time of join

    tSirAddie           addIEAssoc;             // Additional IE to be sent in 
                                                // (Re) Association Request

    tAniEdType          UCEncryptionType;

    tAniEdType          MCEncryptionType;
#ifdef WLAN_FEATURE_VOWIFI_11R
    tAniBool            is11Rconnection;
#endif
#ifdef FEATURE_WLAN_CCX
    tAniBool            isCCXconnection;
    tCCXTspecInfo       ccxTspecInfo;
#endif
    
#if defined WLAN_FEATURE_VOWIFI_11R || defined FEATURE_WLAN_CCX || defined(FEATURE_WLAN_LFR)
    tAniBool            isFastTransitionEnabled;
#endif
#ifdef FEATURE_WLAN_LFR
    tAniBool            isFastRoamIniFeatureEnabled;
#endif
    
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
    tAniBool            bpIndicator;
    tSirBpIndicatorType bpType;
    tSirNeighborBssList neighborBssList;        // TBD Move this outside 'AP'
                                                // flag
#endif
    tAniTitanCBNeighborInfo cbNeighbors;
    tAniBool            spectrumMgtIndicator;
    tSirMacPowerCapInfo powerCap;
    tSirSupChnl         supportedChannels;
//#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA )
    tSirBssDescription  bssDescription;
//#endif

} tSirSmeJoinReq, *tpSirSmeJoinReq;

/// Definition for reponse message to previously issued join request
/// MAC --->
typedef struct sSirSmeJoinRsp
{
    tANI_U16                messageType; // eWNI_SME_JOIN_RSP
    tANI_U16                length;
    tANI_U8                 sessionId;         // Session ID
    tANI_U16                transactionId;     // Transaction ID for cmd
    tSirResultCodes    statusCode;
#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA)
    tAniAuthType       authType;
//    tANI_U16           staId;             // Station ID for peer
#endif
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
    // Following are needed for Roaming algorithm
    // to 'associate' with an alternate BSS.
    tSirMacAddr        alternateBssId;
    tANI_U8                 alternateChannelId;
#endif
    tANI_U16        protStatusCode; //It holds reasonCode when join fails due to deauth/disassoc frame.
                                    //Otherwise it holds status code.
    tANI_U16        aid;
    tANI_U32        beaconLength;
    tANI_U32        assocReqLength;
    tANI_U32        assocRspLength;
#ifdef WLAN_FEATURE_VOWIFI_11R
    tANI_U32        parsedRicRspLen;
#endif
#ifdef FEATURE_WLAN_CCX
    tANI_U32        tspecIeLen;
#endif
    tANI_U32        staId;//Station ID for peer

    /*The DPU signatures will be sent eventually to TL to help it determine the 
      association to which a packet belongs to*/
    /*Unicast DPU signature*/
    tANI_U8            ucastSig;

    /*Broadcast DPU signature*/
    tANI_U8            bcastSig;

    tANI_U8         frames[ 1 ];
} tSirSmeJoinRsp, *tpSirSmeJoinRsp;

/// Definition for Authentication indication from peer
typedef struct sSirSmeAuthInd
{
    tANI_U16           messageType; // eWNI_SME_AUTH_IND
    tANI_U16           length;         
    tANI_U8            sessionId;
    tSirMacAddr        bssId;             // Self BSSID
    tSirMacAddr        peerMacAddr;
    tAniAuthType       authType;
} tSirSmeAuthInd, *tpSirSmeAuthInd;

/// probereq from peer, when wsc is enabled
typedef struct sSirSmeProbereq
{
    tANI_U16           messageType; // eWNI_SME_PROBE_REQ
    tANI_U16           length;
    tANI_U8            sessionId;
    tSirMacAddr        peerMacAddr;
    tANI_U16           devicePasswdId;
} tSirSmeProbeReq, *tpSirSmeProbeReq;

/// Definition for Association indication from peer
/// MAC --->
typedef struct sSirSmeAssocInd
{
    tANI_U16             messageType; // eWNI_SME_ASSOC_IND
    tANI_U16             length;
    tANI_U8              sessionId;
    tSirMacAddr          peerMacAddr;
    tANI_U16             aid;
    tSirMacAddr          bssId; // Self BSSID
    tANI_U16             staId; // Station ID for peer
    tANI_U8              uniSig;  // DPU signature for unicast packets
    tANI_U8              bcastSig; // DPU signature for broadcast packets
    tAniAuthType         authType;    
    tAniSSID             ssId; // SSID used by STA to associate
    tSirRSNie            rsnIE;// RSN IE received from peer
    tSirAddie            addIE;// Additional IE received from peer, which possibly include WSC IE and/or P2P IE

#if defined (ANI_PRODUCT_TYPE_AP)
    tANI_U16                  seqNum;
    tAniBool             wniIndicator;
    tAniBool             bpIndicator;
    tSirBpIndicatorType  bpType;
    tSirAssocType        assocType; // Indicates whether STA is LB'ed or not
    tSirLoad             load; // Current load on the radio for LB
    tSirNeighborBssList  neighborList; // List received from STA
    tANI_U16                  capabilityInfo; // STA capability
    tSirNwType           nwType;            // Indicates 11a/b/g
#endif
    // powerCap & supportedChannels are present only when
    // spectrumMgtIndicator flag is set
    tAniBool                spectrumMgtIndicator;
    tSirMacPowerCapInfo     powerCap;
    tSirSupChnl             supportedChannels;
#ifdef WLAN_SOFTAP_FEATURE
    tAniBool             wmmEnabledSta; /* if present - STA is WMM enabled */
    tAniBool             reassocReq;
    // Required for indicating the frames to upper layer
    tANI_U32             beaconLength;
    tANI_U8*             beaconPtr;
    tANI_U32             assocReqLength;
    tANI_U8*             assocReqPtr;
#endif
} tSirSmeAssocInd, *tpSirSmeAssocInd;


/// Definition for Association confirm
/// ---> MAC
typedef struct sSirSmeAssocCnf
{
    tANI_U16             messageType; // eWNI_SME_ASSOC_CNF
    tANI_U16             length;
    tSirResultCodes      statusCode;
    tSirMacAddr          bssId;      // Self BSSID
    tSirMacAddr          peerMacAddr;
    tANI_U16             aid;
    tSirMacAddr          alternateBssId;
    tANI_U8              alternateChannelId;
} tSirSmeAssocCnf, *tpSirSmeAssocCnf;

/// Definition for Reassociation request
/// ---> MAC
/// WARNING! If you add a field in REASSOC REQ. 
///         Make sure to add it in JOIN REQ 
/// The Serdes function is the same and its 
/// shared with REASSOC. So if we add a field
//  here and dont add it in JOIN REQ. It will BREAK!!! JOIN.
typedef struct sSirSmeReassocReq
{
    tANI_U16            messageType; // eWNI_SME_REASSOC_REQ
    tANI_U16            length;
    tANI_U8             sessionId;              
    tANI_U16            transactionId;  
    tSirMacSSid         ssId;
    tSirMacAddr         selfMacAddr;            // self Mac address
    tSirBssType         bsstype;                // add new type for BT -AMP STA and AP Modules
    tANI_U8             dot11mode;              // to support BT-AMP     
    tVOS_CON_MODE       staPersona;             //Persona
    ePhyChanBondState   cbMode;                 // CBMode value to be passed with reassoc req

    /*This contains the UAPSD Flag for all 4 AC
     * B0: AC_VO UAPSD FLAG
     * B1: AC_VI UAPSD FLAG
     * B2: AC_BK UAPSD FLAG
     * B3: AC_BE UASPD FLAG
     */
    tANI_U8             uapsdPerAcBitmask;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
    tSirAssocType       assocType; // Indicates whether STA is
                                   // sending (Re) Association
                                   // due to load balance or not
#endif

    tSirMacRateSet          operationalRateSet;// Has 11a or 11b rates
    tSirMacRateSet          extendedRateSet;    // Has 11g rates

    tSirRSNie           rsnIE;     // RSN IE to be sent in
                                   // (Re) Association Request

#ifdef FEATURE_WLAN_CCX
    tSirCCKMie          cckmIE;    // CCMK IE to be included in ReAssoc if length != 0.
#endif
    tSirAddie           addIEScan; // Addtional IE to be sent in
                                   // (unicast) Probe Request at the time of join

    tSirAddie           addIEAssoc; // Additional IE to be sent in 
                                    // (Re) Association Request
                                   
    tAniEdType          UCEncryptionType;
    tAniEdType          MCEncryptionType;
#ifdef WLAN_FEATURE_VOWIFI_11R
    tAniBool            is11Rconnection;
#endif
#ifdef FEATURE_WLAN_CCX
    tAniBool            isCCXconnection;
    tCCXTspecInfo       ccxTspecInfo;
#endif
#if defined WLAN_FEATURE_VOWIFI_11R || defined FEATURE_WLAN_CCX || defined(FEATURE_WLAN_LFR)
    tAniBool            isFastTransitionEnabled;
#endif
#ifdef FEATURE_WLAN_LFR
    tAniBool            isFastRoamIniFeatureEnabled;
#endif

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
    tAniBool             bpIndicator;
    tSirBpIndicatorType  bpType;
    tSirNeighborBssList  neighborBssList;
#endif
    tAniTitanCBNeighborInfo cbNeighbors;
    tAniBool                spectrumMgtIndicator;
    tSirMacPowerCapInfo     powerCap;
    tSirSupChnl             supportedChannels;
//#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA )
    tSirBssDescription  bssDescription;
//#endif
} tSirSmeReassocReq, *tpSirSmeReassocReq;

/// Definition for reponse message to previously issued
/// Reassociation request
typedef struct sSirSmeReassocRsp
{
    tANI_U16           messageType; // eWNI_SME_REASSOC_RSP
    tANI_U16           length;
    tANI_U8            sessionId;         // Session ID
    tANI_U16           transactionId;     // Transaction ID for cmd
    tSirResultCodes    statusCode;
    tANI_U8            staId;             // Station ID for peer
#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA)
    tAniAuthType       authType;
#endif
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
    tSirMacAddr        alternateBssId;
    tANI_U8                 alternateChannelId;
#endif
} tSirSmeReassocRsp, *tpSirSmeReassocRsp;

/// Definition for Reassociation indication from peer
typedef struct sSirSmeReassocInd
{
    tANI_U16            messageType; // eWNI_SME_REASSOC_IND
    tANI_U16            length;
    tANI_U8             sessionId;         // Session ID
    tSirMacAddr         peerMacAddr;
    tSirMacAddr         oldMacAddr;
    tANI_U16            aid;
    tSirMacAddr         bssId;           // Self BSSID
    tANI_U16            staId;           // Station ID for peer
    tAniAuthType        authType;
    tAniSSID            ssId;   // SSID used by STA to reassociate
    tSirRSNie           rsnIE;  // RSN IE received from peer

    tSirAddie           addIE;  // Additional IE received from peer
    
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
    tANI_U16             seqNum;
    tAniBool             wniIndicator;
    tAniBool             bpIndicator;
    tSirBpIndicatorType  bpType;
    tSirAssocType        reassocType; // Indicates whether STA is LB'ed or not
    tSirLoad             load; // Current load on the radio for LB
    tSirNeighborBssList  neighborList; // List received from STA
    tANI_U16             capabilityInfo; // STA capability
    tSirNwType           nwType;            // Indicates 11a/b/g
#endif
    // powerCap & supportedChannels are present only when
    // spectrumMgtIndicator flag is set
    tAniBool                spectrumMgtIndicator;
    tSirMacPowerCapInfo     powerCap;
    tSirSupChnl             supportedChannels;
#ifdef WLAN_SOFTAP_FEATURE
    // Required for indicating the frames to upper layer
    // TODO: use the appropriate names to distinguish between the other similar names used above for station mode of operation
    tANI_U32             beaconLength;
    tANI_U8*             beaconPtr;
    tANI_U32             assocReqLength;
    tANI_U8*             assocReqPtr;
#endif
} tSirSmeReassocInd, *tpSirSmeReassocInd;

/// Definition for Reassociation confirm
/// ---> MAC
typedef struct sSirSmeReassocCnf
{
    tANI_U16                  messageType; // eWNI_SME_REASSOC_CNF
    tANI_U16                  length;
    tSirResultCodes      statusCode;
    tSirMacAddr          bssId;             // Self BSSID
    tSirMacAddr          peerMacAddr;
    tANI_U16                  aid;
    tSirMacAddr          alternateBssId;
    tANI_U8                   alternateChannelId;
} tSirSmeReassocCnf, *tpSirSmeReassocCnf;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
typedef enum {
    // Based on Periodic evaluation
    eSIR_PERIODIC_EVALATION,

    // Dectection of primary users such as RADARs
    eSIR_DETECTION_OF_RADAR,

    // Degradation in system performance (eg. Triggered by
    //  increase in PER beyond a certain threshold)
    eSIR_PERFORMANCE_DEGRADATION,

    // Frequency changed due to previously issued SWITCH_CHANNEL_REQ
    eSIR_UPPER_LAYER_TRIGGERED
}tSirFreqChangeReason;
#endif

/// Enum definition for  Wireless medium status change codes
typedef enum eSirSmeStatusChangeCode
{
    eSIR_SME_DEAUTH_FROM_PEER,
    eSIR_SME_DISASSOC_FROM_PEER,
    eSIR_SME_LOST_LINK_WITH_PEER,
    eSIR_SME_CHANNEL_SWITCH,
    eSIR_SME_JOINED_NEW_BSS,
    eSIR_SME_LEAVING_BSS,
    eSIR_SME_IBSS_ACTIVE,
    eSIR_SME_IBSS_INACTIVE,
    eSIR_SME_IBSS_PEER_DEPARTED,
    eSIR_SME_RADAR_DETECTED,
    eSIR_SME_IBSS_NEW_PEER,
    eSIR_SME_AP_CAPS_CHANGED,
    eSIR_SME_BACKGROUND_SCAN_FAIL,
    eSIR_SME_CB_LEGACY_BSS_FOUND_BY_AP,
    eSIR_SME_CB_LEGACY_BSS_FOUND_BY_STA
} tSirSmeStatusChangeCode;

typedef struct sSirSmeNewBssInfo
{
    tSirMacAddr   bssId;
    tANI_U8            channelNumber;
    tANI_U8            reserved;
    tSirMacSSid   ssId;
} tSirSmeNewBssInfo, *tpSirSmeNewBssInfo;

typedef struct sSirSmeApNewCaps
{
    tANI_U16           capabilityInfo;
    tSirMacAddr   bssId;
    tANI_U8            channelId;
    tANI_U8            reserved[3];
    tSirMacSSid   ssId;
} tSirSmeApNewCaps, *tpSirSmeApNewCaps;

/**
 * Table below indicates what information is passed for each of
 * the Wireless Media status change notifications:
 *
 * Status Change code           Status change info
 * ----------------------------------------------------------------------
 * eSIR_SME_DEAUTH_FROM_PEER        Reason code received in DEAUTH frame
 * eSIR_SME_DISASSOC_FROM_PEER      Reason code received in DISASSOC frame
 * eSIR_SME_LOST_LINK_WITH_PEER     None
 * eSIR_SME_CHANNEL_SWITCH          New channel number
 * eSIR_SME_JOINED_NEW_BSS          BSSID, SSID and channel number
 * eSIR_SME_LEAVING_BSS             None
 * eSIR_SME_IBSS_ACTIVE             Indicates that another STA joined
 *                                  IBSS apart from this STA that
 *                                  started IBSS
 * eSIR_SME_IBSS_INACTIVE           Indicates that only this STA is left
 *                                  in IBSS
 * eSIR_SME_RADAR_DETECTED          Indicates that radar is detected
 * eSIR_SME_IBSS_NEW_PEER           Indicates that a new peer is detected
 * eSIR_SME_AP_CAPS_CHANGED         Indicates that capabilities of the AP
 *                                  that STA is currently associated with
 *                                  have changed.
 * eSIR_SME_BACKGROUND_SCAN_FAIL    Indicates background scan failure
 */

/// Definition for Wireless medium status change notification
typedef struct sSirSmeWmStatusChangeNtf
{
    tANI_U16                     messageType; // eWNI_SME_WM_STATUS_CHANGE_NTF
    tANI_U16                     length;
    tANI_U8                      sessionId;         // Session ID
    tSirSmeStatusChangeCode statusChangeCode;
    tSirMacAddr             bssId;             // Self BSSID
    union
    {
        tANI_U16                 deAuthReasonCode; // eSIR_SME_DEAUTH_FROM_PEER
        tANI_U16                 disassocReasonCode; // eSIR_SME_DISASSOC_FROM_PEER
        // none for eSIR_SME_LOST_LINK_WITH_PEER
        tANI_U8                  newChannelId;   // eSIR_SME_CHANNEL_SWITCH
        tSirSmeNewBssInfo   newBssInfo;     // eSIR_SME_JOINED_NEW_BSS
        // none for eSIR_SME_LEAVING_BSS
        // none for eSIR_SME_IBSS_ACTIVE
        // none for eSIR_SME_IBSS_INACTIVE
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
        tSirRadarInfo          radarInfo;         // eSIR_SME_RADAR_DETECTED
#endif
        tSirNewIbssPeerInfo     newIbssPeerInfo;  // eSIR_SME_IBSS_NEW_PEER
        tSirSmeApNewCaps        apNewCaps;        // eSIR_SME_AP_CAPS_CHANGED
        tSirBackgroundScanInfo  bkgndScanInfo;    // eSIR_SME_BACKGROUND_SCAN_FAIL
        tAniTitanCBNeighborInfo cbNeighbors;      // eSIR_SME_CB_LEGACY_BSS_FOUND_BY_STA
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
        tSirNeighborBssWdsInfo  neighborWdsInfo;  // eSIR_SME_CB_LEGACY_BSS_FOUND_BY_AP
#endif
    } statusChangeInfo;
} tSirSmeWmStatusChangeNtf, *tpSirSmeWmStatusChangeNtf;

/// Definition for Disassociation request
typedef
#ifdef WLAN_SOFTAP_FEATURE
__ani_attr_pre_packed
#endif
struct sSirSmeDisassocReq
{
    tANI_U16            messageType; // eWNI_SME_DISASSOC_REQ
    tANI_U16            length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirMacAddr         bssId;             // Peer BSSID
    tSirMacAddr peerMacAddr;
    tANI_U16         reasonCode;
    tANI_U8          doNotSendOverTheAir;  //This flag tells LIM whether to send the disassoc OTA or not
                                           //This will be set in while handing off from one AP to other
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16         aid;
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED)
    tANI_U16         seqNum;
#endif
#endif
}
#ifdef WLAN_SOFTAP_FEATURE
__ani_attr_packed
#endif
tSirSmeDisassocReq, *tpSirSmeDisassocReq;

/// Definition for Tkip countermeasures request
#ifdef WLAN_SOFTAP_FEATURE
typedef __ani_attr_pre_packed struct sSirSmeTkipCntrMeasReq
{
    tANI_U16            messageType;    // eWNI_SME_DISASSOC_REQ
    tANI_U16            length;
    tANI_U8             sessionId;      // Session ID
    tANI_U16            transactionId;  // Transaction ID for cmd
    tSirMacAddr         bssId;          // Peer BSSID
    tANI_BOOLEAN        bEnable;        // Start/stop countermeasures
} __ani_attr_packed tSirSmeTkipCntrMeasReq, *tpSirSmeTkipCntrMeasReq;
#endif

typedef struct sAni64BitCounters
{
    tANI_U32 Hi;
    tANI_U32 Lo;
}tAni64BitCounters, *tpAni64BitCounters;

typedef struct sAniSecurityStat
{
    tAni64BitCounters txBlks;
    tAni64BitCounters rxBlks;
    tAni64BitCounters formatErrorCnt;
    tAni64BitCounters decryptErr;
    tAni64BitCounters protExclCnt;
    tAni64BitCounters unDecryptableCnt;
    tAni64BitCounters decryptOkCnt;

}tAniSecurityStat, *tpAniSecurityStat;

typedef struct sAniTxRxCounters
{
    tANI_U32 txFrames; // Incremented for every packet tx
    tANI_U32 rxFrames;    
    tANI_U32 nRcvBytes;
    tANI_U32 nXmitBytes;
}tAniTxRxCounters, *tpAniTxRxCounters;

typedef struct sAniTxRxStats
{
    tAni64BitCounters txFrames;
    tAni64BitCounters rxFrames;
    tAni64BitCounters nRcvBytes;
    tAni64BitCounters nXmitBytes;

}tAniTxRxStats,*tpAniTxRxStats;

typedef struct sAniSecStats
{
    tAniSecurityStat aes;
    tAni64BitCounters aesReplays;
    tAniSecurityStat tkip;
    tAni64BitCounters tkipReplays;
    tAni64BitCounters tkipMicError;

    tAniSecurityStat wep;
#if defined(FEATURE_WLAN_WAPI) && !defined(LIBRA_WAPI_SUPPORT)
    tAniSecurityStat wpi;
    tAni64BitCounters wpiReplays;
    tAni64BitCounters wpiMicError;
#endif
}tAniSecStats, *tpAniSecStats;    

#define SIR_MAX_RX_CHAINS 3

typedef struct sAniStaStatStruct
{
    /* following statistic elements till expandPktRxCntLo are not filled with valid data.
     * These are kept as it is, since WSM is using this structure.
     * These elements can be removed whenever WSM is updated.
     * Phystats is used to hold phystats from BD.
     */
    tANI_U32 sentAesBlksUcastHi;
    tANI_U32 sentAesBlksUcastLo;
    tANI_U32 recvAesBlksUcastHi;
    tANI_U32 recvAesBlksUcastLo;
    tANI_U32 aesFormatErrorUcastCnts;
    tANI_U32 aesReplaysUcast;
    tANI_U32 aesDecryptErrUcast;
    tANI_U32 singleRetryPkts;
    tANI_U32 failedTxPkts;
    tANI_U32 ackTimeouts;
    tANI_U32 multiRetryPkts;
    tANI_U32 fragTxCntsHi;
    tANI_U32 fragTxCntsLo;
    tANI_U32 transmittedPktsHi;
    tANI_U32 transmittedPktsLo;
    tANI_U32 phyStatHi; //These are used to fill in the phystats.
    tANI_U32 phyStatLo; //This is only for private use.

    tANI_U32 uplinkRssi;
    tANI_U32 uplinkSinr;
    tANI_U32 uplinkRate;
    tANI_U32 downlinkRssi;
    tANI_U32 downlinkSinr;
    tANI_U32 downlinkRate;
    tANI_U32 nRcvBytes;
    tANI_U32 nXmitBytes;

    // titan 3c stats
    tANI_U32 chunksTxCntHi;          // Number of Chunks Transmitted
    tANI_U32 chunksTxCntLo;
    tANI_U32 compPktRxCntHi;         // Number of Packets Received that were actually compressed
    tANI_U32 compPktRxCntLo;
    tANI_U32 expanPktRxCntHi;        // Number of Packets Received that got expanded
    tANI_U32 expanPktRxCntLo;


    /* Following elements are valid and filled in correctly. They have valid values.
     */

    //Unicast frames and bytes.
    tAniTxRxStats ucStats;

    //Broadcast frames and bytes.
    tAniTxRxStats bcStats;

    //Multicast frames and bytes.
    tAniTxRxStats mcStats;

    tANI_U32      currentTxRate; 
    tANI_U32      currentRxRate; //Rate in 100Kbps

    tANI_U32      maxTxRate;
    tANI_U32      maxRxRate;

    tANI_S8       rssi[SIR_MAX_RX_CHAINS]; 


    tAniSecStats   securityStats;

    tANI_U8       currentRxRateIdx; //This the softmac rate Index.
    tANI_U8       currentTxRateIdx;

} tAniStaStatStruct, *tpAniStaStatStruct;

//Statistics that are not maintained per stations.
typedef struct sAniGlobalStatStruct
{
  tAni64BitCounters txError;
  tAni64BitCounters rxError;
  tAni64BitCounters rxDropNoBuffer;
  tAni64BitCounters rxDropDup;
  tAni64BitCounters rxCRCError;

  tAni64BitCounters singleRetryPkts;
  tAni64BitCounters failedTxPkts;
  tAni64BitCounters ackTimeouts;
  tAni64BitCounters multiRetryPkts;
  tAni64BitCounters fragTxCnts;
  tAni64BitCounters fragRxCnts;

  tAni64BitCounters txRTSSuccess;
  tAni64BitCounters txCTSSuccess;
  tAni64BitCounters rxRTSSuccess;
  tAni64BitCounters rxCTSSuccess;

  tAniSecStats      securityStats;

  tAniTxRxStats     mcStats;
  tAniTxRxStats     bcStats;
    
}tAniGlobalStatStruct,*tpAniGlobalStatStruct;

typedef enum sPacketType
{
    ePACKET_TYPE_UNKNOWN,
    ePACKET_TYPE_11A,
    ePACKET_TYPE_11G,
    ePACKET_TYPE_11B,
    ePACKET_TYPE_11N

}tPacketType, *tpPacketType;

typedef struct sAniStatSummaryStruct
{
    tAniTxRxStats uc; //Unicast counters.
    tAniTxRxStats bc; //Broadcast counters.
    tAniTxRxStats mc; //Multicast counters.
    tAni64BitCounters txError;
    tAni64BitCounters rxError;
    tANI_S8     rssi[SIR_MAX_RX_CHAINS]; //For each chain.
    tANI_U32    rxRate; // Rx rate of the last received packet.
    tANI_U32    txRate;
    tANI_U16    rxMCSId; //MCS index is valid only when packet type is ePACKET_TYPE_11N
    tANI_U16    txMCSId;
    tPacketType rxPacketType;
    tPacketType txPacketType;
    tSirMacAddr macAddr; //Mac Address of the station from which above RSSI and rate is from.
}tAniStatSummaryStruct,*tpAniStatSummaryStruct;

#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA)
//structure for stats that may be reset, like the ones in sta descriptor
//The stats are saved into here before reset. It should be tANI_U32 aligned.
typedef struct _sPermStaStats
{
    //tANI_U32 sentAesBlksUcastHi;
    //tANI_U32 sentAesBlksUcastLo;
    //tANI_U32 recvAesBlksUcastHi;
    //tANI_U32 recvAesBlksUcastLo;
    tANI_U32 aesFormatErrorUcastCnts;
    tANI_U32 aesReplaysUcast;
    tANI_U32 aesDecryptErrUcast;
    tANI_U32 singleRetryPkts;
    tANI_U32 failedTxPkts;
    tANI_U32 ackTimeouts;
    tANI_U32 multiRetryPkts;
    tANI_U32 fragTxCntsHi;
    tANI_U32 fragTxCntsLo;
    tANI_U32 transmittedPktsHi;
    tANI_U32 transmittedPktsLo;

    // titan 3c stats
    tANI_U32 chunksTxCntHi;          // Number of Chunks Transmitted
    tANI_U32 chunksTxCntLo;
    tANI_U32 compPktRxCntHi;         // Number of Packets Received that were actually compressed
    tANI_U32 compPktRxCntLo;
    tANI_U32 expanPktRxCntHi;        // Number of Packets Received that got expanded
    tANI_U32 expanPktRxCntLo;
}tPermanentStaStats;

#endif//#if (WNI_POLARIS_FW_PRODUCT == WLAN_STA)



/// Definition for Disassociation response
typedef struct sSirSmeDisassocRsp
{
    tANI_U16           messageType; // eWNI_SME_DISASSOC_RSP
    tANI_U16           length;
    tANI_U8            sessionId;         // Session ID
    tANI_U16           transactionId;     // Transaction ID for cmd
    tSirResultCodes    statusCode;
    tSirMacAddr        peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16           aid;
#endif
    tAniStaStatStruct  perStaStats; // STA stats
#ifdef WLAN_SOFTAP_FEATURE
    tANI_U16           staId;
#endif     
}
#ifdef WLAN_SOFTAP_FEATURE
__ani_attr_packed
#endif
 tSirSmeDisassocRsp, *tpSirSmeDisassocRsp;

/// Definition for Disassociation indication from peer
typedef struct sSirSmeDisassocInd
{
    tANI_U16            messageType; // eWNI_SME_DISASSOC_IND
    tANI_U16            length;
    tANI_U8             sessionId;  // Session Identifier
    tANI_U16            transactionId;   // Transaction Identifier with PE
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;            
    tSirMacAddr         peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16            aid;
#endif
    tAniStaStatStruct  perStaStats; // STA stats
#ifdef WLAN_SOFTAP_FEATURE
    tANI_U16            staId;
#endif
    tANI_U32            reasonCode;
} tSirSmeDisassocInd, *tpSirSmeDisassocInd;

/// Definition for Disassociation confirm
/// MAC --->
typedef struct sSirSmeDisassocCnf
{
    tANI_U16            messageType; // eWNI_SME_DISASSOC_CNF
    tANI_U16            length;
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;            
    tSirMacAddr         peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16            aid;
#endif
} tSirSmeDisassocCnf, *tpSirSmeDisassocCnf;

/// Definition for Deauthetication request
typedef struct sSirSmeDeauthReq
{
    tANI_U16            messageType;   // eWNI_SME_DEAUTH_REQ
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tSirMacAddr         bssId;         // AP BSSID
    tSirMacAddr         peerMacAddr;
    tANI_U16            reasonCode;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16         aid;
#endif
} tSirSmeDeauthReq, *tpSirSmeDeauthReq;

/// Definition for Deauthetication response
typedef struct sSirSmeDeauthRsp
{
    tANI_U16                messageType; // eWNI_SME_DEAUTH_RSP
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirMacAddr        peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirSmeDeauthRsp, *tpSirSmeDeauthRsp;

/// Definition for Deauthetication indication from peer
typedef struct sSirSmeDeauthInd
{
    tANI_U16            messageType; // eWNI_SME_DEAUTH_IND
    tANI_U16            length;
    tANI_U8            sessionId;       //Added for BT-AMP
    tANI_U16            transactionId;  //Added for BT-AMP
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;// AP BSSID
    tSirMacAddr         peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16            aid;
#endif

#ifdef WLAN_SOFTAP_FEATURE
    tANI_U16            staId;
#endif
    tANI_U32            reasonCode;
} tSirSmeDeauthInd, *tpSirSmeDeauthInd;

/// Definition for Deauthentication confirm
/// MAC --->
typedef struct sSirSmeDeauthCnf
{
    tANI_U16                messageType; // eWNI_SME_DEAUTH_CNF
    tANI_U16                length;
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;             // AP BSSID
    tSirMacAddr        peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirSmeDeauthCnf, *tpSirSmeDeauthCnf;

/// Definition for stop BSS request message
typedef struct sSirSmeStopBssReq
{
    tANI_U16                messageType;    // eWNI_SME_STOP_BSS_REQ
    tANI_U16                length;
    tANI_U8             sessionId;      //Session ID
    tANI_U16            transactionId;  //tranSaction ID for cmd
    tSirResultCodes         reasonCode;
    tSirMacAddr             bssId;          //Self BSSID
} tSirSmeStopBssReq, *tpSirSmeStopBssReq;

/// Definition for stop BSS response message
typedef struct sSirSmeStopBssRsp
{
    tANI_U16             messageType; // eWNI_SME_STOP_BSS_RSP
    tANI_U16             length;
    tSirResultCodes statusCode;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeStopBssRsp, *tpSirSmeStopBssRsp;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
/// Definition for Channel Select request
/// ---> MAC
typedef struct sSirSmeSelectChannelReq
{
    tANI_U16                messageType; // eWNI_SME_SELECT_CHL_REQ
    tANI_U16                length;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tANI_U8                 channelId;
} tSirSmeSelectChannelReq, *tpSirSmeSelectChannelReq;

/// Definition for Channel Select response
/// MAC --->
typedef struct sSirSmeSelectChannelRsp
{
    tANI_U16                messageType; // eWNI_SME_SELECT_CHL_RSP
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeSelectChannelRsp, *tpSirSmeSelectChannelRsp;

/// Definition for Channel Switch request
/// ---> MAC
typedef struct sSirSmeSwitchChannelReq
{
    // eWNI_SME_SWITCH_CHL_REQ,
    // eWNI_SME_SWITCH_CHL_CB_PRIMARY_REQ,
    // eWNI_SME_SWITCH_CHL_CB_SECONDARY_REQ
    tANI_U16                messageType;

    tANI_U16                length;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tANI_U8                 channelId;
    //
    // The cbMode field is applicable to TITAN only.
    // This indicates as to how the CB secondary
    // channel will be used (if at all).
    //
    // In a non-CB environment, with 11H enabled,
    // this field will be ignored
    //
    ePhyChanBondState    cbMode;

    // dtimFactor indicates the number of DTIM
    // Beacon before LIM switches channel
    tANI_U32                dtimFactor;
} tSirSmeSwitchChannelReq, *tpSirSmeSwitchChannelReq;

/// Definition for Channel Switch response
/// MAC --->
typedef struct sSirSmeSwitchChannelRsp
{
    tANI_U16                messageType; // eWNI_SME_SWITCH_CHL_RSP
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeSwitchChannelRsp, *tpSirSmeSwitchChannelRsp;

#endif


/// Definition for Channel Switch indication for station
/// MAC --->
typedef struct sSirSmeSwitchChannelInd
{
    tANI_U16                messageType; // eWNI_SME_SWITCH_CHL_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;
    tANI_U16    newChannelId;
    tSirMacAddr        bssId;      // BSSID
} tSirSmeSwitchChannelInd, *tpSirSmeSwitchChannelInd;

/// Definition for ULA complete indication message
typedef struct sirUlaCompleteInd
{
    tANI_U16                messageType; // eWNI_ULA_COMPLETE_IND
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tSirMacAddr        peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirUlaCompleteInd, *tpSirUlaCompleteInd;

/// Definition for ULA complete confirmation message
typedef struct sirUlaCompleteCnf
{
    tANI_U16                messageType; // eWNI_ULA_COMPLETE_CNF
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tSirMacAddr        peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirUlaCompleteCnf, *tpSirUlaCompleteCnf;

/// Definition for Neighbor BSS indication
/// MAC --->
/// MAC reports this each time a new I/BSS is detected
typedef struct sSirSmeNeighborBssInd
{
    tANI_U16                    messageType; // eWNI_SME_NEIGHBOR_BSS_IND
    tANI_U16                    length;
    tANI_U8                     sessionId;
#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
    tSirNeighborBssInfo    neighborInfo;
    tSirWdsInfo            wdsInfo;
#else
    tSirBssDescription     bssDescription[1];
#endif
} tSirSmeNeighborBssInd, *tpSirSmeNeighborBssInd;

/// Definition for MIC failure indication
/// MAC --->
/// MAC reports this each time a MIC failure occures on Rx TKIP packet
typedef struct sSirSmeMicFailureInd
{
    tANI_U16                    messageType; // eWNI_SME_MIC_FAILURE_IND
    tANI_U16                    length;
    tANI_U8                     sessionId;
    tSirMacAddr         bssId;             // BSSID
    tSirMicFailureInfo     info;
} tSirSmeMicFailureInd, *tpSirSmeMicFailureInd;


/// Definition for Set Context request
/// ---> MAC
typedef struct sSirSmeSetContextReq
{
    tANI_U16           messageType; // eWNI_SME_SET_CONTEXT_REQ
    tANI_U16          length;
    tANI_U8            sessionId;  //Session ID
    tANI_U16           transactionId; //Transaction ID for cmd
    tSirMacAddr        peerMacAddr;
    tSirMacAddr        bssId;      // BSSID
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
    // TBD Following QOS fields to be uncommented
    //tAniBool           qosInfoPresent;
    //tSirQos            qos;
    tSirKeyMaterial    keyMaterial;
} tSirSmeSetContextReq, *tpSirSmeSetContextReq;

/// Definition for Set Context response
/// MAC --->
typedef struct sSirSmeSetContextRsp
{
    tANI_U16                messageType; // eWNI_SME_SET_CONTEXT_RSP
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirMacAddr             peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirSmeSetContextRsp, *tpSirSmeSetContextRsp;

/// Definition for Remove Key Context request
/// ---> MAC
typedef struct sSirSmeRemoveKeyReq
{
    tANI_U16                messageType;    // eWNI_SME_REMOVE_KEY_REQ
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirMacAddr         bssId;             // BSSID
    tSirMacAddr             peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
    tANI_U8    edType;
    tANI_U8    wepType;
    tANI_U8    keyId;
    tANI_BOOLEAN unicast;
} tSirSmeRemoveKeyReq, *tpSirSmeRemoveKeyReq;

/// Definition for Remove Key Context response
/// MAC --->
typedef struct sSirSmeRemoveKeyRsp
{
    tANI_U16                messageType; // eWNI_SME_REMOVE_KEY_RSP
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirMacAddr             peerMacAddr;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirSmeRemoveKeyRsp, *tpSirSmeRemoveKeyRsp;

/// Definition for Set Power request
/// ---> MAC
typedef struct sSirSmeSetPowerReq
{
    tANI_U16                messageType; // eWNI_SME_SET_POWER_REQ
    tANI_U16                length;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tANI_S8                 powerLevel;
} tSirSmeSetPowerReq, *tpSirSmeSetPowerReq;

/// Definition for Set Power response
/// MAC --->
typedef struct sSirSmeSetPowerRsp
{
    tANI_U16                messageType; // eWNI_SME_SET_POWER_RSP
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeSetPowerRsp, *tpSirSmeSetPowerRsp;

#if (WNI_POLARIS_FW_PACKAGE == ADVANCED) && (WNI_POLARIS_FW_PRODUCT == AP)
/// Definition for Client Side Load Balancing request
/// ---> MAC
typedef struct sSirSmeSetClientLoadBalanceReq
{
    tANI_U16                messageType; // eWNI_SME_CLIENT_LOAD_BALANCE_REQ
    tANI_U16                length;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirMacAddr             alternateBssId;
    tANI_U8                 alternateChannelId;
    tANI_U8                 numberStas;
} tSirSmeClientLoadBalanceReq, *tpSirSmeClientLoadBalanceReq;

/// Definition for Client Side Load Balancing response
/// MAC --->
typedef struct sSirSmeSetClientLoadBalanceRsp
{
    tANI_U16                messageType; // eWNI_SME_CLIENT_LOAD_BALANCE_RSP
    tANI_U16                length;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes         statusCode;
} tSirSmeClientLoadBalanceRsp, *tpSirSmeClientLoadBalanceRsp;
#endif

/// Definition for Link Test Start response
/// MAC --->
typedef struct sSirSmeLinkTestStartRsp
{
    tANI_U16                messageType; // eWNI_SME_LINK_TEST_START_RSP
    tANI_U16                length;
    tSirMacAddr        peerMacAddr;
    tSirResultCodes    statusCode;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirSmeLinkTestStartRsp, *tpSirSmeLinkTestStartRsp;

/// Definition for Link Test Stop response
/// WSM ---> MAC
typedef struct sSirSmeLinkTestStopRsp
{
    tANI_U16                messageType; // eWNI_SME_LINK_TEST_STOP_RSP
    tANI_U16                length;
    tSirMacAddr        peerMacAddr;
    tSirResultCodes    statusCode;
#if (WNI_POLARIS_FW_PRODUCT == AP)
    tANI_U16                aid;
#endif
} tSirSmeLinkTestStopRsp, *tpSirSmeLinkTestStopRsp;

/// Definition for kick starting DFS measurements
typedef struct sSirSmeDFSreq
{
    tANI_U16             messageType; // eWNI_SME_DFS_REQ
    tANI_U16             length;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeDFSrequest, *tpSirSmeDFSrequest;

/// Definition for response message to previously
/// issued DFS request
typedef struct sSirSmeDFSrsp
{
    tANI_U16             messageType; // eWNI_SME_DFS_RSP
    tANI_U16             length;
    tSirResultCodes statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tANI_U32             dfsReport[1];
} tSirSmeDFSrsp, *tpSirSmeDFSrsp;

/// Statistic definitions
