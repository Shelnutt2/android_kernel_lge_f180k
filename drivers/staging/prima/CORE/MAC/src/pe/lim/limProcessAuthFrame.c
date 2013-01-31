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
 *
 * Airgo Networks, Inc proprietary. All rights reserved.
 * This file limProcessAuthFrame.cc contains the code
 * for processing received Authentication Frame.
 * Author:        Chandra Modumudi
 * Date:          03/11/02
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 * 05/12/2010     js             To support Shared key authentication at AP side
 *
 */

#include "wniApi.h"
#ifdef FEATURE_WLAN_NON_INTEGRATED_SOC
#include "halDataStruct.h"
#endif
#if (WNI_POLARIS_FW_PRODUCT == AP)
#include "wniCfgAp.h"
#else
#include "wniCfgSta.h"
#endif
#include "aniGlobal.h"
#include "cfgApi.h"

#include "utilsApi.h"
#include "limUtils.h"
#include "limAssocUtils.h"
#include "limSecurityUtils.h"
#include "limSerDesUtils.h"
#ifdef WLAN_FEATURE_VOWIFI_11R
#include "limFT.h"
#endif
#include "vos_utils.h"


/**
 * isAuthValid
 *
 *FUNCTION:
 * This function is called by limProcessAuthFrame() upon Authentication
 * frame reception.
 *
 *LOGIC:
 * This function is used to test validity of auth frame:
 * - AUTH1 and AUTH3 must be received in AP mode
 * - AUTH2 and AUTH4 must be received in STA mode
 * - AUTH3 and AUTH4 must have challenge text IE, that is,'type' field has been set to
 *                 SIR_MAC_CHALLENGE_TEXT_EID by parser
 * -
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  *auth - Pointer to extracted auth frame body
 *
 * @return 0 or 1 (Valid)
 */


static inline unsigned int isAuthValid(tpAniSirGlobal pMac, tpSirMacAuthFrameBody auth,tpPESession sessionEntry) {
    unsigned int valid;
    valid=1;

    if (  ((auth->authTransactionSeqNumber==SIR_MAC_AUTH_FRAME_1)||
           (auth->authTransactionSeqNumber==SIR_MAC_AUTH_FRAME_3)) &&
          ((sessionEntry->limSystemRole == eLIM_STA_ROLE)||(sessionEntry->limSystemRole == eLIM_BT_AMP_STA_ROLE)))
        valid=0;

    if ( ((auth->authTransactionSeqNumber==SIR_MAC_AUTH_FRAME_2)||(auth->authTransactionSeqNumber==SIR_MAC_AUTH_FRAME_4))&&
         ((sessionEntry->limSystemRole == eLIM_AP_ROLE)||(sessionEntry->limSystemRole == eLIM_BT_AMP_AP_ROLE)))
        valid=0;

    if ( ((auth->authTransactionSeqNumber==SIR_MAC_AUTH_FRAME_3)||(auth->authTransactionSeqNumber==SIR_MAC_AUTH_FRAME_4))&&
         (auth->type!=SIR_MAC_CHALLENGE_TEXT_EID)&&(auth->authAlgoNumber != eSIR_SHARED_KEY))
        valid=0;

    return valid;
}


/**
 * limProcessAuthFrame
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue() upon Authentication
 * frame reception.
 *
 *LOGIC:
 * This function processes received Authentication frame and responds
 * with either next Authentication frame in sequence to peer MAC entity
 * or LIM_MLM_AUTH_IND on AP or LIM_MLM_AUTH_CNF on STA.
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 * 1. Authentication failures are reported to SME with same status code
 *    received from the peer MAC entity.
 * 2. Authentication frame2/4 received with alogirthm number other than
 *    one requested in frame1/3 are logged with an error and auth confirm
 *    will be sent to SME only after auth failure timeout.
 * 3. Inconsistency in the spec:
 *    On receiving Auth frame2, specs says that if WEP key mapping key
 *    or default key is NULL, Auth frame3 with a status code 15 (challenge
 *    failure to be returned to peer entity. However, section 7.2.3.10,
 *    table 14 says that status code field is 'reserved' for frame3 !
 *    In the current implementation, Auth frame3 is returned with status
 *    code 15 overriding section 7.2.3.10.
 * 4. If number pre-authentications reach configrable max limit,
 *    Authentication frame with 'unspecified failure' status code is
 *    returned to requesting entity.
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  *pRxPacketInfo - A pointer to Rx packet info structure
 * @return None
 */

void
limProcessAuthFrame(tpAniSirGlobal pMac, tANI_U8 *pRxPacketInfo, tpPESession psessionEntry)
{
    tANI_U8                 *pBody, keyId, cfgPrivacyOptImp,
                            defaultKey[SIR_MAC_KEY_LENGTH],
                            encrAuthFrame[LIM_ENCR_AUTH_BODY_LEN],
                            plainBody[256];
    tANI_U16                frameLen;
    //tANI_U32                authRspTimeout, maxNumPreAuth, val;
    tANI_U32                maxNumPreAuth, val;
    tSirMacAuthFrameBody    *pRxAuthFrameBody, rxAuthFrame, authFrame;
    tpSirMacMgmtHdr         pHdr;
    tCfgWepKeyEntry         *pKeyMapEntry = NULL;
    struct tLimPreAuthNode  *pAuthNode;
    tLimMlmAuthInd          mlmAuthInd;
    tANI_U8                 decryptResult;
    tANI_U8                 *pChallenge;
    tANI_U32                key_length=8;
    tANI_U8                 challengeTextArray[SIR_MAC_AUTH_CHALLENGE_LENGTH];
#ifdef WLAN_SOFTAP_FEATURE
    tpDphHashNode           pStaDs = NULL;
    tANI_U16                assocId = 0;
#endif
    /* Added For BT -AMP support */
    // Get pointer to Authentication frame header and body
 

    pHdr = WDA_GET_RX_MAC_HEADER(pRxPacketInfo);
    frameLen = WDA_GET_RX_PAYLOAD_LEN(pRxPacketInfo);
    

    if (!frameLen)
    {
        // Log error
        limLog(pMac, LOGE,
               FL("received Authentication frame with no body from "));
        limPrintMacAddr(pMac, pHdr->sa, LOGE);

        return;
    }

    if (limIsGroupAddr(pHdr->sa))
    {
        // Received Auth frame from a BC/MC address
        // Log error and ignore it
        PELOG1(limLog(pMac, LOG1,
               FL("received Auth frame from a BC/MC address - "));)
       PELOG1( limPrintMacAddr(pMac, pHdr->sa, LOG1);)

        return;
    }

    pBody = WDA_GET_RX_MPDU_DATA(pRxPacketInfo);

    //PELOG3(sirDumpBuf(pMac, SIR_LIM_MODULE_ID, LOG3, (tANI_U8*)pBd, ((tpHalBufDesc) pBd)->mpduDataOffset + frameLen);)


   
    /// Determine if WEP bit is set in the FC or received MAC header
    if (pHdr->fc.wep)
    {
        /**
         * WEP bit is set in FC of MAC header.
         */

#ifdef WLAN_SOFTAP_FEATURE
        // If TKIP counter measures enabled issue Deauth frame to station
        if ((psessionEntry->bTkipCntrMeasActive) && (psessionEntry->limSystemRole == eLIM_AP_ROLE))
        {
            PELOGE( limLog(pMac, LOGE,
               FL("Tkip counter measures Enabled, sending Deauth frame to")); )
            limPrintMacAddr(pMac, pHdr->sa, LOGE);

            limSendDeauthMgmtFrame( pMac, eSIR_MAC_MIC_FAILURE_REASON,
                                    pHdr->sa, psessionEntry );
            return;
        }
#endif

        // Extract key ID from IV (most 2 bits of 4th byte of IV)

        keyId = (*(pBody + 3)) >> 6;

        /**
         * On STA in infrastructure BSS, Authentication frames received
         * with WEP bit set in the FC must be rejected with challenge
         * failure status code (wierd thing in the spec - this should have
         * been rejected with unspecified failure or unexpected assertion
         * of wep bit (this status code does not exist though) or
         * Out-of-sequence-Authentication-Frame status code.
         */

        if (psessionEntry->limSystemRole == eLIM_STA_ROLE || psessionEntry->limSystemRole == eLIM_BT_AMP_STA_ROLE)
        {
            authFrame.authAlgoNumber = eSIR_SHARED_KEY;
            authFrame.authTransactionSeqNumber = SIR_MAC_AUTH_FRAME_4;
            authFrame.authStatusCode = eSIR_MAC_CHALLENGE_FAILURE_STATUS;

            limSendAuthMgmtFrame(pMac, &authFrame,
                                 pHdr->sa,
                                 LIM_NO_WEP_IN_FC,psessionEntry);
            // Log error
            PELOGE(limLog(pMac, LOGE,
                   FL("received Authentication frame with wep bit set on role=%d "MAC_ADDRESS_STR),
                   psessionEntry->limSystemRole, MAC_ADDR_ARRAY(pHdr->sa) );)

            return;
        }

        if (frameLen < LIM_ENCR_AUTH_BODY_LEN)
        {
            // Log error
            limLog(pMac, LOGE,
                   FL("Not enough size [%d] to decrypt received Auth frame"),
                   frameLen);
            limPrintMacAddr(pMac, pHdr->sa, LOGE);

            return;
        }
#ifdef WLAN_SOFTAP_FEATURE
        if(psessionEntry->limSystemRole == eLIM_AP_ROLE)
        {
            val = psessionEntry->privacy; 
        } 
        else 
#endif
        // Accept Authentication frame only if Privacy is implemented
        if (wlan_cfgGetInt(pMac, WNI_CFG_PRIVACY_ENABLED,
                      &val) != eSIR_SUCCESS)
        {
            /**
             * Could not get Privacy option
             * from CFG. Log error.
             */
            limLog(pMac, LOGP, FL("could not retrieve Privacy option\n"));
        }

        cfgPrivacyOptImp = (tANI_U8)val;
        if (cfgPrivacyOptImp)
        {
            /**
             * Privacy option is implemented.
             * Check if the received frame is Authentication
             * frame3 and there is a context for requesting STA.
             * If not, reject with unspecified failure status code
             */
            pAuthNode = limSearchPreAuthList(pMac, pHdr->sa);

            if (pAuthNode == NULL)
            {
                /**
                 * No 'pre-auth' context exists for this STA that sent
                 * an Authentication frame with FC bit set.
                 * Send Auth frame4 with 'out of sequence' status code.
                 */
                authFrame.authAlgoNumber = eSIR_SHARED_KEY;
                authFrame.authTransactionSeqNumber =
                SIR_MAC_AUTH_FRAME_4;
                authFrame.authStatusCode =
                eSIR_MAC_AUTH_FRAME_OUT_OF_SEQ_STATUS;

                limSendAuthMgmtFrame(pMac, &authFrame,
                                     pHdr->sa,
                                     LIM_NO_WEP_IN_FC,psessionEntry);

                // Log error
                PELOGE(limLog(pMac, LOGE,
                       FL("received Authentication frame from peer that has "
                       "no preauth context with WEP bit set "MAC_ADDRESS_STR),
                       MAC_ADDR_ARRAY(pHdr->sa));)

                return;
            }
            else
            {
                /// Change the auth-response timeout
                limDeactivateAndChangePerStaIdTimer(pMac,
                                                    eLIM_AUTH_RSP_TIMER,
                                                    pAuthNode->authNodeIdx);

                /// 'Pre-auth' status exists for STA
                if ((pAuthNode->mlmState !=
                     eLIM_MLM_WT_AUTH_FRAME3_STATE) &&
                    (pAuthNode->mlmState !=
                     eLIM_MLM_AUTH_RSP_TIMEOUT_STATE))
                {
                    /**
                     * Should not have received Authentication frame
                     * with WEP bit set in FC in other states.
                     * Reject by sending Authenticaton frame with
                     * out of sequence Auth frame status code.
                     */

                    authFrame.authAlgoNumber = eSIR_SHARED_KEY;
                    authFrame.authTransactionSeqNumber =
                    SIR_MAC_AUTH_FRAME_4;
                    authFrame.authStatusCode =
                    eSIR_MAC_AUTH_FRAME_OUT_OF_SEQ_STATUS;

                    limSendAuthMgmtFrame(pMac, &authFrame,
                                         pHdr->sa,
                                         LIM_NO_WEP_IN_FC,psessionEntry);

                    // Log error
                    PELOGE(limLog(pMac, LOGE,
                           FL("received Authentication frame from peer that is in state %d "
                           MAC_ADDRESS_STR), pAuthNode->mlmState, MAC_ADDR_ARRAY(pHdr->sa));)

                    return;
                }
            }

            /**
             * Check if there exists a key mappping key
             * for the STA that sent Authentication frame
             */
            pKeyMapEntry = limLookUpKeyMappings(pHdr->sa);

            if (pKeyMapEntry)
            {
                if (!pKeyMapEntry->wepOn)
                {
                    /**
                     * Key Mapping entry has null key.
                     * Send Authentication frame
                     * with challenge failure status code
                     */
                    authFrame.authAlgoNumber = eSIR_SHARED_KEY;
                    authFrame.authTransactionSeqNumber =
                    SIR_MAC_AUTH_FRAME_4;
                    authFrame.authStatusCode =
                    eSIR_MAC_CHALLENGE_FAILURE_STATUS;

                    limSendAuthMgmtFrame(pMac, &authFrame,
                                         pHdr->sa,
                                         LIM_NO_WEP_IN_FC,psessionEntry);

                    // Log error
                    PELOGE(limLog(pMac, LOGE,
                           FL("received Auth frame3 from peer that has NULL key map entry "
                           MAC_ADDRESS_STR),MAC_ADDR_ARRAY(pHdr->sa));)

                    return;
                } // if (!pKeyMapEntry->wepOn)
                else
                {
                    decryptResult = limDecryptAuthFrame(pMac, pKeyMapEntry->key,
                                                        pBody,
                                                        plainBody,
                                                        key_length,
                                                        (tANI_U16) (frameLen-SIR_MAC_WEP_IV_LENGTH));
                    if (decryptResult == LIM_DECRYPT_ICV_FAIL)
                    {
                        /// ICV failure
                        PELOGW(limLog(pMac, LOGW, FL("=====> decryptResult == LIM_DECRYPT_ICV_FAIL ..."));)
                        limDeletePreAuthNode(pMac,
                                             pHdr->sa);
                        authFrame.authAlgoNumber = eSIR_SHARED_KEY;
                        authFrame.authTransactionSeqNumber =
                        SIR_MAC_AUTH_FRAME_4;
                        authFrame.authStatusCode =
                        eSIR_MAC_CHALLENGE_FAILURE_STATUS;

                        limSendAuthMgmtFrame(
                                            pMac, &authFrame,
                                            pHdr->sa,
                                            LIM_NO_WEP_IN_FC,psessionEntry);

                        // Log error
                        PELOGE(limLog(pMac, LOGE,
                               FL("received Authentication frame from peer that failed decryption, Addr "
                               MAC_ADDRESS_STR), MAC_ADDR_ARRAY(pHdr->sa));)

                        return;
                    }

                    if ((sirConvertAuthFrame2Struct(pMac, plainBody, frameLen-8, &rxAuthFrame)!=eSIR_SUCCESS)||(!isAuthValid(pMac, &rxAuthFrame,psessionEntry)))
                        return;


                } // end if (pKeyMapEntry->key == NULL)
            } // if keyMappings has entry
            else
            {

                val = SIR_MAC_KEY_LENGTH;

#ifdef WLAN_SOFTAP_FEATURE  
                if(psessionEntry->limSystemRole == eLIM_AP_ROLE)
                {   
                    tpSirKeys pKey;
                    pKey =  &psessionEntry->WEPKeyMaterial[keyId].key[0];              
                    palCopyMemory( pMac->hHdd, defaultKey, pKey->key, pKey->keyLength);
                    val = pKey->keyLength;
                }                   
                else                              
#endif                                    
                if (wlan_cfgGetStr(pMac, (tANI_U16) (WNI_CFG_WEP_DEFAULT_KEY_1 + keyId),
                              defaultKey, &val) != eSIR_SUCCESS)
                {
                    /// Could not get Default key from CFG.
                    //Log error.
                    limLog(pMac, LOGP,
                           FL("could not retrieve Default key\n"));

                    /**
                     * Send Authentication frame
                     * with challenge failure status code
                     */

                    authFrame.authAlgoNumber = eSIR_SHARED_KEY;
                    authFrame.authTransactionSeqNumber =
                    SIR_MAC_AUTH_FRAME_4;
                    authFrame.authStatusCode =
                    eSIR_MAC_CHALLENGE_FAILURE_STATUS;

                    limSendAuthMgmtFrame(pMac, &authFrame,
                                         pHdr->sa,
                                         LIM_NO_WEP_IN_FC,psessionEntry);

                    return;
                }

                    key_length=val;

                    decryptResult = limDecryptAuthFrame(pMac, defaultKey,
                                                        pBody,
                                                        plainBody,
                                                        key_length,
                                                        (tANI_U16) (frameLen-SIR_MAC_WEP_IV_LENGTH));
                    if (decryptResult == LIM_DECRYPT_ICV_FAIL)
                    {
                        PELOGW(limLog(pMac, LOGW, FL("=====> decryptResult == LIM_DECRYPT_ICV_FAIL ...\n"));)
                        /// ICV failure
                        limDeletePreAuthNode(pMac,
                                             pHdr->sa);
                        authFrame.authAlgoNumber = eSIR_SHARED_KEY;
                        authFrame.authTransactionSeqNumber =
                        SIR_MAC_AUTH_FRAME_4;
                        authFrame.authStatusCode =
                        eSIR_MAC_CHALLENGE_FAILURE_STATUS;

                        limSendAuthMgmtFrame(
                                            pMac, &authFrame,
                                            pHdr->sa,
                                            LIM_NO_WEP_IN_FC,psessionEntry);

                        // Log error
                        PELOGE(limLog(pMac, LOGE,
                               FL("received Authentication frame from peer that failed decryption: "
                               MAC_ADDRESS_STR), MAC_ADDR_ARRAY(pHdr->sa));)

                        return;
                    }
                    if ((sirConvertAuthFrame2Struct(pMac, plainBody, frameLen-8, &rxAuthFrame)!=eSIR_SUCCESS)||(!isAuthValid(pMac, &rxAuthFrame,psessionEntry)))
                        return;

            } // End of check for Key Mapping/Default key presence
        }
        else
        {
            /**
             * Privacy option is not implemented.
             * So reject Authentication frame received with
             * WEP bit set by sending Authentication frame
             * with 'challenge failure' status code. This is
             * another strange thing in the spec. Status code
             * should have been 'unsupported algorithm' status code.
             */

            authFrame.authAlgoNumber = eSIR_SHARED_KEY;
            authFrame.authTransactionSeqNumber =
            SIR_MAC_AUTH_FRAME_4;
            authFrame.authStatusCode =
            eSIR_MAC_CHALLENGE_FAILURE_STATUS;

            limSendAuthMgmtFrame(pMac, &authFrame,
                                 pHdr->sa,
                                 LIM_NO_WEP_IN_FC,psessionEntry);

            // Log error
            PELOGE(limLog(pMac, LOGE,
                   FL("received Authentication frame3 from peer that while privacy option is turned OFF "
                   MAC_ADDRESS_STR), MAC_ADDR_ARRAY(pHdr->sa));)

            return;
        } // else if (wlan_cfgGetInt(CFG_PRIVACY_OPTION_IMPLEMENTED))
    } // if (fc.wep)
    else
    {


        if ((sirConvertAuthFrame2Struct(pMac, pBody, frameLen, &rxAuthFrame)!=eSIR_SUCCESS)||(!isAuthValid(pMac, &rxAuthFrame,psessionEntry)))
            return;
    }


    pRxAuthFrameBody = &rxAuthFrame;

   PELOGW(limLog(pMac, LOGW,
           FL("Received Auth frame with type=%d seqnum=%d, status=%d (%d)\n"),
           (tANI_U32) pRxAuthFrameBody->authAlgoNumber,
           (tANI_U32) pRxAuthFrameBody->authTransactionSeqNumber,
           (tANI_U32) pRxAuthFrameBody->authStatusCode,(tANI_U32)pMac->lim.gLimNumPreAuthContexts);)

    switch (pRxAuthFrameBody->authTransactionSeqNumber)
    {
        case SIR_MAC_AUTH_FRAME_1:
            // AuthFrame 1

            /// Check if there exists pre-auth context for this STA
            pAuthNode = limSearchPreAuthList(pMac, pHdr->sa);
            if (pAuthNode)
            {
                /// Pre-auth context exists for the STA
                if (pHdr->fc.retry == 0)
                {
                    /**
                     * STA is initiating brand-new Authentication
                     * sequence after local Auth Response timeout.
                     * Or STA retrying to transmit First Auth frame due to packet drop OTA
                     * Delete Pre-auth node and fall through.
                     */
                    if(pAuthNode->fTimerStarted)
                    {
                        limDeactivateAndChangePerStaIdTimer(pMac,
                                                    eLIM_AUTH_RSP_TIMER,
                                                    pAuthNode->authNodeIdx);
                    }
                    PELOGE(limLog(pMac, LOGE, FL("STA is initiating brand-new Authentication ...\n"));)
                    limDeletePreAuthNode(pMac,
                                         pHdr->sa);
#ifdef WLAN_SOFTAP_FEATURE                    
                    /**
                     *  SAP Mode:Disassociate the station and 
                     *  delete its entry if we have its entry 
                     *  already and received "auth" from the 
                     *  same station.
                     */  

                    for (assocId = 0; assocId < psessionEntry->dph.dphHashTable.size; assocId++)// Softap dphHashTable.size = 8
                    {
                        pStaDs = dphGetHashEntry(pMac, assocId, &psessionEntry->dph.dphHashTable);

                        if (NULL == pStaDs)
                             continue;

                        if (pStaDs->valid)
                        {
                             if (palEqualMemory( pMac->hHdd,(tANI_U8 *) &pStaDs->staAddr,
                                      (tANI_U8 *) &(pHdr->sa), (tANI_U8) (sizeof(tSirMacAddr))) )
                                  break;
                        }
                    }

                    if (NULL != pStaDs)
                    {
                        PELOGE(limLog(pMac, LOGE, FL("lim Delete Station Context (staId: %d, assocId: %d) \n"),pStaDs->staIndex, assocId);)
                        limSendDeauthMgmtFrame(pMac,
                               eSIR_MAC_UNSPEC_FAILURE_REASON, (tANI_U8 *) pAuthNode->peerMacAddr,psessionEntry);
                        limTriggerSTAdeletion(pMac, pStaDs, psessionEntry);
                        return;
                    }
#endif
                }
                else
                {
                    /* 
                     * This can happen when first authentication frame is received
                     * but ACK lost at STA side, in this case 2nd auth frame is already 
                     * in transmission queue
                     * */
                    PELOGE(limLog(pMac, LOGE, FL("STA is initiating Authentication after ACK lost...\n"));)
                    return;
                }
            }
            if (wlan_cfgGetInt(pMac, WNI_CFG_MAX_NUM_PRE_AUTH,
                          (tANI_U32 *) &maxNumPreAuth) != eSIR_SUCCESS)
            {
                /**
                 * Could not get MaxNumPreAuth
                 * from CFG. Log error.
                 */
                limLog(pMac, LOGP,
                       FL("could not retrieve MaxNumPreAuth\n"));
            }
#ifdef ANI_AP_SDK_OPT
            if(maxNumPreAuth > SIR_SDK_OPT_MAX_NUM_PRE_AUTH)
                maxNumPreAuth = SIR_SDK_OPT_MAX_NUM_PRE_AUTH;
#endif // ANI_AP_SDK_OPT
            if (pMac->lim.gLimNumPreAuthContexts == maxNumPreAuth)
            {
                /**
                 * Maximum number of pre-auth contexts
                 * reached. Send Authentication frame
                 * with unspecified failure
                 */
                authFrame.authAlgoNumber =
                pRxAuthFrameBody->authAlgoNumber;
                authFrame.authTransactionSeqNumber =
                pRxAuthFrameBody->authTransactionSeqNumber + 1;
                authFrame.authStatusCode =
                eSIR_MAC_UNSPEC_FAILURE_STATUS;

                limSendAuthMgmtFrame(pMac, &authFrame,
                                     pHdr->sa,
                                     LIM_NO_WEP_IN_FC,psessionEntry);

                return;
            }
            /// No Pre-auth context exists for the STA.
#ifdef WLAN_SOFTAP_FEATURE
            if (limIsAuthAlgoSupported(
                                      pMac,
                                      (tAniAuthType)
                                      pRxAuthFrameBody->authAlgoNumber, psessionEntry))
#else
            if (limIsAuthAlgoSupported(
                                      pMac,
                                      (tAniAuthType)
                                      pRxAuthFrameBody->authAlgoNumber))

#endif
            {
                switch (pRxAuthFrameBody->authAlgoNumber)
                {
                    case eSIR_OPEN_SYSTEM:
