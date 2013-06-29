/*
 * MqttsClient.h
 *                     The MIT License (MIT)
 *
 *               Copyright (c) 2013, Tomoaki YAMAGUCHI
 *                       All rights reserved.
 *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *
 *  Created on: 2013/06/19
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.1
 *
 */

#ifndef MQTTSCLIENT_H_
#define MQTTSCLIENT_H_

#ifndef ARDUINO
    #include "MQTTS_Defines.h"
#endif


#if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
        #include <MQTTS.h>
#else
        #if defined(ARDUINO) && ARDUINO < 100
                #include "WProgram.h"
                #include <inttypes.h>
                #include <MQTTS.h>
        #else
                #ifdef LINUX
                    #include <sys/time.h>
                #endif
                #include <iostream>
                #include "MQTTS.h"
        #endif
#endif

/*=====================================
        Class GatewayHandller
 ======================================*/
class GatewayHandller {
public:
    GatewayHandller();
    bool     isConnected();
    bool     isDisconnected();
    bool     isSearching();
    bool     isFound();
    bool     isLost();
    bool     isInit();
    bool     isPingRequired();
    XBeeAddress64*  getAddress64();
    uint16_t        getAddress16();
    void     setStatus(uint8_t status);
    void     recvGwInfo(MqttsGwInfo* msg);
    void     setLastSendTime();
    void     recvAdvertise(MqttsAdvertise* msg);
    void     setKeepAlive(uint16_t sec);
    uint16_t  getKeepAlive();
    void     recvPingResp();

private:
    uint8_t        _status;  //  0:init 1:searching 2:find 3:connected  4:disconnect 5:lost
    XBeeAddress64  _addr64;
    uint16_t       _addr16;
    uint8_t        _gwId;
    uint16_t       _keepAliveDuration;// PINGREQ interval
    uint16_t       _advertiseDuration;

    XTimer          _keepAliveTimer;
    XTimer          _advertiseTimer;
};

/*=====================================
        Class MqttsClient
 ======================================*/
class MqttsClient {
public:
    MqttsClient();
    ~MqttsClient();
  #ifdef ARDUINO
    void begin(long baudrate);
  #else
    #ifdef MBED
        void begin(long baudrate);
    #else
        void begin(char* device, unsigned int bauderate);  /* MBED & LINUX */
    #endif /* MBED */
  #endif /* ARDUINO */
    Topics* getTopics();
    bool init(const char* clientIdName);
    void setKeepAlive(uint16_t sec);
    void setWillTopic(MQString* topic);
    void setWillMessage(MQString* msg);
    void setQos(uint8_t level);
    void setRetain(bool retain);
    void setClean(bool clean);
    void setRetryMax(uint8_t cnt);
    MQString* getClientId();

    int  connect();
    int  publish(MQString* topic, const char* data, int dataLength);
    int  publish(uint16_t predifinedId,  const char* data, int dataLength);
    int  registerTopic(MQString* topic);
    int  subscribe(MQString* topic, TopicCallback callback);
    int  subscribe(uint16_t predefinedId, TopicCallback callback);
    int  unsubscribe(MQString* topic);
    int  unsubscribe(uint16_t predefinedId);
    int  disconnect(uint16_t duration = 0);

    void runConnect();
    int  run();

    void recieveMessageHandler(ZBRxResponse* msg, int* returnCode);
    void publishHdl(MqttsPublish* msg);
    void createTopic(MQString* topic, TopicCallback callback);
    uint16_t getRxRemoteAddress16();
    XBeeAddress64& getRxRemoteAddress64();
    uint8_t getMsgRequestType();
    uint8_t getMsgRequestStatus();
    uint8_t getMsgRequestCount();
    void   setMsgRequestStatus(uint8_t stat);
    bool   isGwConnected();
    void   clearMsgRequest();
    int    execMsgRequest();

private:
    int    requestSendMsg(MqttsMessage* msg);
    int    requestPrioritySendMsg(MqttsMessage* mqttsMsgPtr);
    int    broadcast(uint16_t packetReadTimeout);
    int    unicast(uint16_t packetReadTimeout);

    int  searchGw(uint8_t radius);
    int  pingReq(MQString* clietnId);
    int  pingResp();
    int  willTopic();
    int  willMsg();
    int  pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc);

    void delayTime(uint16_t baseTime);
    void copyMsg(MqttsMessage* msg, ZBRxResponse* recvMsg);
    uint16_t getNextMsgId();

    ZBeeStack*       _zbee;
    SerialPort*      _sp;
    GatewayHandller  _gwHdl;
    Topics           _topics;
    SendQue*         _sendQ;
    XTimer           _respTimer;
    PublishHandller  _pubHdl;

    uint8_t          _qos;
    uint16_t         _duration;
    MQString*        _clientId;
    uint8_t          _clientFlg;
    uint8_t          _nRetry;
    uint8_t          _nRetryCnt;
    uint16_t         _tRetry;
    MQString*         _willTopic;
    MQString*         _willMessage;
    uint16_t         _msgId;
};



#endif /* MQTTSCLIENT_H_ */
