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
   *  Modefied: 2013/11/30
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
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
		#define SENDQ_SIZE    3
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
		#define SENDQ_SIZE    5
#endif


#define GW_LOST      0
#define GW_SEARCHING 1
#define GW_FIND      2

#define CL_DISCONNECTED  0
#define CL_ACTIVE        1
#define CL_ASLEEP        2
#define CL_AWAKE         3



using namespace tomyClient;

/*=====================================
        Class ClientStatus
 ======================================*/
class ClientStatus{
public:
	ClientStatus();
	~ClientStatus();

	bool isLost();
	bool isSearching();
	bool isConnected();
	bool isAvailableToSend();
	bool isPINGREQRequired();
	bool isGatewayAlive();

	uint16_t getKeepAlive();
	void setKeepAlive(uint16_t sec);
	void sendSEARCHGW();
	void recvGWINFO();
	void recvADVERTISE(MqttsAdvertise* adv);
	void recvCONNACK();
	void recvDISCONNECT();
	void recvPINGRESP();
	void setLastSendTime();
	void init();

private:

	uint8_t _gwId;
	uint8_t _gwStat;
	uint8_t _clStat;
	uint16_t _keepAliveDuration; // PINGREQ interval
	uint16_t _advertiseDuration; // Gateway heart beat
	XTimer   _keepAliveTimer;
	XTimer   _advertiseTimer;
};

/*=====================================
        Class SendQue  (FIFO)
 ======================================*/
class SendQue {
public:
    SendQue();
    ~SendQue();
    int addRequest(MqttsMessage* msg);
    int addPriorityRequest(MqttsMessage* msg);
    void setStatus(uint8_t index, uint8_t status);
    MqttsMessage* getMessage(uint8_t index);
    int  getStatus(uint8_t index);
    uint8_t getCount();
    int deleteRequest(uint8_t index);
    void   deleteAllRequest();
    void setQueSize(uint8_t sz);
private:
    uint8_t   _queSize;
    uint8_t   _queCnt;
    MqttsMessage*  _msg[SENDQ_SIZE];
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
    #if ARDUINO < 100
        void begin(long baudrate, int serialPortNum);
    #endif
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
    void setGwAddress(XBeeAddress64& addr64, uint16_t addr16);
    uint16_t getRxRemoteAddress16();
    XBeeAddress64& getRxRemoteAddress64();
    MQString* getClientId();


    int  publish(MQString* topic, const char* data, int dataLength);
    int  publish(MQString* topic, MQString* data);
    int  publish(uint16_t predifinedId,  const char* data, int dataLength);
    int  registerTopic(MQString* topic);
    int  subscribe(MQString* topic, TopicCallback callback);
    int  subscribe(uint16_t predefinedId, TopicCallback callback);
    int  unsubscribe(MQString* topic);
    int  unsubscribe(uint16_t predefinedId);
    int  disconnect(uint16_t duration = 0);

    void recieveMessageHandler(ZBResponse* msg, int* returnCode);
    void publishHdl(MqttsPublish* msg);
    void recvMsg(uint16_t msec);
    int  exec();
    uint8_t getMsgRequestCount();

private:
    int  sendRecvMsg();
    void clearMsgRequest();
    int  requestSendMsg(MqttsMessage* msg);
    int  requestPrioritySendMsg(MqttsMessage* mqttsMsgPtr);
    bool isRequestable();
    int  broadcast(uint16_t packetReadTimeout);
    int  unicast(uint16_t packetReadTimeout);

    int  searchGw(uint8_t radius);
    int  connect();
    int  pingReq(MQString* clietnId);
    int  willTopic();
    int  willMsg();
    int  pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc);
    int  regAck(uint16_t topicId, uint16_t msgId, uint8_t rc);

    uint8_t getMsgRequestType();
    uint8_t getMsgRequestStatus();
    void   setMsgRequestStatus(uint8_t stat);
    void createTopic(MQString* topic, TopicCallback callback);

    void delayTime(uint16_t baseTime);
    void copyMsg(MqttsMessage* msg, ZBResponse* recvMsg);
    uint16_t getNextMsgId();

    ZBeeStack*       _zbee;
    SerialPort*      _sp;
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
    ClientStatus     _clientStatus;
    bool             _sendFlg;
};



#endif /* MQTTSCLIENT_H_ */
