/*
 * MqttsClient.cpp
 *
 *                    The MIT License (MIT)
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
 * THE SOFTWARE.*
 *
 * 
 *  Created on: 2013/06/23
 *    Modified: 2013/11/30
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.1.0
 *
 */
#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Arduino_defs.h>
#endif

#ifdef ARDUINO
  #include <MqttsClient.h>

  #ifdef DEBUG
        #include <SoftwareSerial.h>
        extern SoftwareSerial debug;
  #endif

#endif  /* ARDUINO */

#ifdef MBED
  #include "mbed.h"
  #include "MqttsClient.h"
#endif  /* MBED */

#ifdef LINUX
  #include "MqttsClient.h"
  #include <stdio.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <string.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <termios.h>
#endif /* LINUX */

using namespace std;
using namespace tomyClient;

static MqttsClient*  theMqtts;

/*===============================================================================

        Class MqttsClient

 ================================================================================*/
void ResponseHandler(ZBResponse* resp, int* returnCode){
        theMqtts->recieveMessageHandler(resp, returnCode);
}

MqttsClient::MqttsClient(){
    _sp = new SerialPort();
    _zbee = new ZBeeStack();
    _zbee->setSerialPort(_sp);
    _zbee->setRxHandler(ResponseHandler);
    _sendQ = new SendQue();
    _qos = 0;
    _duration = 0;
    _clientId = new MQString();
    _clientFlg = 0;
    _nRetry = 5;
    _nRetryCnt = 0;
    _tRetry = 0;
    _willTopic = _willMessage = NULL;
    _status.setKeepAlive(MQTTS_DEFAULT_KEEPALIVE);
    _msgId = 0;
    _topics.allocate(MQTTS_MAX_TOPICS);
    theMqtts = this;
}

MqttsClient::~MqttsClient(){
  _sendQ->deleteAllRequest();
  delete _zbee;
  delete _sp;
}

#ifdef ARDUINO
void MqttsClient::begin(long baudrate){
        _sp->begin(baudrate);
}
#endif /* ARDUINO */

#ifdef MBED
void MqttsClient::begin(long baudrate){
            _sp->begin(baudrate);
    }
#endif /* MBED */

#ifdef LINUX
  void MqttsClient::begin(char* device, unsigned int bauderate){
      if( _sp->begin(device, bauderate) < 0){
      printf(" Serialport open Error %s", device);
        exit(-1);
      }
  }
#endif  /* LINUX */




bool MqttsClient::init(const char* clientNameId){
    _clientId->copy(clientNameId);
    MQString* pre1 = new MQString(MQTTS_TOPIC_PREDEFINED_TIME);
    _topics.addTopic(pre1);
    _topics.setTopicId(pre1,MQTTS_TOPICID_PREDEFINED_TIME);
    return _zbee->init(clientNameId);
}

Topics* MqttsClient::getTopics(){
    return &_topics;
}

void MqttsClient::setKeepAlive(uint16_t sec){
	_status.setKeepAlive(sec);
}

void MqttsClient::setWillTopic(MQString* topic){
    _willTopic = topic;
    _clientFlg |= MQTTS_FLAG_WILL;
}

void MqttsClient::setWillMessage(MQString* msg){
    _willMessage = msg;
    _clientFlg |= MQTTS_FLAG_WILL;
}

void MqttsClient::setQos(uint8_t level){
    if (level == 0){
            _clientFlg |= MQTTS_FLAG_QOS_0;
    }else if (level == 1){
            _clientFlg |= MQTTS_FLAG_QOS_1;
    }
    _qos = level;
}

void MqttsClient::setRetain(bool retain){
    _clientFlg |= MQTTS_FLAG_RETAIN;
}

void MqttsClient::setClean(bool clean){
    _clientFlg |= MQTTS_FLAG_CLEAN;
}

void MqttsClient::setRetryMax(uint8_t cnt){
    _nRetry = cnt;
}

XBeeAddress64& MqttsClient::getRxRemoteAddress64(){
    return _zbee->getRxRemoteAddress64();
}

uint16_t MqttsClient::getRxRemoteAddress16(){
    return _zbee->getRxRemoteAddress16();
}

void MqttsClient::setGwAddress(XBeeAddress64& addr64, uint16_t addr16){
	_zbee->setGwAddress(addr64, addr16);
}

MQString* MqttsClient::getClientId(){
    return _clientId;
}


uint16_t MqttsClient::getNextMsgId(){
    _msgId++;
    if (_msgId == 0){
        _msgId = 1;
    }
    return _msgId;
}

uint8_t MqttsClient::getMsgRequestType(){
    if (_sendQ->getMessage(0)){
        return _sendQ->getMessage(0)->getType();
    }else{
        return 0;
    }
}
uint8_t MqttsClient::getMsgRequestStatus(){
    return _sendQ->getStatus(0);
}

uint8_t MqttsClient::getMsgRequestCount(){
  return _sendQ->getCount();
}

void MqttsClient::setMsgRequestStatus(uint8_t stat){
    _sendQ->setStatus(0,stat);
}

void MqttsClient::createTopic(MQString* topic, TopicCallback callback){
    _topics.addTopic(topic);
    _topics.setCallback(topic, callback);
}

void MqttsClient::delayTime(uint16_t maxTime){
#ifdef ARDUINO
    srand((uint32_t)millis( ));
    uint32_t tm = rand() % (maxTime * 1000);
#else
    srand((uint32_t)time( NULL ));
    uint32_t tm = (rand() % (maxTime * 1000));
#endif
    XTimer delayTimer;
    delayTimer.start(tm);
    while(!delayTimer.isTimeUp()){
        _zbee->readPacket();
    }
}

void MqttsClient::copyMsg(MqttsMessage* msg, ZBResponse* recvMsg){
    memcpy(msg->getMsgBuff(), recvMsg->getPayload(), recvMsg->getPayload(0));
}

void MqttsClient::clearMsgRequest(){
    _sendQ->deleteRequest(0);
}

/*========================================
 *   Create & send the MQTT-S Messages
 =========================================*/

/*--------- CONNECT ------*/
int MqttsClient::connect(){
    MqttsConnect mqttsMsg = MqttsConnect(_clientId);
    mqttsMsg.setDuration(_status.getKeepAlive());
    mqttsMsg.setFlags(_clientFlg);
    return requestPrioritySendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- PUBLISH ------*/
int MqttsClient::publish(MQString* topic, const char* data, int dataLength){
    uint16_t topicId = _topics.getTopicId(topic);
    if (topicId){
        MqttsPublish mqttsMsg = MqttsPublish();
        mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_SHORT);
        mqttsMsg.setTopicId(topicId);
        mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
        if (_qos){
            mqttsMsg.setMsgId(getNextMsgId());
        }
        requestSendMsg((MqttsMessage*)&mqttsMsg);
        return run();
    }
    return MQTTS_ERR_NO_TOPICID;
}

/*--------- PUBLISH ------*/
int MqttsClient::publish(uint16_t predefinedId, const char* data, int dataLength){
    MqttsPublish mqttsMsg = MqttsPublish();
    mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_PREDEFINED);
    mqttsMsg.setTopicId(predefinedId);
    mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
    if (_qos){
        mqttsMsg.setMsgId(getNextMsgId());
    }
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- PUBACK ------*/
int MqttsClient::pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsPubAck mqttsMsg = MqttsPubAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);
    requestPrioritySendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- REGISTER ------*/
int MqttsClient::registerTopic(MQString* topic){
    MqttsRegister mqttsMsg = MqttsRegister();
    mqttsMsg.setTopicName(topic);
    mqttsMsg.setMsgId(getNextMsgId());
    _topics.addTopic(topic);
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- SUBSCRIBE ------*/
int MqttsClient::subscribe(MQString* topic, TopicCallback callback){
    MqttsSubscribe mqttsMsg = MqttsSubscribe();
    uint16_t topicId = _topics.getTopicId(topic);
    if (topicId){
        mqttsMsg.setTopicId(topicId);
        mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_NORMAL);
    }else{
        mqttsMsg.setTopicName(topic);
        mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_SHORT);
        _topics.addTopic(topic);
        _topics.setCallback(topic, callback);
    }
    mqttsMsg.setMsgId(getNextMsgId());
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- SUBSCRIBE ------*/
int MqttsClient::subscribe(uint16_t predefinedId, TopicCallback callback){
    MqttsSubscribe mqttsMsg = MqttsSubscribe();
    mqttsMsg.setTopicId(predefinedId);
    mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_PREDEFINED);
    mqttsMsg.setMsgId(getNextMsgId());
    _topics.setCallback(predefinedId, callback);
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- UNSUBSCRIBE ------*/
int MqttsClient::unsubscribe(MQString* topic){
    MqttsUnsubscribe mqttsMsg = MqttsUnsubscribe();
    uint16_t topicId = _topics.getTopicId(topic);
    if (topicId){
        mqttsMsg.setTopicId(topicId);
        mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_NORMAL);
    }else{
        mqttsMsg.setTopicName(topic);
        mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_SHORT);
    }
    mqttsMsg.setMsgId(getNextMsgId());
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- UNSUBSCRIBE ------*/
int MqttsClient::unsubscribe(uint16_t predefinedId){
    MqttsUnsubscribe mqttsMsg = MqttsUnsubscribe();
    mqttsMsg.setTopicId(predefinedId);
    mqttsMsg.setMsgId(getNextMsgId());
    mqttsMsg.setFlags(_clientFlg | MQTTS_TOPIC_TYPE_PREDEFINED);
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- DISCONNECT ------*/
int MqttsClient::disconnect(uint16_t duration){
    MqttsDisconnect mqttsMsg = MqttsDisconnect();
    if (duration){
        mqttsMsg.setDuration(duration);
    }
    requestSendMsg((MqttsMessage*)&mqttsMsg);
    return run();
}

/*--------- SEARCHGW ------*/
int  MqttsClient::searchGw(uint8_t radius){
    MqttsSearchGw mqttsMsg = MqttsSearchGw();
    mqttsMsg.setRadius(radius);
    return requestPrioritySendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- PINGREQ ------*/
int  MqttsClient::pingReq(MQString* clientId){
    MqttsPingReq mqttsMsg = MqttsPingReq(clientId);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- WILLTOPIC ------*/
int MqttsClient::willTopic(){
    MqttsWillTopic mqttsMsg = MqttsWillTopic();
    mqttsMsg.setWillTopic(_willTopic);
    mqttsMsg.setFlags(_clientFlg & 0x70);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- WILLMSG ------*/
int MqttsClient::willMsg(){
    MqttsWillMsg mqttsMsg = MqttsWillMsg();
    mqttsMsg.setWillMsg(_willMessage);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/* ===================================================
          Procedures for  Received Messages
 =====================================================*/
void MqttsClient::recieveMessageHandler(ZBResponse* recvMsg, int* returnCode){
    if ( _status.isSearching() && (recvMsg->getPayload(1) != MQTTS_TYPE_GWINFO)){
        *returnCode = MQTTS_ERR_NO_ERROR;

/*---------  PUBLISH  --------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_PUBLISH){
    	D_MQTTW("PUBLISH received\r\n");
        MqttsPublish mqMsg = MqttsPublish();
        mqMsg.setFrame(recvMsg);
		//*returnCode = _pubHdl.exec(&mqMsg,&_topics);   // ReturnCode
		_pubHdl.exec(&mqMsg,&_topics);   // ReturnCode
		if (mqMsg.getQos() && MQTTS_FLAG_QOS_1){
			pubAck(mqMsg.getTopicId(), mqMsg.getMsgId(), MQTTS_RC_ACCEPTED);
			run();
		}

/*---------  PUBACK  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_PUBACK && getMsgRequestStatus() == MQTTS_MSG_WAIT_ACK){
        MqttsPubAck mqMsg = MqttsPubAck();
        copyMsg(&mqMsg, recvMsg);
        D_MQTT("\nPUBACK received ReturnCode=");
        D_MQTTLN(mqMsg.getReturnCode(),HEX);
        D_MQTTLN();
        D_MQTTF("\nPUBACK received ReturnCode=%d\r\n", mqMsg.getReturnCode());
        if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 3)){
            if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
                setMsgRequestStatus(MQTTS_MSG_COMPLETE);

            }else if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
                  setMsgRequestStatus(MQTTS_MSG_RESEND_REQ);

            }else{
                *returnCode = MQTTS_ERR_REJECTED;          // ReturnCode
                setMsgRequestStatus(MQTTS_MSG_REJECTED);
            }
        }

/*---------  PINGRESP  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_PINGRESP){
        D_MQTTW(" PINGRESP received\r\n");

        _status.recvPINGRESP();
        if (getMsgRequestType() == MQTTS_TYPE_PINGREQ){
            setMsgRequestStatus(MQTTS_MSG_COMPLETE);
        }

/*---------  ADVERTISE  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_ADVERTISE){
        D_MQTTW(" ********** ADVERTISE received\r\n");

        MqttsAdvertise mqMsg = MqttsAdvertise();
        copyMsg(&mqMsg, recvMsg);
        _status.recvADVERTISE(&mqMsg);

/*---------  GWINFO  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_GWINFO){
        D_MQTTW(" GWINFO received\r\n");
        MqttsGwInfo mqMsg = MqttsGwInfo();
        copyMsg(&mqMsg, recvMsg);
        if (getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
            setMsgRequestStatus(MQTTS_MSG_COMPLETE);
            _status.recvGWINFO();
            _zbee->setGwAddress(_zbee->getRxRemoteAddress64(), _zbee->getRxRemoteAddress16());
        }

/*---------  CONNACK  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_CONNACK){
        D_MQTTW(" CONNACK received");
        if ((getMsgRequestType() == MQTTS_TYPE_CONNECT || getMsgRequestType() == MQTTS_TYPE_WILLMSG)){
            MqttsConnack mqMsg = MqttsConnack();
            copyMsg(&mqMsg, recvMsg);

            D_MQTT(" RC = 0x");
            D_MQTT(mqMsg.getReturnCode(),HEX);
            D_MQTTF(" RC = 0x%x", mqMsg.getReturnCode());

            if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
                setMsgRequestStatus(MQTTS_MSG_COMPLETE);
                _status.recvCONNACK();

            }else if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
            	setMsgRequestStatus(MQTTS_MSG_COMPLETE);
                _status.recvDISCONNECT();
            }else{
               setMsgRequestStatus(MQTTS_MSG_REJECTED);
               *returnCode = MQTTS_ERR_REJECTED;          // Return Code
               //clearMsgRequest();
               _status.recvDISCONNECT();
            }
        }
        D_MQTTW("\r\n");

/*---------  REGISTER  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_REGISTER){
        D_MQTTW(" REGISTER received\r\n");
        MqttsRegister mqMsg = MqttsRegister();

        mqMsg.setFrame(recvMsg);
        uint16_t topicId = _topics.getTopicId(mqMsg.getTopicName());
        if (topicId == 0){
            if (_topics.match(mqMsg.getTopicName())){
                MQString* mqStr = mqMsg.getTopicName()->create();
                _topics.addTopic(mqStr);
                _topics.setTopicId(mqStr,mqMsg.getTopicId());
                _topics.setCallback(mqMsg.getTopicId(),_topics.match(mqStr)->getCallback());
            }
        }

/*---------  REGACK  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_REGACK){
         D_MQTTW(" REGACK received\r\n");

        if (getMsgRequestStatus() == MQTTS_MSG_WAIT_ACK &&
            getMsgRequestType() == MQTTS_TYPE_REGISTER){
            MqttsRegAck mqMsg = MqttsRegAck();
            copyMsg(&mqMsg, recvMsg);
            if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 2)){
                if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 2)){
                    if (getUint16((uint8_t*)_sendQ->getMessage(0)->getBody()+4)){
                        if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
                            setMsgRequestStatus(MQTTS_MSG_COMPLETE);
                            MQString topic;
                            topic.readBuf(_sendQ->getMessage(0)->getBody() + 4);
                            _topics.setTopicId(&topic, mqMsg.getTopicId());
                        }else if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
                          setMsgRequestStatus(MQTTS_MSG_RESEND_REQ);
                        }else{
                            *returnCode = MQTTS_ERR_REJECTED;
                        }
                    }
                }
            }
        }

/*---------  SUBACK  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_SUBACK && getMsgRequestStatus() == MQTTS_MSG_WAIT_ACK){
        MqttsSubAck mqMsg = MqttsSubAck();
        copyMsg(&mqMsg, recvMsg);

        D_MQTTLN();
        D_MQTT("SUBACK ReturnCode=");
        D_MQTTLN(mqMsg.getReturnCode(),HEX);
        D_MQTTF("\nSUBACK ReturnCode=%d\r\n", mqMsg.getReturnCode());

        if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 1)){
            if (mqMsg.getReturnCode() == MQTTS_RC_ACCEPTED){
                setMsgRequestStatus(MQTTS_MSG_COMPLETE);
                if (_sendQ->getMessage(0)->getBodyLength() > 5){ // TopicName is not Id
                    MQString topic;
                    topic.readBuf(_sendQ->getMessage(0)->getBody() + 3);
                    _topics.setTopicId(&topic, mqMsg.getTopicId());

                }
            }else if (mqMsg.getReturnCode() == MQTTS_RC_REJECTED_CONGESTION){
                setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                *returnCode = MQTTS_ERR_REJECTED;       // Return Code
            }
        }

/*---------  UNSUBACK  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_UNSUBACK && getMsgRequestStatus() == MQTTS_MSG_WAIT_ACK){
        D_MQTTW(" UNSUBACK received\r\n");
        MqttsUnSubAck mqMsg = MqttsUnSubAck();
        copyMsg(&mqMsg, recvMsg);
        if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 1)){
              setMsgRequestStatus(MQTTS_MSG_COMPLETE);
        }

/*---------  DISCONNECT  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_DISCONNECT && getMsgRequestStatus() == MQTTS_MSG_WAIT_ACK){
         D_MQTTW(" DISCONNECT received\r\n");
         setMsgRequestStatus(MQTTS_MSG_COMPLETE);
         _status.recvDISCONNECT();


/*---------  WILLTOPICREQ  ----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_WILLTOPICREQ){
        D_MQTTW(" WILLTOPICREQ received\r\n");
        if (getMsgRequestType() == MQTTS_TYPE_CONNECT){
            //setMsgRequestStatus(MQTTS_MSG_COMPLETE);
            clearMsgRequest();
            MqttsWillTopic mqMsg = MqttsWillTopic();
            mqMsg.setWillTopic(_willTopic);

            if (  _sendQ->addPriorityRequest((MqttsMessage*)&mqMsg) == 0){
                setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                *returnCode = MQTTS_ERR_OUT_OF_MEMORY;   // Return Code
            }
        }

/*---------  WILLMSGREQ  -----------*/
    }else if (recvMsg->getPayload(1) == MQTTS_TYPE_WILLMSGREQ){
        D_MQTTW(" WILLMSGREQ received\r\n");
        if (getMsgRequestType() == MQTTS_TYPE_WILLTOPIC){
            //setMsgRequestStatus(MQTTS_MSG_COMPLETE);
            clearMsgRequest();
            MqttsWillMsg mqMsg = MqttsWillMsg();
            mqMsg.setWillMsg(_willMessage);
            if (_sendQ->addPriorityRequest((MqttsMessage*)&mqMsg) == 0){
                setMsgRequestStatus(MQTTS_MSG_REQUEST);
            }else{
                *returnCode = MQTTS_ERR_OUT_OF_MEMORY;   // Return Code
            }
        }
    }
}


/*========================================================
    Send a MQTT-S Message (add the send request)
==========================================================*/
int MqttsClient::requestSendMsg(MqttsMessage* mqttsMsgPtr){
    int index = _sendQ->addRequest((MqttsMessage*)mqttsMsgPtr);
    if ( index >= 0){
        if (_sendQ->getStatus(index) != MQTTS_MSG_RESEND_REQ){
            _sendQ->setStatus(index, MQTTS_MSG_REQUEST);
        }
        return MQTTS_ERR_NO_ERROR;
    }
    return MQTTS_ERR_CANNOT_ADD_REQUEST;
}

/*========================================================
  Send a MQTT-S Message (add to the top of the send request)
==========================================================*/
int MqttsClient::requestPrioritySendMsg(MqttsMessage* mqttsMsgPtr){
    if (_sendQ->addPriorityRequest((MqttsMessage*)mqttsMsgPtr)){
        return MQTTS_ERR_CANNOT_ADD_REQUEST;
    }else{
        if (_sendQ->getStatus(0) != MQTTS_MSG_RESEND_REQ){
            _sendQ->setStatus(0, MQTTS_MSG_REQUEST);
        }
        return MQTTS_ERR_NO_ERROR;
    }
}

/*========================================================
  Execute sending a MQTT-S Message
==========================================================*/

/*-------------  send Message once -----------------*/
int MqttsClient::run(){
    int rc;

    while(true){
        rc = execMsgRequest();

		if (rc == MQTTS_ERR_NO_ERROR){
			break;
		}
	}
    D_MQTTW("---- returned from run()\r\n ");
    return rc;
}

/*=============================
 *   Send or Receive Message
 ==============================*/
int MqttsClient::execMsgRequest(){
    int rc;

    if (getMsgRequestStatus() == MQTTS_MSG_REQUEST || getMsgRequestStatus() == MQTTS_MSG_RESEND_REQ){

    	if (_status.isAvailableToSend()){
			/*-------- Send PUBLISH,SUBSCRIBE,REGISTER,DISCONNECT,PUBACK -------*/
			return unicast(MQTTS_TIME_RETRY);
		}

    	if (_status.isLost()){
    		/*------------ create SEARCHGW --------------*/
            if ( searchGw(ZB_BROADCAST_RADIUS_MAX_HOPS) == MQTTS_ERR_CANNOT_ADD_REQUEST){
                return MQTTS_ERR_CANNOT_ADD_REQUEST;
            }
        }

        if (_status.isLost() && !_status.isSearching()){
            /*------------ Send SEARCHGW --------------*/
            if (getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                delayTime(MQTTS_TIME_SEARCHGW);
                if (getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                    _status.sendSEARCHGW();
                    rc = broadcast(MQTTS_TIME_SEARCHGW);
                    if ( rc != MQTTS_ERR_NO_ERROR){
                        return rc;
                    }
                }
            }
        }

        if (!_status.isConnected()){
			/*-----------  Send CONNECT ----------*/
			if (connect() == MQTTS_ERR_CANNOT_ADD_REQUEST){
			   return MQTTS_ERR_CANNOT_ADD_REQUEST;
			}else if(unicast(MQTTS_TIME_RETRY) == MQTTS_ERR_NO_ERROR){
				   return MQTTS_ERR_NO_ERROR;
			}
			return MQTTS_ERR_NOT_CONNECTED;
        }else{
            return MQTTS_ERR_NO_ERROR;
        }

    }else{
        if (_status.isPINGREQRequired()){
            /*-------- Send PINGREQ -----------*/
            pingReq(_clientId);
            if (unicast(MQTTS_TIME_RETRY)){
                return MQTTS_ERR_PINGRESP_TIMEOUT;
            }
        }
        _zbee->readPacket();  //  Receive MQTT-S Message
        return MQTTS_ERR_NO_ERROR;
    }
}

/*------------------------------------
 *   Broad cast the MQTT-S Message
 -------------------------------------*/
int MqttsClient::broadcast(uint16_t packetReadTimeout){
    int retry = 0;
    while(retry < _nRetry){
        _zbee->send(_sendQ->getMessage(0)->getMsgBuff(), _sendQ->getMessage(0)->getLength(), 0, BcastReq);
        _respTimer.start(packetReadTimeout * 1000);

        while(!_respTimer.isTimeUp()){
           if ((_qos == 0 && getMsgRequestType() != MQTTS_TYPE_SEARCHGW)|| getMsgRequestStatus() == MQTTS_MSG_COMPLETE){
        	   clearMsgRequest();
               return MQTTS_ERR_NO_ERROR;

           }else if (getMsgRequestStatus() == MQTTS_MSG_REQUEST){
               setMsgRequestStatus(MQTTS_MSG_WAIT_ACK);
           }
           _zbee->readResp();
        }

        setMsgRequestStatus(MQTTS_MSG_REQUEST);
        retry++;
    }
   return MQTTS_ERR_RETRY_OVER;
}

/*------------------------------------
 *   Unicast the MQTT-S Message
 -------------------------------------*/
int MqttsClient::unicast(uint16_t packetReadTimeout){
    int retry = 0;
    while(retry < _nRetry){
    	/*------ Send Top message in SendQue -----*/
    	if (getMsgRequestStatus() != MQTTS_MSG_REQUEST){
    		return MQTTS_ERR_NO_ERROR;
    	}
        _zbee->send(_sendQ->getMessage(0)->getMsgBuff(), _sendQ->getMessage(0)->getLength(), 0, UcastReq);
        _sendQ->getMessage(0)->setDup();
        _respTimer.start(packetReadTimeout * 1000);
        _status.setLastSendTime();

        while(!_respTimer.isTimeUp()){
            if ((_qos == 0 && getMsgRequestType() != MQTTS_TYPE_PINGREQ )  ||
                              getMsgRequestType() == MQTTS_TYPE_PUBACK     ||
                              getMsgRequestType() == MQTTS_TYPE_DISCONNECT ||
                              getMsgRequestStatus() == MQTTS_MSG_COMPLETE ){
            	clearMsgRequest();
                return MQTTS_ERR_NO_ERROR;

            }else if (getMsgRequestStatus() == MQTTS_MSG_REJECTED){
            	clearMsgRequest();
                return MQTTS_ERR_REJECTED;

            }else if (getMsgRequestStatus() == MQTTS_MSG_RESEND_REQ){

            	/* ------  Re send Time delay -------*/
                #ifdef ARDUINO
                  delay(MQTTS_TIME_WAIT * 1000);
                #else
                    #ifdef MBED
                        wait_ms(MQTTS_TIME_WAIT * 1000);
                    #else
                        usleep(MQTTS_TIME_WAIT * 1000000);
                    #endif
                #endif

                /* ----- Re send  Top message in SendQue ---*/
				_zbee->send(_sendQ->getMessage(0)->getMsgBuff(), _sendQ->getMessage(0)->getLength(), 0, UcastReq);
				setMsgRequestStatus(MQTTS_MSG_WAIT_ACK);

            }else if (getMsgRequestStatus() == MQTTS_MSG_REQUEST){
                setMsgRequestStatus(MQTTS_MSG_WAIT_ACK);
            }
            /*----- Read response  ----*/
            _zbee->readResp();

            if (getMsgRequestStatus() == MQTTS_MSG_REQUEST &&
                (getMsgRequestType() == MQTTS_TYPE_WILLTOPIC ||
                getMsgRequestType() == MQTTS_TYPE_WILLMSG) ){
                break;
            }
        }
        setMsgRequestStatus(MQTTS_MSG_REQUEST);
        retry++;
    }
    return MQTTS_ERR_RETRY_OVER;
}



/*=====================================
        Class SendQue
 ======================================*/
SendQue::SendQue(){
    _queCnt = 0;
    _queSize = SENDQ_SIZE;
}
SendQue::~SendQue(){
    for( int i = 0; i < _queCnt; i++){
        delete _msg[i];
    }
}

int SendQue::addRequest(MqttsMessage* msg){
    if ( _queCnt < _queSize){
		D_MQTTW("\nAdd SendQue size = ");
		D_MQTT(_queCnt + 1, DEC);
		D_MQTT(" MsgType = 0x")
		D_MQTTLN(msg->getType(), HEX);
		D_MQTTF("%d  MsgType = 0x%x\r\n", _queCnt + 1, msg->getType());

        _msg[_queCnt] =new MqttsMessage();
        _msg[_queCnt++]->copy(msg);
        return _queCnt - 1;
    }
    return MQTTS_ERR_CANNOT_ADD_REQUEST; // Over Que size
}

int SendQue::addPriorityRequest(MqttsMessage* msg){
    if ( _queCnt < _queSize){
		D_MQTTW("\nAdd SendQue Top Size = ");
		D_MQTT(_queCnt + 1, DEC);
		D_MQTT(" MsgType = 0x");
		D_MQTTLN(msg->getType(), HEX);
		D_MQTTF("%d MsgType = 0x%x\r\n", _queCnt + 1, msg->getType());

        for(int i = _queCnt; i > 0; i--){
            _msg[i] = _msg[i - 1];
        }
        _msg[0] = new MqttsMessage();
        _msg[0]->copy(msg);
        _queCnt++;
        return 0;
    }
    return MQTTS_ERR_CANNOT_ADD_REQUEST;
}

int SendQue::deleteRequest(uint8_t index){
	uint8_t type = 0;
    if ( index < _queCnt){
    	type = _msg[index]->getType();

        delete _msg[index];
        _queCnt--;
        for(int i = index; i < _queCnt; i++){
            _msg[i] = _msg[i + 1];
        }
        for(int i = _queCnt; i < _queSize; i++){
            _msg[i] = NULL;
        }
    	D_MQTTW("\nDelete SendQue  size = ");
    	D_MQTTLN( _queCnt, DEC);
    	D_MQTT( " MsgType = 0x");
    	D_MQTTLN(type, HEX);
    	D_MQTTF("%d MsgType = 0x%x\r\n", _queCnt, type);
        return 0;
    }
    return -2;
}

void   SendQue::deleteAllRequest(){
    while ( _queCnt > 0){
        deleteRequest(0);
    }
}

void SendQue::setStatus(uint8_t index, uint8_t status){
    if ( index < _queCnt){
        _msg[index]->setStatus(status);
    }
}

void SendQue::setQueSize(uint8_t sz){
  _queSize = sz;
}

MqttsMessage* SendQue::getMessage(uint8_t index){
  if ( index < _queCnt){
      return _msg[index];
  }
  return NULL;
}

int SendQue::getStatus(uint8_t index){
  if ( index < _queCnt){
      return _msg[index]->getStatus();
  }
  return -1;
}

uint8_t SendQue::getCount(){
   return _queCnt;
}


/*=====================================
        Class SendQue
 ======================================*/
ClientStatus::ClientStatus(){
	_gwId = 0;
	_gwStat = GW_LOST;
	_clStat = CL_DISCONNECTED;
	_keepAliveDuration = MQTTS_DEFAULT_DURATION;
	_advertiseDuration = MQTTS_DEFAULT_KEEPALIVE;
	_keepAliveTimer.stop();
	_advertiseTimer.stop();
}

ClientStatus::~ClientStatus(){

}

bool ClientStatus::isLost(){
	if(_gwStat == GW_LOST){
		return true;
	}else{
		return false;
	}
}

bool ClientStatus::isSearching(){
	if(_gwStat == GW_SEARCHING){
		return true;
	}else{
		return false;
	}
}


bool ClientStatus::isConnected(){
	if(_gwStat == GW_FIND && _clStat != CL_DISCONNECTED){
		return true;
	}else{
		return false;
	}
}

bool ClientStatus::isAvailableToSend(){
	if((_gwStat == GW_FIND) &&
		((_clStat == CL_ACTIVE) || (_clStat == CL_ASLEEP))){
		return true;
	}else{
		return false;
	}
}

bool ClientStatus::isPINGREQRequired(){
	return (_keepAliveTimer.isTimeUp(_keepAliveDuration) && (_clStat != CL_DISCONNECTED));
}

bool ClientStatus::isGatewayAlive(){
	if(_advertiseTimer.isTimeUp(_advertiseDuration)){
		_gwStat = GW_LOST;
		return false;
	}else{
		return true;
	}
}

uint16_t ClientStatus::getKeepAlive(){
	return _keepAliveDuration / 1000;
}

void ClientStatus::setKeepAlive(uint16_t sec){
	_keepAliveDuration = sec * 1000;
}

void ClientStatus::sendSEARCHGW(){
	_gwStat = GW_SEARCHING;
}

void ClientStatus::recvGWINFO(){
	if (_gwStat == GW_SEARCHING){
		_gwStat = GW_FIND;
	}
}

void ClientStatus::recvADVERTISE(MqttsAdvertise* adv){
	if ( adv->getGwId() == _gwId || _gwId == 0){
		_advertiseTimer.start();
		_advertiseDuration = (adv->getDuration() > 60 ?
				adv->getDuration() * 1100 : adv->getDuration() * 1500);
		_gwId = adv->getGwId();
	}
}

void ClientStatus::recvCONNACK(){
	_clStat = CL_ACTIVE;
}

void ClientStatus::recvDISCONNECT(){
	_clStat = CL_DISCONNECTED;
}

void ClientStatus::setLastSendTime(){
	_keepAliveTimer.start();
}


void ClientStatus::recvPINGRESP(){
    _keepAliveTimer.start();
}




/*===================  End of file ====================*/
