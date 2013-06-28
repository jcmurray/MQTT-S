/*
 * MqttsGateway.cpp
 *
 *                        The MIT License (MIT)
 *
 *               Copyright (c) 2013, tomy-tech.com
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
 *  Created on: 2013/06/19
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.5.1
 *
 */

#ifndef ARDUINO
#include "MQTTS_Defines.h"



#ifdef MBED
  #include "mbed.h"
  #include "MqttsGateway.h"
#endif  /* MBED */

#ifdef LINUX
  #include "MqttsGateway.h"
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

static MqttsGateway*  theMqttsGw;

/*=======================================================================================

        Class MqttsGateway   for    DEBUG

 =======================================================================================*/
void ResponseHandlerGw(ZBRxResponse* resp, int* returnCode){
        theMqttsGw->recieveMessageHandler(resp, returnCode);
}

MqttsGateway::MqttsGateway(){
    _zbee = new ZBeeStack();
    _sp = new SerialPort();
    _zbee->setSerialPort(_sp);
    _zbee->setRxHandler(ResponseHandlerGw);
    _sendQ = new SendQue();
    _duration = MQTTS_DEFAULT_KEEPALIVE;
    _gwId = 0;
    _gatewayId = new MQString();
    _nRetry = 3;
    _nRetryCnt = 0;
    _tRetry = 0;
    //_nodeList = new ZBNodeList(30);
    _msgId = 0;
    _topicId = MQTTS_TOPICID_NORMAL;
    _topics.allocate(MQTTS_MAX_TOPICS);
    _advertiseTimer.start();
    theMqttsGw = this;
}

MqttsGateway::~MqttsGateway(){
  _sendQ->deleteAllRequest();
  delete _zbee;
  delete _sp;
}


#ifdef MBED
void MqttsGateway::begin(long baudrate){
        _sp->begin(baudrate);
}
#endif /* MBED */

#ifdef LINUX
void MqttsGateway::begin(char* device, unsigned int bauderate){
  if( _sp->begin(device, bauderate) < 0){
  printf(" Serialport open Error %s", device);
    exit(-1);
  }
}
#endif  /* LINUX */




Topics* MqttsGateway::getTopics(){
    return &_topics;
}


void MqttsGateway::setRetryMax(uint8_t cnt){
    _nRetry = cnt;
}

void MqttsGateway::setDuration(uint16_t sec){
    _duration = sec;
}

XBeeAddress64& MqttsGateway::getRxRemoteAddress64(){
    return _zbee->getRxRemoteAddress64();
}

uint16_t MqttsGateway::getRxRemoteAddress16(){
    return _zbee->getRxRemoteAddress16();
}

uint8_t MqttsGateway::getGwId(){
    return _gwId;
}

uint16_t MqttsGateway::getNextMsgId(){
    _msgId++;
    if (_msgId == 0){
        _msgId = 1;
    }
    return _msgId;
}

uint16_t MqttsGateway::getNextTopicId(){
    _topicId++;
    if (_topicId == 0){
        _topicId = MQTTS_TOPICID_NORMAL;
    }
    return _topicId;
}

void MqttsGateway::createTopic(MQString* topic, TopicCallback callback){
    _topics.addTopic(topic);
    _topics.setCallback(topic, callback);
}


void MqttsGateway::clearMsgRequest(){
    _sendQ->deleteRequest(0);
}

uint8_t MqttsGateway::getMsgRequestType(){
    if (_sendQ->getMessage(0)){
        return _sendQ->getMessage(0)->getType();
    }else{
        return 0;
    }
}
uint8_t MqttsGateway::getMsgRequestStatus(){
    return _sendQ->getStatus(0);
}

uint8_t MqttsGateway::getMsgRequestCount(){
  return _sendQ->getCount();
}

void MqttsGateway::setMsgRequestStatus(uint8_t stat){
    _sendQ->setStatus(0,stat);
}

void MqttsGateway::setLoopCtrl(uint8_t loopType){
    _loopCtrl = loopType;
}

uint8_t MqttsGateway::getLoopCtrl(){
    return _loopCtrl;
}

bool MqttsGateway::init(const char* gwNameId, uint8_t id){
    _gwId = id;
    _gatewayId->copy(gwNameId);
    MQString* pre1 = new MQString(MQTTS_TOPIC_PREDEFINED_TIME);
   _topics.addTopic(pre1);
   _topics.setTopicId(pre1,MQTTS_TOPICID_PREDEFINED_TIME);
    _advertiseTimer.start();
    return _zbee->init(ZB_GATEWAY, gwNameId);
}

int MqttsGateway::publishUnixTime(){
    long int tm = time(NULL);
    char payload[sizeof(long int)];
    memcpy(payload, &tm, sizeof(long int));
    return publish((uint16_t)MQTTS_TOPICID_PREDEFINED_TIME, (const char*)payload, sizeof(long int));

}

/*-------------------------------------------------------------------------*/

int MqttsGateway::requestSendMsg(MqttsMessage* mqttsMsgPtr){
    int index = _sendQ->addRequest((MqttsMessage*)mqttsMsgPtr);
    if ( index >= 0){
        if (_sendQ->getStatus(index) != MQTTS_MSG_RESEND_REQ){
            _sendQ->setStatus(index, MQTTS_MSG_REQUEST);
        }
        return index;
    }
    return MQTTS_ERR_CANNOT_ADD_REQUEST;
}

/*-------------------------------------------------------------------------*/

int MqttsGateway::requestPrioritySendMsg(MqttsMessage* mqttsMsgPtr){
    if (_sendQ->addPriorityRequest((MqttsMessage*)mqttsMsgPtr)){
        return MQTTS_ERR_CANNOT_ADD_REQUEST;
    }else{
        if (_sendQ->getStatus(0) != MQTTS_MSG_RESEND_REQ){
            _sendQ->setStatus(0, MQTTS_MSG_REQUEST);
        }
        return MQTTS_ERR_NO_ERROR;
    }
}

/*-------------------------------------------------------------------------*/
int MqttsGateway::execMsgRequest(){
    int rc = MQTTS_ERR_NO_ERROR;

    if (_sendQ->getStatus(0) == MQTTS_MSG_REQUEST || _sendQ->getStatus(0) == MQTTS_MSG_RESEND_REQ){
        if (_sendQ->getMessage(0)->getType() == MQTTS_TYPE_GWINFO){
            rc = broadcast(MQTTS_TIME_RETRY);
        }else{
            rc = unicast(MQTTS_TIME_RETRY);
        }
    }

    _zbee->readPacket();  //  Just read packet, No send Request

    if (_advertiseTimer.isTimeUp((uint32_t)(_duration * 1000))){
        advertise(_duration, _gwId);
        _advertiseTimer.start();
        printf("\n\n************************ADVERTISE\n\n"); //////////////////
        broadcast(MQTTS_TIME_RETRY);
    }
    return rc;
}

/*-------------------------------------------------------------------------*/
int MqttsGateway::broadcast(uint16_t packetReadTimeout){
  _zbee->bcastData(_sendQ->getMessage(0)->getMsgBuff(),
                                _sendQ->getMessage(0)->getLength());
  _sendQ->getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
  _sendQ->deleteRequest(0);
  return MQTTS_ERR_NO_ERROR;
}

int MqttsGateway::unicast(uint16_t packetReadTimeout){
  _zbee->sendData(&getRxRemoteAddress64(), getRxRemoteAddress16(),
                                  _sendQ->getMessage(0)->getMsgBuff(),
                                  _sendQ->getMessage(0)->getLength(), 0);
    _sendQ->getMessage(0)->setStatus(MQTTS_MSG_COMPLETE);
    _sendQ->deleteRequest(0);
    return MQTTS_ERR_NO_ERROR;
}
/*-------------------------------------------------------------------------*/



/*-----------------------------------------------
 *   Create Messages & send
 ------------------------------------------------*/
/*--------- PUBLISH ------*/
int MqttsGateway::publish(MQString* topic, const char* data, int dataLength){
  uint16_t topicId = _topics.getTopicId(topic);
  if (topicId){
      return publish(topicId, data, dataLength);
  }
  return MQTTS_ERR_NO_TOPICID;
}

int  MqttsGateway::publish(uint16_t topicId, const char* data, int dataLength){
    MqttsPublish mqttsMsg = MqttsPublish();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
    mqttsMsg.setMsgId(getNextMsgId());
    mqttsMsg.setFlags(MQTTS_FLAG_QOS_1);
    setLoopCtrl(MQTTS_TYPE_PUBLISH);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- PUBACK ------*/
int MqttsGateway::pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsPubAck mqttsMsg = MqttsPubAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);
    setLoopCtrl(MQTTS_TYPE_PUBACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- SUBACK ------*/
int MqttsGateway::subAck(uint16_t topicId, uint16_t msgId, uint8_t rc, uint8_t flag){
      MqttsSubAck mqttsMsg = MqttsSubAck();
      mqttsMsg.setFlags(flag);
      mqttsMsg.setTopicId(topicId);
      mqttsMsg.setMsgId(msgId);
      mqttsMsg.setReturnCode(rc);
      setLoopCtrl(MQTTS_TYPE_SUBACK);
      return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- REGISTER ------*/
int MqttsGateway::registerTopic(MQString* mqStr, uint16_t topicId){
    MqttsRegister mqttsMsg = MqttsRegister();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(getNextMsgId());
    mqttsMsg.setTopicName(mqStr);
    setLoopCtrl(MQTTS_TYPE_REGISTER);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- REGACK ------*/
int MqttsGateway::regAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsRegAck mqttsMsg = MqttsRegAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);
    setLoopCtrl(MQTTS_TYPE_REGACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- DISCONNECT ------*/
int MqttsGateway::disconnect(uint16_t duration){
    MqttsDisconnect mqttsMsg = MqttsDisconnect();
    if (duration){
        mqttsMsg.setDuration(duration);
    }
    setLoopCtrl(MQTTS_TYPE_DISCONNECT);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- GWINFO ------*/
int MqttsGateway::gwInfo(uint8_t gwId){
    MqttsGwInfo mqttsMsg = MqttsGwInfo();
    mqttsMsg.setGwId(gwId);
    setLoopCtrl(MQTTS_TYPE_GWINFO);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- CONNACK ------*/
int MqttsGateway::connAck(uint8_t rc){
    MqttsConnack mqttsMsg = MqttsConnack();
    mqttsMsg.setReturnCode(rc);
    setLoopCtrl(MQTTS_TYPE_CONNACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- UNSUBACK ------*/
int MqttsGateway::unsubAck(uint16_t msgId){
    MqttsUnSubAck mqttsMsg = MqttsUnSubAck();
    mqttsMsg.setMsgId(msgId);
    setLoopCtrl(MQTTS_TYPE_UNSUBACK);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- ADVERTISE ------*/
int  MqttsGateway::advertise(uint16_t duration, uint8_t gwId){
    MqttsAdvertise mqttsMsg = MqttsAdvertise();
    mqttsMsg.setDuration(duration);
    mqttsMsg.setGwId(gwId);
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- PINGRESP ------*/
int  MqttsGateway::pingResp(){
    MqttsPingResp mqttsMsg = MqttsPingResp();
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- WILLTOPICREQ ------*/
int  MqttsGateway::willTopicReq(){
    MqttsWillTopicReq mqttsMsg = MqttsWillTopicReq();
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}

/*--------- WILLMSGREQ ------*/
int  MqttsGateway::willMsgReq(){
    MqttsWillMsgReq mqttsMsg = MqttsWillMsgReq();
    return requestSendMsg((MqttsMessage*)&mqttsMsg);
}



/* ===================================================
                  Receive Message
 =====================================================*/
void MqttsGateway::recieveMessageHandler(ZBRxResponse* recvMsg, int* returnCode){

/*---------  PUBLISH  --------*/
    if (recvMsg->getData()[1] == MQTTS_TYPE_PUBLISH){
       printf("PUBLISH recv\n");
        MqttsPublish mqMsg = MqttsPublish();
        mqMsg.setFrame(recvMsg);
        if (mqMsg.getTopicId() == MQTTS_TOPICID_PREDEFINED_TIME){
            uint32_t t = (uint32_t(mqMsg.getData()[3]) << 24) +
                (uint32_t(mqMsg.getData()[2]) << 16) +
                (uint32_t(mqMsg.getData()[1]) <<  8) +
                mqMsg.getData()[0];
            printf("\n\n\n=====  %ld =======\n", t - time(NULL));
            struct tm *tm = localtime((const long*)&t);
            char date[20];
            strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", tm);
            printf("%s\n\n", date);
        }
        pubAck(mqMsg.getTopicId(), mqMsg.getMsgId(), MQTTS_RC_ACCEPTED);

/*---------  PUBACK  --------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PUBACK){
        printf("PUBACK recv\n");

/*---------  SUBSCRIBE  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_SUBSCRIBE){
        printf("SUBSCRIBE recv\n");
        MqttsSubscribe mqMsg = MqttsSubscribe();
        mqMsg.setFrame(recvMsg);
        uint16_t id;

        if (mqMsg.getFlags() && MQTTS_TOPIC_TYPE_PREDEFINED){
            id = mqMsg.getTopicId();
        }else{
            MQString* mqStr = mqMsg.getTopicName()->create();

            id = _topics.getTopicId(mqStr);
            if (!id){
                _topics.addTopic(mqStr);
                id = getNextTopicId();
                _topics.setTopicId(mqStr, id);
            }
        }

        subAck(id, mqMsg.getMsgId(), MQTTS_RC_ACCEPTED, mqMsg.getFlags());

        if (id == MQTTS_TOPICID_PREDEFINED_TIME){
            publishUnixTime();
        }

/*---------  SEARCHGW  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_SEARCHGW){
        printf("SEARCHGW recv\n");
        gwInfo(_gwId);

/*---------  CONNECT  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_CONNECT){
        printf("CONNECT recv\n");
        MqttsConnect mqMsg = MqttsConnect(_gatewayId);
        mqMsg.setFrame(recvMsg->getData(), recvMsg->getData()[0]);
        MQString id;
        id.readBuf(mqMsg.getClientId());

        if (mqMsg.getFlags() && MQTTS_FLAG_WILL){
            willTopicReq();
        }else{
            connAck(MQTTS_RC_ACCEPTED);
        }

/*---------  REGISTER  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_REGISTER){
        printf("REGISTER recv\n");
        MqttsRegister mqMsg = MqttsRegister();
        mqMsg.setFrame(recvMsg);

        uint16_t topicId = _topics.getTopicId(mqMsg.getTopicName());
        if (topicId){
            regAck(topicId, mqMsg.getMsgId(), 0);
        }else{
            MQString* mqStr = mqMsg.getTopicName()->create();
            _topics.addTopic(mqStr);
            _topics.setTopicId(mqStr, getNextTopicId());
            regAck(_topics.getTopicId(mqStr), mqMsg.getMsgId(), 0);
        }



/*---------  UNSUBSCRIBE  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_UNSUBSCRIBE){
        printf("UNSUBSCRIBE recv\n");
        MqttsUnsubscribe mqMsg = MqttsUnsubscribe();
        mqMsg.setFrame(recvMsg);
        unsubAck(mqMsg.getMsgId());

/*---------  DISCONNECT  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_DISCONNECT){
        printf("DISCONNECT recv\n");
        MqttsDisconnect mqMsg = MqttsDisconnect();
        //_nodeList->getZBNode(getRxRemoteAddress16())->setNodeStatus(MQTTS_DEVICE_DISCONNECTED);
        memcpy(mqMsg.getMsgBuff(), recvMsg->getData(), recvMsg->getData()[0]);
        disconnect();


/*---------  WILLMSG  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLMSG){
        printf("WILLMSG recv\n");
        MqttsWillMsg mqMsg = MqttsWillMsg();
        // ToDo  willmessage process
        connAck(MQTTS_RC_ACCEPTED);

/*---------  WILLTOPIC  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_WILLTOPIC){
        printf("WILLTOPIC recv\n");
        // ToDo WillTopic process
        willMsgReq();

/*---------  PINGREQ  ----------*/
    }else if (recvMsg->getData()[1] == MQTTS_TYPE_PINGREQ){
        printf("PINGREQ recv\n");
        pingResp();

    }else{
        *returnCode = MQTTS_ERR_NO_ERROR;
    }
}


#endif  /* ARDUINO */

/*========= End of File ==============*/






