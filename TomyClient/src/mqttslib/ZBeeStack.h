/*
 * ZBeeStack.h
 *
 *
 *               Copyright (c) 2009 Andrew Rapp.       All rights reserved.
 *               Copyright (c) 2013 Tomoaki YAMAGUCHI  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 *     Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation 
 *     and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * You should have received a copy of the GNU General Public License
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * 
 *  Created on: 2013/06/17
 *    Modified: 2013/11/30
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.1.0
 *
 */

#ifndef ZBEESTACK_H_
#define ZBEESTACK_H_

#ifndef ARDUINO
    #include "MQTTS_Defines.h"
#endif

#if defined(ARDUINO)
	#if ARDUINO >= 100
		#include "Arduino.h"
		#include <inttypes.h>
	#else
		#if ARDUINO < 100
			#include "WProgram.h"
			#include <inttypes.h>
		#endif
	#endif

	#define ZB_RTSPIN  5

#endif /* ARDUINO */


#ifdef MBED
    #include "mbed.h"
    #define ZB_MBED_SERIAL_TXPIN  p13
    #define ZB_MBED_SERIAL_RXPIN  p14
#endif

#ifdef LINUX
    #include <sys/time.h>
    #include <iostream>
#endif



namespace tomyClient {

#define START_BYTE 0x7e
#define ESCAPE     0x7d
#define XON        0x11
#define XOFF       0x13

#define MAX_PAYLOAD_SIZE             70

#define ZB_API_REQUEST               0x10
#define ZB_API_RESPONSE              0x90

#define ZB_PACKET_ACKNOWLEGED        0x01
#define ZB_BROADCAST_PACKET          0x02
#define ZB_BROADCAST_RADIUS_MAX_HOPS 0
#define ZB_RSP_DATA_OFFSET           11
#define ZB_REQ_DATA_OFFSET           13

#define API_ID_POS                    3
#define PACKET_OVERHEAD_LENGTH        6
//#define TX_API_LENGTH  12

#define ZB_MAX_NODEID  20


/*====  STATUS ====== */
#define SUCCESS           0x0

#define NO_ERROR                          0
#define CHECKSUM_FAILURE                  1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH  2
#define UNEXPECTED_START_BYTE             3


#define PACKET_SENDED           0
#define PACKET_ERROR_RESPONSE  -1
#define PACKET_ERROR_UNKOWN    -2
#define PACKET_ERROR_NODATA    -3

enum RespWaitStatus {
	NoResp = 0,
	RxResp
};

enum SendReqType{
	NoReq= 0,
	UcastReq,
	BcastReq
};

/*
 *   MQTTS  Client's state
 */
enum NodeStatus {
	NdDisconnected = 0,
	NdActive,
	NdAsleep,
	NdAwaik,
	NdLost
};
/*
 *   Packet Read Constants
 */
#ifdef ARDUINO
#define PACKET_TIMEOUT_RESP   500
#define PACKET_TIMEOUT_CHECK  100
#else
#define PACKET_TIMEOUT_RESP   200
#define PACKET_TIMEOUT_CHECK   50
#endif


/*============================================
              XBeeAddress64
 =============================================*/
class XBeeAddress64 {
public:
  XBeeAddress64(uint32_t msb, uint32_t lsb);
  XBeeAddress64();
  uint32_t getMsb();
  uint32_t getLsb();
  void setMsb(uint32_t msb);
  void setLsb(uint32_t lsb);
private:
  uint32_t _msb;
  uint32_t _lsb;
};

 /*============================================
                ZBResponse
 =============================================*/

class ZBResponse {
public:
	ZBResponse();
	uint8_t getApiId();
	uint8_t getMsbLength();
	uint8_t getLsbLength();
	uint8_t getChecksum();
	uint8_t getFrameDataLength();
	uint16_t getPacketLength();
	uint16_t getRemoteAddress16();
	uint8_t  getPayload(uint8_t index);
	uint8_t* getPayload();
	uint8_t  getPayloadLength();
	uint8_t  getOption();
	XBeeAddress64& getRemoteAddress64();

	void setApiId(uint8_t api);
	void setMsbLength(uint8_t msbLength);
	void setLsbLength(uint8_t lsbLength);
	void setChecksum(uint8_t checksum);
	void setPayload(uint8_t* payloadPtr);
	void setPayloadLength(uint8_t payloadLength);
	void setRemoteAddress64(XBeeAddress64& addr64);
	void setRemoteAddress16(uint16_t addr16);
	void setOption(uint8_t options);

	bool isBrodcast();
	bool isAvailable();
	void setAvailable(bool complete);
	bool isError();
	uint8_t getErrorCode();
	void setErrorCode(uint8_t errorCode);
	void reset();

private:
	//void copyCommon(ZBResponse &target);

	uint8_t *_payloadPtr;
	uint8_t _msbLength;
	uint8_t _lsbLength;
	uint8_t _apiId;
	XBeeAddress64 _remoteAddress64;
	uint16_t _remoteAddress16;
	uint8_t  _options;
	uint8_t _checksum;
	uint8_t _payloadLength;
	bool   _complete;
	uint8_t _errorCode;


};

/*============================================*
                ZBRequest
 =============================================*/

class ZBRequest {
public:
	ZBRequest();
    ~ZBRequest(){};
//    uint8_t getFrameData(uint8_t pos);
    uint8_t getFrameDataLength();
	uint8_t getBroadcastRadius();
	uint8_t getOption();
	uint8_t* getPayload();
	uint8_t getPayloadLength();

	void setBroadcastRadius(uint8_t broadcastRadius);
	void setOption(uint8_t option);
	void setPayload(uint8_t *payload);
	void setPayloadLength(uint8_t payLoadLength);

private:
	uint8_t _broadcastRadius;
	uint8_t _option;
	uint8_t* _payloadPtr;
	uint8_t _payloadLength;
};


/*===========================================
                SerialPort
 ============================================*/

#ifdef ARDUINO
#include <Stream.h>
class SerialPort{
public:
	SerialPort( );
	void begin(long baudrate);
	bool send(unsigned char b);
	bool recv(unsigned char* b);
	void flush();

private:
	Stream* _serialDev;
};
#endif /* ARDUINO */

#ifdef MBED
/*-------------------------
    For MBED
 --------------------------*/
class SerialPort{
public:
	SerialPort( );
	void begin(long baudrate);
	bool send(unsigned char b);
	bool recv(unsigned char* b);
	void flush();

private:
        Serial* _serialDev;
};
#endif /* MBED */

#ifdef LINUX
/*-------------------------
    For Linux
 --------------------------*/
#include <termios.h>
class SerialPort{
public:
	SerialPort();
	~SerialPort();
	int begin(const char* devName);
	int begin(const char* devName,
			   unsigned int boaurate);
	int begin(const char* devName, unsigned int boaurate, bool parity);
	int begin(const char* devName, unsigned int boaurate,
				  bool parity, unsigned int stopbit);

	bool send(unsigned char b);
	bool recv(unsigned char* b);
void flush();

private:
	int _fd;  // file descriptor
	struct termios _tio;
};
#endif /* LINUX */



#ifdef ARDUINO
/*============================================
       XBeeTimer for Arduino
 ============================================*/
class XTimer {
public:
	XTimer();
	void start(uint32_t msec = 0);
	bool isTimeUp(uint32_t msec);
	bool isTimeUp(void);
	void stop();
private:
	uint32_t _startTime;
	uint32_t _currentTime;
	uint32_t _millis;
};
#endif

#ifdef MBED
/*============================================
    XBeeTimer  for MBED
 ============================================*/
class XTimer {
public:
    XTimer();
    void start(uint32_t msec = 0);
    bool isTimeUp(uint32_t msec);
    bool isTimeUp(void);
    void stop();
private:
    Timer    _timer;
    uint32_t _millis;
};

#endif

#ifdef LINUX
/*============================================
                XBeeTimer
 ============================================*/
class XTimer {
public:
    XTimer();
    void start(uint32_t msec = 0);
    bool isTimeUp(uint32_t msec);
    bool isTimeUp(void);
    void stop();
private:
    struct timeval _startTime;
    uint32_t _millis;
};
#endif


/*===========================================
               Class  ZBeeStack
 ============================================*/
class ZBeeStack {
public:
	ZBeeStack();
	~ZBeeStack();

	int  send(uint8_t* xmitData, uint8_t dataLen, uint8_t option, SendReqType type);
	int  readPacket();
	int  readResp();


	void setSerialPort(SerialPort *serialPort);
	void setGwAddress(XBeeAddress64& addr64, uint16_t addr16);
	void setRxHandler(void (*callbackPtr)(ZBResponse* data, int* returnCode));

	XBeeAddress64& getRxRemoteAddress64();
	uint16_t       getRxRemoteAddress16();
	const char*   getNodeId();

	void          getResponse(ZBResponse& response);
	ZBResponse*    getRxResponse();

	bool init(const char* nodeId);

private:
	void sendZBRequest(ZBRequest& request);
    int  packetHandle();
    void execCallback();
    void readApiFrame(void);
    bool readApiFrame(uint16_t timeoutMillsec);
    void flush();
    void resetResponse();
    bool read(uint8_t* buff);
    bool write(uint8_t val);
    void sendByte(uint8_t, bool escape);
    uint8_t getAddrByte(uint8_t pos);

	ZBRequest   _txRequest;
	ZBResponse  _rxResp;
	ZBRequest   _txRetryRequest;
	int         _returnCode;

	uint8_t _rxPayloadBuf[MAX_PAYLOAD_SIZE];
	uint8_t _respWaitStat;  // 0:no wait  1:TxResp

	NodeStatus _nodeStatus;
	SendReqType _sendReqStat;

	ZBResponse _response;    //  Received data

	uint8_t _pos;
	uint8_t _byteData;
	bool   _escape;
	uint8_t _checksumTotal;
	uint16_t _addr16;
	uint32_t _addr32;
	uint8_t _responsePayload[MAX_PAYLOAD_SIZE];
	SerialPort *_serialPort;
	XBeeAddress64 _gwAddress64;
	uint16_t  _gwAddress16;

	XTimer  _tm;

	void (*_rxCallbackPtr)(ZBResponse* data, int* returnCode);
};

}
#endif  /* ZBEESTACK_H_ */
