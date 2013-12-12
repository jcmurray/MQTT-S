/*
 * ZBeeStack.cpp
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
 *
 *  Created on: 2013/06/29
 *    Modified: 2013/12/12
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */

#ifndef ARDUINO
        #include "MQTTS_Defines.h"
#else
        #include <MQTTS_Arduino_defs.h>
#endif


#ifdef ARDUINO
  #include <ZBeeStack.h>

  #if defined( XBEE_DEBUG) || defined(MQTT_DEBUG)
        #include <SoftwareSerial.h>
        extern SoftwareSerial debug;
  #endif

#endif  /* ARDUINO */

#ifdef MBED
        #include "mbed.h"
        #include "ZBeeStack.h"
#endif /* MBED */

#ifdef LINUX
        #include "ZBeeStack.h"
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

/*=====================================
        Global functions
 ======================================*/

uint16_t getUint16(uint8_t* pos){
  uint16_t val = ((uint16_t)*pos++ << 8);
  return val += *pos;
}

void setUint16(uint8_t* pos, uint16_t val){
    *pos++ = (val >> 8) & 0xff;
    *pos = val &0xff;
}

long getLong(uint8_t* pos){
    long val = (uint32_t(*(pos + 3)) << 24) +
        (uint32_t(*(pos + 2)) << 16) +
        (uint32_t(*(pos + 1)) <<  8);
        return val += *pos;
}

void setLong(uint8_t* pos, uint32_t val){
    *pos++ = (val >> 24) & 0xff;
    *pos++ = (val >> 16) & 0xff;
    *pos++ = (val >>  8) & 0xff;
    *pos   =  val & 0xff;
}



/*=========================================
             Class XBeeTimer
 =========================================*/

#ifdef ARDUINO
/**
 *   for Arduino
 */
XTimer::XTimer(){
    stop();
}

void XTimer::start(uint32_t msec){
    _startTime = millis();
    _millis = msec;
    _currentTime = 0;
}

bool XTimer::isTimeUp(){
    return isTimeUp(_millis);
}

bool XTimer::isTimeUp(uint32_t msec){
    if ( _startTime){
        _currentTime = millis();
        if ( _currentTime < _startTime){
            return (0xffffffff - _startTime + _currentTime > msec);
        }else{
            return (_currentTime - _startTime > msec);
        }
    }else{
        return false;
    }
}

void XTimer::stop(){
    _startTime = 0;
    _millis = 0;
}

#endif
#ifdef MBED
/**
 *   for MBED
 */
XTimer::XTimer(){
    stop();
}

void XTimer::start(uint32_t msec){
    _timer.start();
    _millis = msec;
}

bool XTimer::isTimeUp(){
    return isTimeUp(_millis);
}

bool XTimer::isTimeUp(uint32_t msec){
    return _timer.read_ms() > msec;
}

void XTimer::stop(){
    _timer.stop();
    _millis = 0;
}
#endif

#ifdef LINUX
/**
 *   for LINUX
 */
XTimer::XTimer(){
  stop();
}

void XTimer::start(uint32_t msec){
  gettimeofday(&_startTime, NULL);
  _millis = msec;
}

bool XTimer::isTimeUp(){
  return isTimeUp(_millis);
}

bool XTimer::isTimeUp(uint32_t msec){
    struct timeval curTime;
    long secs, usecs;
    if (_startTime.tv_sec == 0){
        return false;
    }else{
        gettimeofday(&curTime, NULL);
        secs  = (curTime.tv_sec  - _startTime.tv_sec) * 1000;
        usecs = (curTime.tv_usec - _startTime.tv_usec) / 1000.0;
        return ((secs + usecs) > (long)msec);
    }
}

void XTimer::stop(){
  _startTime.tv_sec = 0;
  _millis = 0;
}

#endif

/*=========================================
       Class SerialPort
 =========================================*/
#ifdef ARDUINO
/**
 *  For Arduino
 */
SerialPort::SerialPort(){
  _serialDev = NULL;
}

void SerialPort::begin(long baudrate){
  Serial.begin(baudrate);
  _serialDev = (Stream*) &Serial;
}

bool SerialPort::checkRecvBuf(){
	return _serialDev->available() > 0;
}

bool SerialPort::send(unsigned char b){
  if(_serialDev->write(b) != 1){
      return false;
  }else{
	  D_ZBSTACK(" 0x");
	  D_ZBSTACK(b,HEX);
      return true;
  }
}

bool SerialPort::recv(unsigned char* buf){
	if ( _serialDev->available() > 0 ){
		buf[0] = _serialDev->read();

		D_ZBSTACK(" 0x");
		D_ZBSTACK(*buf,HEX);
		return true;

	}else{
		return false;
	}
}

void SerialPort::flush(void){
    _serialDev->flush();
}

#endif /* ARDUINO */

#ifdef MBED
/**
 *  For MBED
 */
SerialPort::SerialPort(){
  _serialDev = new Serial(ZB_MBED_SERIAL_TXPIN, ZB_MBED_SERIAL_RXPIN);
}

void SerialPort::begin(long baudrate){
  _serialDev->baud(baudrate);
  _serialDev->format(8,Serial::None,1);
}

bool SerialPort::checkRecvBuf(){
	return _serialDev->readable() > 0;
}

bool SerialPort::send(unsigned char b){
  _serialDev->putc(b);
        D_ZBSTACKF( " 0x%x", b);
      return true;
}

bool SerialPort::recv(unsigned char* buf){
  if ( _serialDev->readable() > 0 ){
    buf[0] = _serialDev->getc();
    D_ZBSTACKF( " 0x%x",*buf );
    return true;
  }else{
    return false;
  }
}

void SerialPort::flush(void){
  //_serialDev->flush();
}

#endif /* MBED */

#ifdef LINUX

SerialPort::SerialPort(){
    _tio.c_iflag = IGNBRK | IGNPAR;
#ifdef XBEE_FLOWCTRL_CRTSCTS
    _tio.c_cflag = CS8 | CLOCAL | CREAD | CRTSCTS;
#else
    _tio.c_cflag = CS8 | CLOCAL | CREAD;
#endif
    _tio.c_cc[VINTR] = 0;
    _tio.c_cc[VTIME] = 0;
    _tio.c_cc[VMIN] = 0;
    _fd = 0;
}

SerialPort::~SerialPort(){
  if (_fd){
      close(_fd);
  }
}

int SerialPort::begin(const char* devName){
  return begin(devName, B9600, false, 1);
}


int SerialPort::begin(const char* devName, unsigned int boaurate){
  return begin(devName, boaurate, false, 1);
}

int SerialPort::begin(const char* devName, unsigned int boaurate, bool parity){
  return begin(devName, boaurate, parity, 1);
}

int SerialPort::begin(const char* devName, unsigned int boaurate,  bool parity, unsigned int stopbit){
  _fd = open(devName, O_RDWR | O_NOCTTY);
  if(_fd < 0){
      return _fd;
  }

  if (parity){
      _tio.c_cflag = _tio.c_cflag | PARENB;
  }
  if (stopbit == 2){
      _tio.c_cflag = _tio.c_cflag | CSTOPB ;
  }
  switch(boaurate){
    case B9600:
    case B19200:
    case B38400:
    case B57600:
    case B115200:
      if( cfsetspeed(&_tio, boaurate)<0){
        return errno;
      }
      break;
    default:
      return -1;
  }
    return tcsetattr(_fd, TCSANOW, &_tio);
}

bool SerialPort::checkRecvBuf(){
	return true;
}

bool SerialPort::send(unsigned char b){
  if (write(_fd, &b,1) != 1){
      return false;
  }else{
	  D_ZBSTACKF( " 0x%x", b);
      return true;
  }
}

bool SerialPort::recv(unsigned char* buf){
  if(read(_fd, buf, 1) == 0){
      return false;
  }else{
	  D_ZBSTACKF( " 0x%x",*buf );
      return true;
  }
}

void SerialPort::flush(void){
  tcsetattr(_fd, TCSAFLUSH, &_tio);
}

#endif

/*=========================================
             Class XBeeAddress64
 =========================================*/
XBeeAddress64::XBeeAddress64(){
    _msb = _lsb = 0;
}

XBeeAddress64::XBeeAddress64(uint32_t msb, uint32_t lsb){
	_msb = msb;
	_lsb = lsb;
}

uint32_t XBeeAddress64::getMsb(){
	return _msb;
}

uint32_t XBeeAddress64::getLsb(){
	return _lsb;
}

void XBeeAddress64::setMsb(uint32_t msb){
	_msb = msb;
}

void XBeeAddress64::setLsb(uint32_t lsb){
	_lsb = lsb;
}

/*=========================================
             Class ZBResponse
 =========================================*/
ZBResponse::ZBResponse(){
    reset();
}

uint8_t ZBResponse::getApiId(){
	return _apiId;
}

uint8_t ZBResponse::getMsbLength(){
	return _msbLength;
}

uint8_t ZBResponse::getLsbLength(){
	return _lsbLength;
}

uint8_t ZBResponse::getChecksum(){
	return _checksum;
}

uint8_t ZBResponse::getFrameDataLength(){
	return _payloadLength + ZB_RSP_DATA_OFFSET;
}

uint16_t ZBResponse::getPacketLength() {
	return ((_msbLength << 8) & 0xff) + (_lsbLength & 0xff);
}

void ZBResponse::setMsbLength(uint8_t msbLength){
	_msbLength = msbLength;
}

void ZBResponse::setLsbLength(uint8_t lsbLength){
	_lsbLength = lsbLength;
}

void ZBResponse::setChecksum(uint8_t checksum){
	_checksum = checksum;
}

void ZBResponse::setApiId(uint8_t api){
	_apiId = api;
}

void ZBResponse::setPayloadLength(uint8_t payloadLength){
	_payloadLength = payloadLength;
}

void ZBResponse::setPayload(uint8_t* payloadPtr){
	_payloadPtr = payloadPtr;
}

bool ZBResponse::isAvailable(){
	return _complete;
}

void ZBResponse::setAvailable(bool complete){
	_complete = complete;
}

bool ZBResponse::isError(){
    return _errorCode > 0;
}

uint8_t ZBResponse::getErrorCode(){
    return _errorCode;
}

void ZBResponse::setErrorCode(uint8_t errorCode){
	_errorCode = errorCode;
}


void ZBResponse::reset(){
	  _apiId = ZB_API_RESPONSE;
	  _msbLength = 0;
	  _lsbLength = 0;
	  _checksum = 0;
	  _payloadLength = 0;
	  _remoteAddress16 = 0;
	  _options = 0;
	  _errorCode = NO_ERROR;
	  _complete = false;
}

XBeeAddress64&  ZBResponse::getRemoteAddress64(){
	return _remoteAddress64;
}

uint16_t ZBResponse::getRemoteAddress16(){
  return _remoteAddress16;
}

void  ZBResponse::setRemoteAddress64(XBeeAddress64& addr64){
	_remoteAddress64 = addr64;
}

void  ZBResponse::setRemoteAddress16(uint16_t addr16){
	_remoteAddress16 = addr16;
}

uint8_t ZBResponse::getPayload(uint8_t index){
	return _payloadPtr[index];
}

uint8_t* ZBResponse::getPayload(){
	return _payloadPtr;
}

uint8_t ZBResponse::getPayloadLength(){
	return getFrameDataLength() - ZB_RSP_DATA_OFFSET;
}

uint8_t ZBResponse::getOption(){
	return _options;
}

void ZBResponse::setOption(uint8_t options){
	_options = options;
}

bool ZBResponse::isBrodcast(){
	return ( _options && 0x02);
}

/*=========================================
           Class ZBRequest
 =========================================*/

ZBRequest::ZBRequest(){

}

uint8_t ZBRequest::getFrameDataLength(){
	return  _payloadLength + ZB_REQ_DATA_OFFSET;
}

uint8_t* ZBRequest::getPayload(){
	return _payloadPtr;
}

uint8_t ZBRequest::getPayloadLength(){
	return _payloadLength;
}

uint8_t ZBRequest::getBroadcastRadius(){
	return _broadcastRadius;
}

uint8_t ZBRequest::getOption(){
	return _option;
}

void ZBRequest::setBroadcastRadius(uint8_t broadcastRadius){
	_broadcastRadius = broadcastRadius;
}

void ZBRequest::setOption(uint8_t option){
	_option = option;
}

void ZBRequest::setPayload(uint8_t *payload){
	_payloadPtr = payload;
}

void ZBRequest::setPayloadLength(uint8_t payLoadLength){
	_payloadLength = payLoadLength;
}


/*===========================================
              Class  ZBeeStack
 ============================================*/

ZBeeStack::ZBeeStack(){
	_rxCallbackPtr = NULL;
	_returnCode = 0;
	_response = ZBResponse();
	_pos = 0;
	_escape = false;
	_checksumTotal = 0;
	_response.setPayload(_responsePayload);
	_serialPort = 0;
	_gwAddress64.setMsb(0);
	_gwAddress64.setLsb(0);
	_gwAddress16 = 0;
	_tm.stop();
}

ZBeeStack::~ZBeeStack(){

}

bool ZBeeStack::init(const char* nodeId){
	_nodeStatus = NdDisconnected;

	  int len = strlen(nodeId);
	  if (len <= ZB_MAX_NODEID ){
		  uint8_t node[len + 1];
		  strcpy((char*)node, nodeId);
		  return true;
	  }else{
		  return false;
	  }
	return true;
}

void ZBeeStack::setRxHandler(void (*callbackPtr)(ZBResponse* data, int* returnCode)){
	_rxCallbackPtr = callbackPtr;
}

XBeeAddress64& ZBeeStack::getRxRemoteAddress64(){
	return _rxResp.getRemoteAddress64();
}

uint16_t ZBeeStack::getRxRemoteAddress16(){
	return _rxResp.getRemoteAddress16();
}

ZBResponse* ZBeeStack::getRxResponse(){
	return &_rxResp;
}


void ZBeeStack::getResponse(ZBResponse& response){
	response.setMsbLength(_response.getMsbLength());
	response.setLsbLength(_response.getLsbLength());
	response.setApiId(_response.getApiId());
	response.setPayloadLength(_response.getPayloadLength());
	response.setPayload(_response.getPayload());
	response.setOption(_response.getOption());
	response.setRemoteAddress16(_response.getRemoteAddress16());
	response.getRemoteAddress64().setMsb(_response.getRemoteAddress64().getMsb());
	response.getRemoteAddress64().setLsb(_response.getRemoteAddress64().getLsb());
}


void ZBeeStack::setGwAddress(XBeeAddress64& addr64, uint16_t addr16){
	_gwAddress64.setMsb(addr64.getMsb());
	_gwAddress64.setLsb(addr64.getLsb());
	_gwAddress16 = addr16;
}

void ZBeeStack::setSerialPort(SerialPort *serialPort){
  _serialPort = serialPort;
}


void ZBeeStack::send(uint8_t* payload, uint8_t payloadLen, uint8_t option, SendReqType type ){
	_txRequest.setOption(option);
	_txRequest.setPayload(payload);
	_txRequest.setPayloadLength(payloadLen);
	sendZBRequest(_txRequest, type);
}

int ZBeeStack::readPacket(){
	if(_serialPort->checkRecvBuf()){
		return readResp();
	}
	return PACKET_ERROR_NODATA;
}

int ZBeeStack::readResp(){
	_returnCode = PACKET_ERROR_NODATA;

	if(readApiFrame(PACKET_TIMEOUT_CHECK)){
		if(_response.getApiId() == ZB_API_RESPONSE){
			if (!_response.isError()){
				getResponse(_rxResp);
				for ( int i = 0; i < _rxResp.getPayloadLength(); i++){
					_rxPayloadBuf[i] = _rxResp.getPayload()[i];
				}
				_rxResp.setPayload(_rxPayloadBuf);
				if (_rxCallbackPtr != NULL){
					_rxCallbackPtr(&_rxResp, &_returnCode);
				}
			}
		}
	}
	return _returnCode;
}


bool ZBeeStack::readApiFrame(uint16_t timeoutMillsec){
	_pos = 0;
    _tm.start((uint32_t)timeoutMillsec);


    while(!_tm.isTimeUp()){

        readApiFrame();

        if(_response.isAvailable()){
        	ZB_RTS_OFF();
        	D_ZBSTACKW("\r\n<=== CheckSum OK\r\n\n");
            if( (_response.getOption() & 0x02 ) == 0x02 ){ //  broadcast ?
            	return true;
            }else if(_gwAddress16 &&
				(_gwAddress64.getMsb() != _response.getRemoteAddress64().getMsb()) &&
				(_gwAddress64.getLsb() != _response.getRemoteAddress64().getLsb())){
            	D_ZBSTACKW("  Sender is not Gateway!\r\n" );
            	return false;
            }else{
                return true;
            }
        }else if(_response.isError()){
        	D_ZBSTACKW("\r\n<=== Packet Error Code = ");
        	D_ZBSTACKLN(_response.getErrorCode(), DEC);
        	D_ZBSTACKF("%d\r\n",_response.getErrorCode() );
            return false;
        }
    }
    return false;   //Timeout
}

void ZBeeStack::readApiFrame(){

	if (_response.isAvailable() || _response.isError()){
	  resetResponse();
	}

	while(read(&_byteData )){

		if( _byteData == START_BYTE){
			_pos = 1;
			continue;
		}
		// Check ESC
		if(_pos > 0 && _byteData == ESCAPE){
		  if(read(&_byteData )){
			  _byteData = 0x20 ^ _byteData;  // decode
		  }else{
			  _escape = true;
			  continue;
		  }
		}

		if(_escape){
			_byteData = 0x20 ^ _byteData;
			_escape = false;
		}

		if(_pos >= API_ID_POS){
			_checksumTotal+= _byteData;
		}

		switch(_pos){
		case 0:
			break;

		case 1:
			_response.setMsbLength(_byteData);
			break;

		case 2:
			_response.setLsbLength(_byteData);
			D_ZBSTACKW("\r\n===> Recv:    ");
			break;
		case 3:
			_response.setApiId(_byteData);   // API
			break;

		case 4:
			_addr32 = (uint32_t(_byteData) << 24);
			break;
		case 5:
			_addr32 += (uint32_t(_byteData) << 16);
			break;

		case 6:
			_addr32 += (uint32_t(_byteData) << 8);
			break;

		case 7:
			_addr32 += _byteData;
			_response.getRemoteAddress64().setMsb(_addr32);
			break;

		case 8:
			_addr32 = (uint32_t(_byteData) << 24);
			break;

		case 9:
			_addr32 += (uint32_t(_byteData) << 16);
			break;

		case 10:
			_addr32 += (uint32_t(_byteData) << 8);
			break;

		case 11:
			_addr32 += _byteData;
			_response.getRemoteAddress64().setLsb(_addr32);
			break;

		case 12:
			_addr16 = (uint16_t(_byteData) << 8);
			break;

		case 13:
			_addr16 += _byteData;
			_response.setRemoteAddress16(_addr16);
			break;

		case 14:
			_response.setOption(_byteData);
			D_ZBSTACKW( "\r\n     Payload: ");
			break;

		default:
			if(_pos > MAX_PAYLOAD_SIZE){
			  _response.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH);
			  _pos = 0;
			  return;
			}else if(_pos == (_response.getPacketLength() + 3)){  // 3 = 2(packet len) + 1(checksum)
			  if((_checksumTotal & 0xff) == 0xff){
				  _response.setChecksum(_byteData);
				  _response.setAvailable(true);
				  _response.setErrorCode(NO_ERROR);
			  }else{
				  _response.setErrorCode(CHECKSUM_FAILURE);
			  }
			  _response.setPayloadLength(_pos - 4 - ZB_RSP_DATA_OFFSET);    // 4 = 2(packet len) + 1(Api) + 1(checksum)
			  _pos = 0;
			  _checksumTotal = 0;
			  return;
			}else{
			  _response.getPayload()[_pos - 4 - ZB_RSP_DATA_OFFSET] = _byteData;
			}
			break;
		}
		_pos++;

	}
}

void ZBeeStack::sendZBRequest(ZBRequest& request, SendReqType type){
	D_ZBSTACKW("\r\n===> Send:    ");

	sendByte(START_BYTE, false);           // Start byte

	uint8_t msbLen = ((request.getFrameDataLength() + 1) >> 8) & 0xff; // 1  for Checksum
	uint8_t lsbLen = (request.getFrameDataLength() + 1) & 0xff;
	sendByte(msbLen, true);               // Message Length
	sendByte(lsbLen, true);               // Message Length

	sendByte(ZB_API_REQUEST, true);       // API
	uint8_t checksum = 0;
	checksum+= ZB_API_REQUEST;

	sendByte(0x00, true);                 // Frame ID

	for(int i = 0; i < 10; i++){
		sendByte(getAddrByte(i,type), true);   // Gateway Address 64 & 16
		checksum += getAddrByte(i,type);       //   or Broadcast Address
	}

	sendByte(request.getBroadcastRadius(), true);
	checksum += request.getBroadcastRadius();

	sendByte(request.getOption(), true);
	checksum += request.getOption();

	D_ZBSTACKW("\r\n     Payload: ");

	for( int i = 0; i < request.getPayloadLength(); i++ ){
	  	sendByte(request.getPayload()[i], true);     // Payload
	  	checksum+= request.getPayload()[i];
	}
	checksum = 0xff - checksum;
	sendByte(checksum, true);

	//flush();  // clear receive buffer

	D_ZBSTACKW("\r\n<=== Send completed\r\n\n" );
}

void ZBeeStack::sendByte(uint8_t b, bool escape){
  if(escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)){
      write(ESCAPE);
      write(b ^ 0x20);
  }else{
      write(b);
  }
}

void ZBeeStack::resetResponse(){
  _pos = 0;
  _escape = 0;
  _response.reset();
  _addr16 = 0;
  _addr32 = 0;

}

void ZBeeStack::flush(){
  _serialPort->flush();
}

bool ZBeeStack::write(uint8_t val){
  return (_serialPort->send(val) ? true : false );
}

bool ZBeeStack::read(uint8_t *buff){
        return  _serialPort->recv(buff);
}

uint8_t ZBeeStack::getAddrByte(uint8_t pos, SendReqType type){
	if(type == UcastReq){
		if (pos == 0){
			return (_gwAddress64.getMsb() >> 24) & 0xff;
		}else if (pos == 1){
			return (_gwAddress64.getMsb() >> 16) & 0xff;
		}else if (pos == 2){
			return (_gwAddress64.getMsb() >> 8) & 0xff;
		}else if (pos == 3){
			return (_gwAddress64.getMsb()) & 0xff;
		}else if (pos == 4){
			return (_gwAddress64.getLsb() >> 24) & 0xff;
		}else if (pos == 5){
			return (_gwAddress64.getLsb() >> 16) & 0xff;
		}else if (pos == 6){
			return (_gwAddress64.getLsb() >> 8) & 0xff;
		}else if (pos == 7){
			return (_gwAddress64.getLsb()) & 0xff;
		}else if (pos == 8){
			return (_gwAddress16 >> 8) & 0xff;
		}else if (pos == 9){
			return _gwAddress16 & 0xff;
		}
	}else if(type == BcastReq){
		if((pos >= 0) && (pos < 6)){
			return 0x00;
		}else if((pos > 5) && (pos < 9)){
			return 0xff;
		}else if(pos == 9){
			return 0xfe;
		}
	}
	return 0;
}





