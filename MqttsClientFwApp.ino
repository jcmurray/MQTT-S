/*
 * MqttsClientFwApp.ino
 *
 *                      The MIT License (MIT)
 *
 *               Copyright (c) 2013 tomy-tech.com  All rights reserved.
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
 *
 *  Created on: 2013/06/08
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.3.0
 *
 */
 

#include <MQTTS_Defines.h>
#include <MqttsClientAppFw4Arduino.h>

#if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
    #include <SoftwareSerial.h>
    uint8_t ssRX = 8; // Connect Arduino pin 8 to TX of usb-serial device
    uint8_t ssTX = 9; // Connect Arduino pin 9 to RX of usb-serial device
    SoftwareSerial debug(ssRX, ssTX);
#endif

    MqttsClientApplication app = MqttsClientApplication();
    
    MQString* tp1 = new MQString("abc/def/g");
    MQString* tp2 = new MQString("efg/hi/j");
    
       int  cb1(MqttsPublish* msg){
          #ifdef MQTTS_DEBUG
          debug.println("exec callback");
          #endif
          return 0;
    }
    
    
void setup(){
  #if  defined(DEBUG_MQTTS) || defined(DEBUG_ZBEESTACK) 
  debug.begin(19200);
#endif

  app.begin(38400);
  app.init("Node-02");
    app.setQos(1);
    //app.setWillTopic(willtopic);
    //app.setWillMessage(willmsg);
    app.setKeepAlive(300000);
    
}

void loop(){    

    app.connect();
    app.runConnect();
   

    app.registerTopic(tp1);
    app.run();
    
    app.registerTopic(tp2);
    app.run();

    /*
    app.disconnect();
    app.run();
    */

    /*
    app.willTopic();
    app.run();
     */

    MQString *topic = new MQString("a/bcd/ef");

    app.subscribe(topic, cb1);
    app.run();

    app.publish(topic, "abcde", 5);
    app.run();
       
    /*
    app.unsubscribe(topic);
    app.run();
    */
    app.startWdt();
    app.runLoop();
        

}



