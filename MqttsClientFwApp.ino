/*
 * MqttsClientFwApp.ino
 *
 *                      The MIT License (MIT)
 *
 *               Copyright (c) 2013 tomy-tech.com  All rights reserved.
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



