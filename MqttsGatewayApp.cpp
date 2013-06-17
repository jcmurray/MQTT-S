/*
 * MqttsGatewayApp.cpp
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
 *  Created on: 2013/06/17
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.5.0
 *
 */
#ifdef ARDUINO
  #include <MQTTS_Defines.h>
  #include <MqttsGateway.h>
#else
  #include "MQTTS_Defines.h"
  #include "MqttsGateway.h"
#endif

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


int main(int argc, char **argv){

/*  Gateway  Test  */


     MqttsGateway gw = MqttsGateway();
     gw.begin(argv[1], B57600);
     gw.setDuration(60000);
     gw.init("Node-GW", 1);



     while(true){
         int rc = gw.execMsgRequest();
         if ((rc == MQTTS_ERR_NO_ERROR) &&gw.getMsgRequestCount() == 0  && (gw.getLoopCtrl() == MQTTS_TYPE_SUBACK)){
             gw.setLoopCtrl(0);
             break;
         }else if (rc){
             fprintf(stdout,"Rc = %d, TYPE = 0x%x\n", rc, gw.getMsgRequestType());
             if ( gw.getMsgRequestType() == MQTTS_TYPE_SEARCHGW){
                 sleep(5);
                 gw.setMsgRequestStatus(MQTTS_MSG_REQUEST);
             }else{
                 gw.clearMsgRequest();
             }
         }
     }
     /*
     MQString *topic = new MQString("a/bcd/ef");

         gw.registerTopic(topic, 16);

         while(true){
             int rc = gw.execMsgRequest();
             if (rc == MQTTS_ERR_NO_ERROR && gw.getMsgRequestCount() == 0){
                 break;
             }else{
                 fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, gw.getMsgRequestStatus());
                     gw.clearMsgRequest();
             }
     }
     */

     gw.execMsgRequest();
     printf("send publish\n");

     MQString *topic = new MQString("a/bcd/ef");
     const char* data = "123456";
     int length = 6;

     gw.publish(topic,data,length);

     while(true){
          int rc = gw.execMsgRequest();
          if (rc == MQTTS_ERR_NO_ERROR && gw.getMsgRequestCount() == 0 && gw.getLoopCtrl() == MQTTS_TYPE_PUBACK){
              continue;
          }else if (rc){
              fprintf(stdout,"Rc = %d, Request status = 0x%x\n", rc, gw.getMsgRequestStatus());
                  gw.clearMsgRequest();
          }
    }
     fprintf(stdout,"__end__\n");

}
