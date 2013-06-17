/*
 * MqttsGatewayApp.cpp
 *
 *                        The MIT License (MIT)
 *
 *               Copyright (c) 2013, tomy-tech.com
 *                       All rights reserved.
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
