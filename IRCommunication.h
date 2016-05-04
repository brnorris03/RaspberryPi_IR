/* Simple generic message communication interface. Each message has size+3 bytes.
 *  The first three bytes are 0x00, recipient ID (1 byte), sender ID (1 byte).
 */
#ifndef __IRCommunication_H
#define __IRCommunication_H

#include "Arduino.h"

// From RingoHardware.h
// ***************************************************
// IR
// ***************************************************

#define Chirp 9
#define _38kHz_Rx 3
#define IR_Enable_Front 13
#define IR_Enable_RearLeft 12
#define IR_Enable_RearRight 11
#define IR_Send 10

//The following was used as reference code in the development of the IR receive functions:
//http://playground.arduino.cc/Code/InfraredReceivers by Paul Malmsten
//https://github.com/z3t0/Arduino-IRremote by Ken Shirriff
extern void TxIR(unsigned char *Data, int Length);
extern void TxIRKey(byte key);
//
extern void RxIRStop(void);
extern void RxIRRestart(char BytesToLookFor);
extern char IsIRDone(void);
extern byte GetIRButton(void);
extern short IRNumOfBytes;
extern char CheckMenuButton(void);
//
extern byte GetIRButton(void);
extern const uint8_t IRRemoteButtons[][2];
#define IR_1 1
#define IR_2 2
#define IR_3 3
#define IR_4 4
#define IR_5 5
#define IR_6 6
#define IR_7 7
#define IR_8 8
#define IR_9 9
#define IR_0 10
#define IR_Forward 11
#define IR_Left 12
#define IR_Right 13
#define IR_Backward 14
#define IR_Power 15
#define IR_PlumLogo 16
#define IR_Menu 17
#define IR_A 18
#define IR_B 19
#define IR_Play 20
#define IR_X 21
//

//Low level functions:
extern int IRTransitionCount;
extern unsigned char IRBytes[20];
byte irData[]={0x00,0xFF,0x00,0x00};
extern char IRActive;
extern volatile char IRReceiving;//note: IRReceiving turned off if IsIRDone() in regular or auto NavigationHandler() and causes ReadSideSensors() to repeat
//
extern void EnableIROutputs(char Level);  //enables or disables the 3 IR outputs. Pass 0 to disable, 1 to enable all 3 IR outputs.
extern void ModulateIR(unsigned int Frequency, unsigned int OnTime); //ModulateIR(38000,5) seems to produce best square wave for 38kHz.
extern void PlayChirpIR(unsigned int Frequency, unsigned int OnTime); //wrapper for IRCarrierWave, bkwd compatibility. use ModulateIR() instead of PlayChirpIR going forward
//
extern void IRHandler(void);
//
// ***************************************************
// end IR
// ***************************************************

// ***************************************************
// Simple Timer
// ***************************************************
  //Ultra-simple stop watch functions using the built-in arduino millis() function.
  extern int32_t GetTime(void);
  extern void RestartTimer(void);
  extern void StopTimer(void);
// ***************************************************
// end Simple Timer
// ***************************************************

// end from RingoHardware.h


#define HEADER_SIZE 3


byte irmsg[20]; // max in RingoHardware is 20, including header

void ResetIR(short size);

// Broadcast messages should have 0 as the recipient

void SendIRMsg(byte sender, byte recipient, byte *msg, short size);

int ReceiveIRMsg(byte &sender, byte recipient, byte *msg, short size);

#endif
