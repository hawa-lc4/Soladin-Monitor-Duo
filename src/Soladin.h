/*
  Soladin.h  - Library for comunicating to a Soladin 600.
  Created by   Teding,  November , 2011.
  Released into the public domain.
*/

#ifndef Soladin_h
#define Soladin_h

// commands
#define PRB 0xC1   // probe
#define FWI 0xB4   // firmware info
#define DVS 0xB6   // current data
#define HSD 0x9A   // history data
#define RMP 0xB9   // read max. power
#define ZMP 0x97   // reset max. power to zero

// adress
#define null 0x00
#define desti 0x11

// action
#define Ac_zmp 0x01

#define TimeOut 100  // 100 ms
#define dly 50   // 50 ms  give me time to push some buttons

#include "Arduino.h"
#include <SoftwareSerial.h>


class Soladin
{
  public:
    void begin(SoftwareSerial *theSerial);
    Soladin();       // constructer     
    bool query(uint8_t Cmdo);
    bool query(uint8_t Cmdo,uint8_t _day);  // overload
//  variable from devstate call	
    uint16_t Flag ;
    uint16_t PVvolt ;
    uint16_t PVamp ;
    uint16_t Gridfreq ;
    uint16_t Gridvolt ;
    uint16_t Gridpower ;
    uint32_t Totalpower ;
    uint8_t DeviceTemp ;
    uint32_t TotalOperaTime ; 
//  variable from Firware info call	
    uint8_t FW_ID;
    uint16_t FW_version;
    uint16_t FW_date;
//  variable from Max power call	
    uint16_t MaxPower;
//  variable from historical data call	
    uint8_t DailyOpTm ;
    uint16_t Gridoutput ;
//  communication status	
    char RxBuf[40];   
    uint8_t RxLgth ;
	uint16_t RxError ;
  
    
  private:
    SoftwareSerial *_serial;
    void conCat(uint8_t _cmd, uint8_t _act, char *OutBuf);
    void sndBuf(int size, char *OutBuf);
	bool txRx(uint8_t Cmdo);
    void DS_deCode(char *InBuf);
    void FW_deCode(char *InBuf);  
    void MP_deCode(char *InBuf);  
    void HD_deCode(char *InBuf);     
    int PolRxBuf(uint8_t Cmd);
    char TxBuf[9];


   
};

#endif
