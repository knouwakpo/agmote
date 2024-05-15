// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h

#define LOW 0
#define HIGH 1
//typedef int bool; // or #define bool int

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define RF69_SPI_CS             SS // SS is the SPI slave select pin, for instance D10 on ATmega328



#define CSMA_LIMIT            -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS  500
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

#define RFM69_WRITE_TIMER      1

// 
#define ISRFM69HW  1

// module interface, platform specific
extern void noInterrupts();                // function to disable interrupts
extern void interrupts();                  // function to enable interrupts
extern void RFM69_SetCSPin(bool);          // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
extern bool RFM69_ReadDIO0Pin(void);       // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
extern uint8_t SPI_transfer(uint8_t *, uint8_t nbytes);     // function to transfer 1byte on SPI with readback
uint8_t SPI_transferbytes(uint8_t *rq_buffer, uint8_t nbytes, uint8_t *rx_buffer);  // function to transfer n bytes on SPI with readback
extern void Serialprint(char*);            // function to print to serial port a string
extern bool Timeout_IsTimeout1(void);      // function for timeout handling, checks if previously set timeout expired
extern pthread_t Timeout_SetTimeout1(uint16_t); // function for timeout handling, sets a timeout, parameter is in milliseconds (ms)
extern void Cancel_Timeout(pthread_t thr);
uint32_t RFM69_getFrequency();
extern void RFM69_interruptHandler(void);    
typedef struct {
	uint8_t data[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
	uint8_t datalen;
	uint8_t senderID;
	uint8_t targetID;
	int16_t rssi;
	int timestamp;} rfm69frame_t ;
extern uint8_t RFM69_getFrame(rfm69frame_t*);
rfm69frame_t RFM69_Get_latestFrame(void);
bool isDataReady(void);

extern bool _haveData;

#endif

