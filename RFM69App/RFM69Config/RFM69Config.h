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
#ifndef RFM69Config_h
#define RFM69Config_h

//#define false 0
//#define true 1
//typedef int bool; // or #define bool int

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#define RFM_CS_PIN  7
#define RFM_IO0_PIN 22
#define RFM_RST_PIN 25
#define RFM_SPI_HW  "/dev/spidev0.1"


// module interface, platform specific
extern void noInterrupts();                // function to disable interrupts
extern void interrupts();                  // function to enable interrupts
extern void RFM69_SetCSPin(bool);          // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
extern bool RFM69_ReadDIO0Pin(void);       // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)

extern uint8_t SPI_transfer(uint8_t *, uint8_t nbytes);     // function to transfer 1byte on SPI with readback
uint8_t SPI_transferbytes(uint8_t *rq_buffer, uint8_t nbytes, uint8_t *rx_buffer); // function to transfer n bytes on SPI with readback
extern void Serialprint(char*);            // function to print to serial port a string
extern bool Timeout_IsTimeout1(void);      // function for timeout handling, checks if previously set timeout expired
extern pthread_t Timeout_SetTimeout1(uint16_t); // function for timeout handling, sets a timeout, parameter is in milliseconds (ms)
extern void Cancel_Timeout(pthread_t thr);
extern void RFM69_interruptHandler(void);
extern void RFM69Config(int spifd_in);
extern bool _haveData;
#endif

