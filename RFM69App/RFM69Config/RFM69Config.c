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
#include <RFM69Config.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <pigpio.h>
#include <RFM69.h>
#include <string.h>
#include <stdlib.h>




volatile bool RFM_timeout;


// extern functions
void noInterrupts();                // function to disable interrupts
void interrupts();                  // function to enable interrupts
void RFM69_SetCSPin(bool);          // function to control the GPIO tied to RFM69 chip select (parameter HIGH or LOW)
bool RFM69_ReadDIO0Pin(void);       // function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
uint8_t SPI_transfer(uint8_t *, uint8_t nbytes);     // function to transfer 1byte on SPI with readback
uint8_t SPI_transferbytes(uint8_t *rq_buffer, uint8_t nbytes, uint8_t *rx_buffer); // function to transfer n bytes on SPI with readback
bool Timeout_IsTimeout1(void);      // function for timeout handling, checks if previously set timeout expired
pthread_t Timeout_SetTimeout1(uint16_t); // function for timeout handling, sets a timeout, parameter is in milliseconds (ms)
void Cancel_Timeout(pthread_t thr);
void RFM69_interruptHandler(void);
void interruptCallback(int,  int,  unsigned int);

void RFM69Config(int spifd_in);

int spifd;

bool _haveData = 0;

void initialize_rfm_spi(int spifd_in){
    spifd = spifd_in + 0;
}
void RFM69Config(int spifd_in){
    initialize_rfm_spi(spifd_in); //Initialize spi
    //gpioSetMode(RFM_CS_PIN, PI_OUTPUT); //set CS pin to output comment out since already handled by spi
    gpioSetMode(RFM_IO0_PIN, PI_INPUT); //set DIO0 pin to input
    gpioSetMode(RFM_RST_PIN,  PI_OUTPUT); //Set the reset pin to output
    
    //Reset the device

    gpioWrite(RFM_RST_PIN, 1);
    usleep(10000);
    gpioWrite(RFM_RST_PIN, 0);
    usleep(10000);
    
    
}

volatile bool canFire = 1;

void reset_can_fire(void){
    usleep(5);
    canFire = 1;
}

void interruptCallback(int gpio, int level, uint32_t tick){
    if ((gpio== RFM_IO0_PIN) & (level == RISING_EDGE)){
        RFM69_interruptHandler();
    }
}

void noInterrupts(){
    gpioSetAlertFunc(RFM_IO0_PIN, NULL);
    //gpioSetISRFunc(RFM_IO0_PIN, RISING_EDGE, 10, NULL);
}

void interrupts(){
    gpioNoiseFilter(RFM_IO0_PIN, 10, 50);
    gpioSetAlertFunc(RFM_IO0_PIN, interruptCallback);
    gpioSetWatchdog(RFM_IO0_PIN, 60000); //Forces an interruptCallback run after 1 min of inactivity on RFM_IO0_PIN
    //gpioSetISRFunc(RFM_IO0_PIN, RISING_EDGE, 10, interruptCallback);
}



void RFM69_SetCSPin(bool State_){
    //gpioWrite(RFM_CS_PIN, State_); //Comment out since spi hardware already dealing with it
}

bool RFM69_ReadDIO0Pin(void){
    return (bool) gpioRead(RFM_IO0_PIN);
}




uint8_t SPI_transfer(uint8_t *tx_buffer, uint8_t nbytes){
    uint8_t rx_buffer[nbytes];
    memset(rx_buffer,111,nbytes);
    spiXfer(spifd, tx_buffer, rx_buffer, nbytes);
    
    //printf("SPI txbuf =%d, %d, output is %d, %d of spifd %d, nbytes = %d\n", tx_buffer[0], tx_buffer[nbytes-1], rx_buffer[0], rx_buffer[nbytes-1], spifd, nbytes);

    return rx_buffer[nbytes-1];
}

uint8_t SPI_transferbytes(uint8_t *rq_buffer, uint8_t nbytes, uint8_t *rx_buffer){
    memset(rx_buffer,111,nbytes);
    spiXfer(spifd, rq_buffer, rx_buffer, nbytes);
    
    //printf("SPI txbuf =%d, %d, output is %d, %d of spifd %d, nbytes = %d\n", rq_buffer[0], rq_buffer[nbytes-1], rx_buffer[0], rx_buffer[nbytes-1], spifd, nbytes);
    return 0;
}

void Serialprint(char* print_buf){
    printf(print_buf);
    
}

void alarm_callback_function(){
    RFM_timeout = true;
    printf("Timeout occured\n");
}

void * alarm_callback(void *vargp) {
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    uint16_t timeout = *((uint16_t *)vargp);
    //printf("Starting timer for %d milliseconds\n", timeout);
    usleep((int)timeout * 1000);
    alarm_callback_function();
    free(vargp);
}


pthread_t  Timeout_SetTimeout1(uint16_t timeout_val){
    RFM_timeout = false;
    uint16_t *timeoutarg = malloc(sizeof(*timeoutarg));
    *timeoutarg = timeout_val;
    //printf("Setting timer for %d milliseconds\n", *timeoutarg);
    pthread_t thr;
    pthread_create(&thr, NULL, &alarm_callback, (void *) timeoutarg);
    return thr;
}

bool Timeout_IsTimeout1(void){
    
    return RFM_timeout;
}

void Cancel_Timeout(pthread_t thr){
    pthread_cancel(thr);
}

