#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <RFM69.h>
#include <RFM69Config.h>

#include <pigpio.h>

#include <unistd.h>
#include <oled.h>
#include <time.h>


#define NEOPIXEL_IO          4

#define PRESSCHAN            0
#define TEMPCHAN             1
#define WETCHAN              3

#define WETPINH             10
#define WETPINL             11
#define WETVOLTMIN        1.0f

#define TIPBUCKETPIN        12
#define TBCALIBRATION   0.254f

#define PUMPPIN              6
#define PUMPRUNMS        10000
#define PUMPCYCLEMS      60000
#define SEND_INTERVAL_MS 60000
#define DEBOUNCE_US        100

#define NRETRIES             3
#define SENDRETRYWAIT_MS    50





// extern functions


void RFM69_receiveBegin(void);
void RFM69_setNetwork(uint16_t);
void RFM69_setFrequency(uint32_t);
void RFM69_promiscuous(bool); 
void RFM69_setAddress(uint8_t);
void RFM69_setPowerLevel(uint8_t);
uint8_t RFM69_getFrame(rfm69frame_t*);
bool RFM69_receiveDone(void);
bool RFM69_ACKRequested();
void RFM69_sendACK(const void* buffer, uint8_t bufferSize) ;



bool RFM69_initialize(uint8_t freqBand, uint8_t nodeID, uint16_t networkID);
void RFM69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
bool RFM69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);

void RFM69_readAllRegs(void);
void RFM69_sleep(void);

void RFM69Config(int spifd_in);

bool isDataReady(void);


uint8_t tx_buffer[32];
uint8_t rx_buffer[32];
uint8_t oled_l1[32];
uint8_t oled_l2[32];

uint8_t NODEID;
uint8_t NETWORKID;


static volatile uint16_t done_counter = 0;

rfm69frame_t LastFrame;
struct tm* latest_time;

int main(int argc, char* argv[])
{
    if (argc < 3){
        printf(" Not enough arguments provided\n Usage: sudo ./rfm69app networkid nodeid\n networkid is 16-bit integer and nodeid 8-bit integer\n");
        return 1;
    }
    else{
        
        NETWORKID = atoi(argv[1]);
        NODEID = atoi(argv[2]);
    }
    
    initialize_oled();
    
    //Configure gpio
    //Controls sample rate for gpio used to check state and trigger callbacks
    gpioCfgClock(5, 0, 0); 
    //Initialize gpio
    if (gpioInitialise()<0) return 1;

    int spifd;
    uint32_t spiflag_ = 0;
    
    //Sets ux to 1 if CE line to be controlled by code
    //spiflag_ |= (1 << 6);
    
    spifd = spiOpen(1, 4000000, spiflag_);
    printf("Opened spi at spifd = %d\n", spifd);
    RFM69Config(spifd);

    sprintf(tx_buffer,"hello world");
    
    printf("Starting intialization\n");
    uint8_t init_val = RFM69_initialize(RF69_915MHZ, NODEID, NETWORKID);
    printf("Passed initialization\n");
    RFM69_setAddress(NODEID);
    RFM69_setNetwork(NETWORKID);
    RFM69_promiscuous(false);
    //Set the transmit power level (ranges from 0 to 31). 
    //Maximum works fine but at the expense of power consumption
    RFM69_setPowerLevel(31); 
    //RFM69_readAllRegs();
    RFM69_receiveBegin();
    
    
    
    while (1){
        LastFrame = RFM69_Get_latestFrame();
        sprintf(oled_l1, "Sndr %d rssi %d", 
        LastFrame.senderID, LastFrame.rssi);
        //sprintf(oled_l1, "Ahohh");
        latest_time = localtime((time_t *) &LastFrame.timestamp);
        //sprintf(oled_l2, "%s",asctime(latest_time));
        strftime(oled_l2, 32, "%m-%d-%y %H:%M", latest_time);
        //sprintf(oled_l2, "Ahaha");
        write_text_to_oled(oled_l1, oled_l2);
        oled_set_power(1);
        sleep(10);
        invert_oled_color();
        sleep(10);
        memset(oled_l1, 0, 32);
        memset(oled_l2, 0, 32);
        oled_set_power(0);
        sleep(10);
    }
    return 0;
}
