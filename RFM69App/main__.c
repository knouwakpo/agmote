#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <RFM69.h>
#include <RFM69Config.h>
#include <spidev_lib.h>


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
void Serialprint(char*);            // function to print to serial port a string
void RFM69_receiveBegin(void);
void RFM69_setNetwork(uint16_t);
void RFM69_setFrequency(uint32_t);
void RFM69_promiscuous(bool); 
void RFM69_setAddress(uint8_t);
void RFM69_setPowerLevel(uint8_t);

void alarm_callback_function(void);


bool RFM69_initialize(uint8_t freqBand, uint8_t nodeID, uint16_t networkID);
void RFM69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
bool RFM69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);

void RFM69_readAllRegs(void);
void RFM69_sleep(void);
void gpio_callback(uint gpio, uint32_t event_mask);

double get_wetness_volt(struct Adafruit_ADS1015* ads);
bool timer_callback(repeating_timer_t *rt);

void TB_gpio_callback(uint gpio, uint32_t event_mask);
int64_t debounceTB(alarm_id_t id, void *user_data);

volatile bool SEND_DATA_FLAG = false;
volatile double TBdepth;

typedef struct Pump {
    bool status;
    bool ready;
    uint8_t pin;
} Pump_t;

double get_wetness_volt(struct Adafruit_ADS1015* ads){
    double wetness;
    //Reset
    gpio_put(WETPINH, 0);
    gpio_put(WETPINL, 0);
    //Forward
    gpio_put(WETPINH, 1);
    sleep_ms(100);
    wetness = Adafruit_ADS1015_get_SingleEnded_voltage(ads, WETCHAN);
    //Reverse to balance average charge across electrods over time 
    //This reduces electromigration of charged particles
    gpio_put(WETPINH, 0);
    gpio_put(WETPINL, 1);
    sleep_ms(100);
    Adafruit_ADS1015_get_SingleEnded_voltage(ads, WETCHAN);
    //Reset
    gpio_put(WETPINH, 0);
    gpio_put(WETPINL, 0);
    
    return wetness;
}

int64_t pump_off_callback(alarm_id_t id, void *user_data) {
    //Turns off the pump;
    Pump_t *pump = (Pump_t *)user_data;
    //printf("Pump off alarm went off\n");
    gpio_put(pump->pin, 0);
    pump->status = false;
    return 0;
}

int64_t new_pump_cycle_callback(alarm_id_t id, void *user_data) {
    //Turns off the pump;
    Pump_t *pump = (Pump_t *)user_data;
    pump->ready = true;
    return 0;
}

void start_pump(Pump_t *pump){
    if (pump->ready){
        if(!pump->status){
            //Sets the flag to false which will be reset on by the new_pump_cycle callback
            pump->ready = false; 
            gpio_put(pump->pin, 1);
            pump->status = true;
            add_alarm_in_ms(PUMPRUNMS, pump_off_callback, pump, false);
            add_alarm_in_ms(PUMPCYCLEMS, new_pump_cycle_callback, pump, false);
        }
    }
}

void core1_entry() {
    Pump_t mypump;
    mypump.pin = PUMPPIN;
    mypump.status = false;
    mypump.ready = true;
    gpio_init(mypump.pin);
    gpio_set_dir(mypump.pin, GPIO_OUT);
    gpio_set_drive_strength(mypump.pin, GPIO_DRIVE_STRENGTH_12MA);
    bi_decl(bi_1pin_with_name(PUMPPIN, "Pump control pin"));
    gpio_put(mypump.pin, 0);
    multicore_fifo_push_blocking(0);

    uint32_t pump_flag = 0;
    //char received_buf[50];
    printf("Running core 1!\n");

    while (1){
        sleep_ms(2);
        if(multicore_fifo_rvalid()){
            pump_flag = multicore_fifo_pop_blocking();
        }
        multicore_fifo_push_blocking(mypump.status);
        //sprintf(received_buf, "Received pump flag= %d, pump_status is %d pumpready is %d \n",
        // pump_flag, mypump.status, mypump.ready);
        // printf(received_buf);
        if (pump_flag == 1){
            //Reset the pump flag in the main code on core 0
            //printf("Starting pump on core 1!\n");
            start_pump(&mypump);
        }
    }
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
 
    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);
 
    // Can't measure clk_ref / xosc as it is the ref
}

int main()
{
    stdio_init_all();
    multicore_reset_core1();
    sleep_ms(1000);
    
    multicore_launch_core1(core1_entry);
    
    sleep_ms(1000);
    //Check if reboot caused by watchdog
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    } else {
        printf("Clean boot\n");
    }
    
    clock_stop(clk_adc);
    
    measure_freqs();
    
    //Enable watchdog with reboot in 4 seconds if no update called
    watchdog_enable(4000, 1);
    
    
    
    //Reset all i2c devices that respond to general call command 0x6h
    //including adx1x15 devices
    uint8_t resetcommand[1] = {0x06};
    //Send reset command with a 100 us timeout so it is not blocking
    i2c_write_timeout_us(ADS1015_I2C_INSTANCE, 0x00, resetcommand, 1, false, 100);
    struct Adafruit_ADS1015 ads;
    Adafruit_ADS1015_begin(&ads, ADS1115);
    ads.m_gain = GAIN_TWO;

    //Update watchdog throughout initialization
    watchdog_update();
    
    NODEID = get_nodeid(NODETABLE, PLOTID);
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    
    
    
    gpio_init(RFM_CS_PIN);
    gpio_set_dir(RFM_CS_PIN, GPIO_OUT);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(RFM_CS_PIN, "RFM SPI CS"));

    gpio_init(RFM_IO0_PIN);
    gpio_set_dir(RFM_IO0_PIN, GPIO_IN);
    // Make the RFM_IO0_PIN pin available to picotool
    bi_decl(bi_1pin_with_name(RFM_IO0_PIN, "RFM IO0 INTERRUPT"));

    gpio_init(NEOPIXEL_IO);
    gpio_set_dir(NEOPIXEL_IO, GPIO_OUT);
    bi_decl(bi_1pin_with_name(NEOPIXEL_IO, "NEOPIXEL IO"));

    //Setup wetness pins as outputs
    gpio_init(WETPINH);
    gpio_set_dir(WETPINH, GPIO_OUT);
    gpio_set_drive_strength(WETPINH, GPIO_DRIVE_STRENGTH_2MA);
    bi_decl(bi_1pin_with_name(WETPINH, "Wetness pin H"));
    gpio_put(WETPINH, 0);
    
    gpio_init(WETPINL);
    gpio_set_dir(WETPINL, GPIO_OUT);
    gpio_set_drive_strength(WETPINL, GPIO_DRIVE_STRENGTH_2MA);
    bi_decl(bi_1pin_with_name(WETPINL, "Wetness pin L"));
    gpio_put(WETPINL, 0);
    
    //Setup tipping bucket pin as input
    gpio_init(TIPBUCKETPIN);
    gpio_set_dir(TIPBUCKETPIN, GPIO_IN);
    //Pull up 
    gpio_pull_up(TIPBUCKETPIN);
    //Enable interrupt for TIPBUCKETPIN
    gpio_set_irq_enabled(TIPBUCKETPIN, GPIO_IRQ_EDGE_FALL, true);
    //gpio_add_raw_irq_handler(TIPBUCKETPIN, &TB_gpio_callback);
    
    //Update watchdog throughout initialization
    watchdog_update();

    //Setup SPI for RFM69 Enable SPI 1 at 1 MHz and connect to GPIOs
    spi_init(RFM_SPI_HW, 1000 * 1000);
    spi_set_format(RFM_SPI_HW, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // set SPI CPOL= 0 and CPHA = 0 ( in Motorola/Freescale nomenclature), MSB first
    gpio_set_function(RFM_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(RFM_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(RFM_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(RFM_SPI_RX_PIN, RFM_SPI_TX_PIN, RFM_SPI_SCK_PIN, GPIO_FUNC_SPI));
    printf("SPI initialised, let's goooooo\n");
    
    //Update watchdog throughout initialization
    watchdog_update();

    //Set interrupt handler
    //For RFM
    gpio_set_irq_enabled_with_callback(RFM_IO0_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
    uint8_t init_val = RFM69_initialize(RF69_915MHZ, NODEID, NETWORKID);
    RFM69_setAddress(NODEID);
    RFM69_setNetwork(NETWORKID);
    RFM69_promiscuous(false);
    //Set the transmit power level (ranges from 0 to 31). 
    //Maximum works fine but at the expense of power consumption
    RFM69_setPowerLevel(31); 
    
    //Update watchdog throughout initialization
    watchdog_update();
    
    
    char databuffer[250];
    
            
    uint8_t datalen;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    //Send timer
    repeating_timer_t sendtimer;
    //Repeating timer every minute (60000ms)
    if (!add_repeating_timer_ms(SEND_INTERVAL_MS, timer_callback, NULL, &sendtimer)) {
        printf("Failed to add timer\n");
        return 1;
    }
    uint32_t cur_freq;
    
    uint16_t cnt = 0; 
    uint16_t tempadc;
    double temp_volt, temp_deg, pressure_volt, pressure_mm, wetvolt, cur_tb_depth;
    temp_volt = Adafruit_ADS1015_get_SingleEnded_voltage(&ads, TEMPCHAN);
    sleep_us(500);
    pressure_volt = Adafruit_ADS1015_get_SingleEnded_voltage(&ads, PRESSCHAN);
    sleep_us(500);
    wetvolt = get_wetness_volt(&ads);
    TBdepth = 0.0f;
    cur_tb_depth = 0.0f;
    bool pump_status;
    bool send_response;
    
    //Update watchdog throughout initialization
    watchdog_update();
    
    while (true) {
        send_response = false;
        //printf("Hello world\nThis is RadioFruit\n");
        
        //Do not measure anything if pump is running
        pump_status = (bool) multicore_fifo_pop_blocking();
        if (pump_status){
            continue;
        }
        temp_volt = 0.5*temp_volt + 0.5*Adafruit_ADS1015_get_SingleEnded_voltage(&ads, TEMPCHAN);
        sleep_us(500);
        pressure_volt = 0.5*pressure_volt + 0.5*Adafruit_ADS1015_get_SingleEnded_voltage(&ads, PRESSCHAN);
        sleep_us(500);
        wetvolt = 0.5*wetvolt + 0.5*get_wetness_volt(&ads);
        
        if (wetvolt <= WETVOLTMIN){
            multicore_fifo_push_blocking(1);
        }
        else{
            multicore_fifo_push_blocking(0);
        }
        
        //printf(databuffer);
        //RFM69_readAllRegs();
        //Send message to rfm69 with ACK request
        if(SEND_DATA_FLAG){
            SEND_DATA_FLAG = false;
            cnt++;
            temp_deg = dsp_lookup_f(bths_table, temp_volt, BTH_TABLE_SIZE);
            pressure_mm = get_water_depth(pressure_volt);
            cur_tb_depth = TBdepth + 0.0f;
            TBdepth = 0; //Reset TBdepth to 0 quickly
            datalen = sprintf(databuffer, "T,%.1f,P,%.1f,Wv,%.3f,TB,%.3f", 
            temp_deg, pressure_mm, wetvolt, cur_tb_depth);
            printf(databuffer);
            datalen = (uint8_t) strlen(databuffer);


            if ((wetvolt < WETVOLTMIN) | (cur_tb_depth > 0.0f)){
                //Only send if wetvolt < threshold or tipping bucket counts >0
                //datalen = (uint8_t) strlen(databuffer);
                //RFM69_send(TONODEID, databuffer, datalen+1, true);
                send_response = RFM69_sendWithRetry(TONODEID, databuffer,
                datalen+1, NRETRIES, SENDRETRYWAIT_MS);
                cnt = 0;
            }
            if (cnt >= 30){
                //Send if the counter reaches 30 i.e. after 30 minutes
                //if wetvolt >= wetvoltmin TBCount == 0
                //datalen = (uint8_t) strlen(databuffer);
                //RFM69_send(TONODEID, databuffer, datalen+1, true);
                send_response = RFM69_sendWithRetry(TONODEID, databuffer,
                datalen+1, NRETRIES, SENDRETRYWAIT_MS);
                cnt = 0;
            }

                       
        }
        //Comment out receive if not expecting any message
        RFM69_receiveBegin();
        RFM69_sleep();
        if(send_response){
            printf("Sent message successfully\n");
        }
        
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        //Update the watchdog so reboot does not occur
        watchdog_update();
        sleep_ms(1000);
    }
}


bool timer_callback(repeating_timer_t *rt) {
    SEND_DATA_FLAG = true;
    return true; // keep repeating
}

//Tipping bucket gpio callback
void TB_gpio_callback(uint gpio, uint32_t event_mask){
    TBdepth = TBdepth + TBCALIBRATION;
}

int64_t debounceTB(alarm_id_t id, void *user_data){
    gpio_set_irq_enabled(TIPBUCKETPIN, GPIO_IRQ_EDGE_FALL,true);
    return 0;
}

void gpio_callback(uint gpio, uint32_t event_mask){
    if ((gpio == RFM_IO0_PIN) & ((event_mask & GPIO_IRQ_EDGE_RISE) != 0)){
        gpio_acknowledge_irq(RFM_IO0_PIN, GPIO_IRQ_EDGE_RISE);
        RFM69_interruptHandler();
    }
    if ((gpio == TIPBUCKETPIN) & ((event_mask & GPIO_IRQ_EDGE_FALL) != 0)){
        gpio_acknowledge_irq(TIPBUCKETPIN, GPIO_IRQ_EDGE_FALL);
        gpio_set_irq_enabled(TIPBUCKETPIN, GPIO_IRQ_EDGE_FALL,false);
        add_alarm_in_us(DEBOUNCE_US, debounceTB, NULL, false);
        TBdepth = TBdepth + TBCALIBRATION;
    }

}
