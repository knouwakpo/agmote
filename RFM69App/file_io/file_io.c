#include "RFM69.h"
#include <pigpio.h>
#include <string.h>
#include <file_io.h>
#include <unistd.h>
#include <stdio.h>


//External variables
extern char MAINDATABUFFER[1024];
extern volatile uint16_t MAINNBYTES;
extern rfm69frame_t latestframe; 
extern char datainfo[255];
extern volatile uint8_t targetID;
extern char datastream[255];
extern volatile int16_t rssi;
extern volatile bool dataReady;


static volatile bool changeFile = false;
static volatile bool filebusy = false;

void writeData(int event_, uint32_t tick_ );
void write_to_file(char *radiostream, uint8_t nvals);
void newFile(int event_, uint32_t tick_ );
bool isFileBusy(void);
void writeMainbuffer(void);





void writeData(int event_, uint32_t tick_ ){
  if(event_ == 2){
    datastream[0] = 0;
    datainfo[0] = 0;
    sprintf(datainfo, "rcvd:%d,Sender:%d,Rcvr:%d,rssi:%d,", 
        latestframe.timestamp, latestframe.senderID, 
        latestframe.targetID, latestframe.rssi);
    strcpy(datastream, datainfo); 
    strcat(datastream, (char *) latestframe.data);
    uint8_t nbytes;
    
    nbytes = strlen(datastream);
    
    
    //printf("Datastream has %d characters\n", nbytes);
    write_to_file(datastream, nbytes);
    dataReady = 0;
  }
}

bool isFileBusy(void){
  return filebusy;
}

void write_to_file(char *radiostream, uint8_t nvals){
    strcat(MAINDATABUFFER, radiostream);
    MAINNBYTES = MAINNBYTES + (uint16_t)nvals;
    writeMainbuffer();
    
}

void writeMainbuffer(void){
  //Cancel the timer that forces write in case no new data triggered write_to_file
  //This timer is started every time a #3 event is triggered
  // See newFile function below
  gpioSetTimerFunc(RFM69_WRITE_TIMER, 10000, NULL); 
  //If signal given to save temp file, do so
  if(changeFile){
    char tempf[50];
    for(uint8_t i = 0; i < 256; i++){
      sprintf(tempf, "/home/roccflume/.apps/radiodata.temp.%d.txt", i);
      if (access(tempf, F_OK) == 0) continue;
      break;
    }
    if (!filebusy){
      filebusy = true;
      if (access("/home/roccflume/.apps/radiodata.txt", F_OK) == 0){
        rename("/home/roccflume/.apps/radiodata.txt", tempf);
        changeFile = false;
      }
      filebusy = false;
    }
  }

  if (!filebusy){
    filebusy = true;
    char* fname = "/home/roccflume/.apps/radiodata.txt";
    FILE *fp;
    if (access(fname, F_OK) == 0) {
        fp = fopen( fname , "a" );
    } else {
        fp = fopen( fname , "w" );
    }
    fwrite(MAINDATABUFFER, 1, MAINNBYTES, fp);
    
    fclose(fp);
    filebusy = false;
    MAINDATABUFFER[0] = 0;
    MAINNBYTES = 0;
  }
  else{
    int now_sec;
    int now_usec;
    gpioTime(PI_TIME_ABSOLUTE, &now_sec, &now_usec);
    printf("File busy will write later %d\n", now_sec);
  }
}


  

void newFile(int event_, uint32_t tick_ ){
  if(event_ == 3){
    changeFile = true;
    //Start a timer to run writeMainbuffer after 60 seconds
    gpioSetTimerFunc(RFM69_WRITE_TIMER, 60000, writeMainbuffer);
  }
}

