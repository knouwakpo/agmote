//Written by Kossi Nouwakpo 2024-01-15
//Handles some file io operations
//Used by RFM69 radio


#ifndef FILE_IO_H
#define FILE_IO_H



//External variables
extern volatile uint8_t targetID;                // should match _address
extern volatile bool dataReady;
extern char MAINDATABUFFER[1024];
extern volatile uint16_t MAINNBYTES;
extern rfm69frame_t latestframe;
extern char datastream[255];
extern char datainfo[255];

void writeData(int event_, uint32_t tick_ );
void write_to_file(char *radiostream, uint8_t nvals);
void newFile(int event_, uint32_t tick_ );
bool isFileBusy(void);
void writeMainbuffer(void);



#endif
