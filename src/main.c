 
// C library headers
#include <stdio.h>
#include <string.h>
#include <inttypes.h>


// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#define FAILSAFE 18
#define LOST_FRAME 19


int serialBegin();
int serialRead(uint8_t* frame);
int sbusParse(uint8_t* frame, uint16_t* channels);
void sbusPrint(uint16_t* channels);


static const uint32_t _sbusBaud = 100000; 
static const uint8_t _numChannels = 20;
static const uint8_t _sbusHeader = 0x0F;
static const uint8_t _sbusFooter = 0x00;

static const uint8_t _channel16 = 0x01;
static const uint8_t _channel17 = 0x02;
static const uint8_t _sbusLostFrame = 0x04;
static const uint8_t _sbusFailSafe = 0x08;

// static const uint8_t _sbus2Footer = 0x04;
// static const uint8_t _sbus2Mask = 0x0F;
// static const uint32_t SBUS_TIMEOUT_US = 7000;
static const uint8_t _payloadSize = 24;

static const char _serialPath[] = "/dev/ttyUSB0";

int serial_port = -1;


int main(int argc, char const *argv[]){

    uint16_t channels[_numChannels];
    uint8_t frame[_payloadSize];

    if(!serialBegin()){
        return -1;
    }
    
    while(1){
        if(serialRead(frame)){
            sbusParse(frame, channels);
            sbusPrint(channels);  
        }
              
    }

    close(serial_port);
    return 0;
}




/**
 * @brief Prints channel data to the console.
 * 
 * @param channels - array of values for individual channels.
 */
void sbusPrint(uint16_t* channels){

    for(int i = 0; i < _numChannels; i++){
            printf("%4d ", channels[i]);
        }
        printf("\n");

    return;
}



/**
 * @brief Parses raw data into usefull values representing separate controll 
 * channels
 * 
 * @param frame - raw data received from the serial interface.
 * @param channels - array of integers that will hold decoded data. Must have the
 * length of at least 20:
 *      - 0 - 15:   16 analogue channels,
 *      - 16 - 17:  2 additional digital channels, 
 *      - 18: failsafe activated bit,
 *      - 19: lost frame bit.
 * @return int - 1 in noramal operation, -1 when lost frame was detected.
 */
int sbusParse(uint8_t* frame, uint16_t* channels){

    	channels[0]  = (uint16_t) ((frame[0]     | frame[1] <<8)                     & 0x07FF);
        channels[1]  = (uint16_t) ((frame[1]>>3  | frame[2] <<5)                     & 0x07FF);
        channels[2]  = (uint16_t) ((frame[2]>>6  | frame[3] <<2 | frame[4]<<10)      & 0x07FF);
        channels[3]  = (uint16_t) ((frame[4]>>1  | frame[5] <<7)                     & 0x07FF);
        channels[4]  = (uint16_t) ((frame[5]>>4  | frame[6] <<4)                     & 0x07FF);
        channels[5]  = (uint16_t) ((frame[6]>>7  | frame[7] <<1 | frame[8]<<9)       & 0x07FF);
        channels[6]  = (uint16_t) ((frame[8]>>2  | frame[9] <<6)                     & 0x07FF);
        channels[7]  = (uint16_t) ((frame[9]>>5  | frame[10]<<3)                     & 0x07FF);
        channels[8]  = (uint16_t) ((frame[11]    | frame[12]<<8)                     & 0x07FF);
        channels[9]  = (uint16_t) ((frame[12]>>3 | frame[13]<<5)                     & 0x07FF);
        channels[10] = (uint16_t) ((frame[13]>>6 | frame[14]<<2 | frame[15]<<10)     & 0x07FF);
        channels[11] = (uint16_t) ((frame[15]>>1 | frame[16]<<7)                     & 0x07FF);
        channels[12] = (uint16_t) ((frame[16]>>4 | frame[17]<<4)                     & 0x07FF);
        channels[13] = (uint16_t) ((frame[17]>>7 | frame[18]<<1 | frame[19]<<9)      & 0x07FF);
        channels[14] = (uint16_t) ((frame[19]>>2 | frame[20]<<6)                     & 0x07FF);
        channels[15] = (uint16_t) ((frame[20]>>5 | frame[21]<<3)                     & 0x07FF);

        if (frame[22] & _channel16) {
      		channels[16] = 1;
    	} else {
			channels[16] = 1;
        }

        if (frame[22] & _channel17) {
      		channels[17] = 1;
    	} else {
			channels[17] = 1;
        }

        if (frame[22] & _sbusFailSafe) {
      		channels[FAILSAFE] = 1;
    	} else {
			channels[FAILSAFE] = 0;
        }


        if (frame[22] & _sbusLostFrame) {
      	    channels[LOST_FRAME] = 1;
            return -1;
    	} else {
			channels[LOST_FRAME] = 0;
        }

    return 1;
}



/**
 * @brief Initializes Rasberry Pi's serial interface to accept communication 
 * with the radio receiver.
 * 
 * @return int: 1 - on success, -1 - on failure
 */
int serialBegin(){

    serial_port = open(_serialPath, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    struct termios2 tio;

    errno = 0;        
    if(ioctl(serial_port, TCGETS2, &tio)){
        printf("Error %i from ioctl TCGETS2: %s\n", errno, strerror(errno));
        return -1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_cflag |= CSTOPB; // 2 stop bits
    tio.c_cflag |= PARENB; // enable parity bit, even by default

    tio.c_cflag |= CS8; 
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= CREAD | CLOCAL;
    tio.c_lflag &= ~ICANON;
    tio.c_lflag &= ~ECHO;
    tio.c_lflag &= ~ISIG;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tio.c_oflag &= ~OPOST;
    tio.c_oflag &= ~ONLCR;
    tio.c_cc[VTIME] = 10;
    tio.c_cc[VMIN] = 0;

    tio.c_ispeed = tio.c_ospeed = _sbusBaud;

    if(ioctl(serial_port, TCSETS2, &tio)){
        printf("Error %i from ioctl TCSETS2: %s\n", errno, strerror(errno));
        return -1;
    }

    return 1;
}


/**
 * @brief Reads raw data from the serial interface.
 * 
 * @param frame - an array of bytes to store incomming data. Must be minimum 24 
 * bytes long
 * @return int: 1 - on success, -1 - on failure
 */
int serialRead(uint8_t* frame){

    char byteBuffer[1];
    char prevByte = _sbusFooter;
    int frameCouter = 0;


    // TODO: Check if testing for incomming data on serial port is necessary.
    // int available = 0;
    // if( ioctl(serial_port, FIONREAD, &available ) < 0 ) {
    //     printf("Error %i from ioctl FIONREAD: %s\n", errno, strerror(errno));
    // }
    // if(available >0){

    while(frameCouter < _payloadSize){
        int len = read(serial_port, byteBuffer, sizeof(byteBuffer));

        if(len < 0){
            printf("Error %i from read: %s\n", errno, strerror(errno));
            return -1;  
        }
        
        if(byteBuffer[0] == _sbusHeader && prevByte == _sbusFooter){
            frameCouter = 0;
        }else{
            frame[frameCouter] = (uint8_t)byteBuffer[0]; 
            frameCouter++;
        }

        prevByte = byteBuffer[0];
    }

    return 1;
}