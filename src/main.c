 
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
// #include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <inttypes.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

    int serialBegin();
    int serialRead(uint8_t* frame);
    int sbusParse(uint8_t* frame, uint16_t* channels);

    static const uint32_t _sbusBaud = 100000;
    static const uint8_t _numChannels = 16;
    static const uint8_t _sbusHeader = 0x0F;
    static const uint8_t _sbusFooter = 0x00;
    static const uint8_t _sbus2Footer = 0x04;
    static const uint8_t _sbus2Mask = 0x0F;
    static const uint32_t SBUS_TIMEOUT_US = 7000;
    static const uint8_t _payloadSize = 24;
    static const uint8_t _sbusLostFrame = 0x04;
    static const uint8_t _sbusFailSafe = 0x08;
    static const char _serialPath[] = "/dev/ttyUSB0";

    int serial_port = -1;


int main(int argc, char const *argv[]){

    serialBegin();

    uint16_t channels[_numChannels];

    uint8_t frame[_payloadSize];

    while(1){
        // char buffer[1256];
        serialRead(frame);

        // for(int i = 0; i < _payloadSize; i++){
        //     printf("%3d ", frame[i]);
        // }
        // printf("\n");

        sbusParse(frame, channels);

        for(int i = 0; i < _numChannels; i++){
            printf("%4d ", channels[i]);
        }
        printf("\n");
    }




    

        //     // switch (read_buf[0])
        //     // {
        //     // case 0x0F:
        //     //     packetIndex = 0;
        //     //     printf("Packet Start\n");
        //     //     break;

        //     // case 0x00:

        //     //     for(int i = 0; i < 16; i++){
        //     //         printf("%d ", (int)packet[i]);
        //     //     }

        //     //     printf("\nPacket end\n");

        //     //     break;
            
        //     // default:
        //     //     packet[packetIndex++] = read_char;
        //     //     break;
        //     // }

        //     // 16 channels of 11 bit data
   
		// 	channels[0]  = (int) ((read_buf[0]     | read_buf[1] <<8)                     & 0x07FF);
		// 	channels[1]  = (int) ((read_buf[1]>>3  | read_buf[2] <<5)                     & 0x07FF);
		// 	channels[2]  = (int) ((read_buf[2]>>6  | read_buf[3] <<2 | read_buf[4]<<10)   & 0x07FF);
		// 	channels[3]  = (int) ((read_buf[4]>>1  | read_buf[5] <<7)                     & 0x07FF);
		// 	channels[4]  = (int) ((read_buf[5]>>4  | read_buf[6] <<4)                     & 0x07FF);
		// 	channels[5]  = (int) ((read_buf[6]>>7  | read_buf[7] <<1 | read_buf[8]<<9)    & 0x07FF);
		// 	channels[6]  = (int) ((read_buf[8]>>2  | read_buf[9] <<6)                     & 0x07FF);
		// 	channels[7]  = (int) ((read_buf[9]>>5  | read_buf[10]<<3)                     & 0x07FF);
		// 	channels[8]  = (int) ((read_buf[11]    | read_buf[12]<<8)                     & 0x07FF);
		// 	channels[9]  = (int) ((read_buf[12]>>3 | read_buf[13]<<5)                     & 0x07FF);
		// 	channels[10] = (int) ((read_buf[13]>>6 | read_buf[14]<<2 | read_buf[15]<<10)  & 0x07FF);
		// 	channels[11] = (int) ((read_buf[15]>>1 | read_buf[16]<<7)                     & 0x07FF);
		// 	channels[12] = (int) ((read_buf[16]>>4 | read_buf[17]<<4)                     & 0x07FF);
		// 	channels[13] = (int) ((read_buf[17]>>7 | read_buf[18]<<1 | read_buf[19]<<9)   & 0x07FF);
		// 	channels[14] = (int) ((read_buf[19]>>2 | read_buf[20]<<6)                     & 0x07FF);
		// 	channels[15] = (int) ((read_buf[20]>>5 | read_buf[21]<<3)                     & 0x07FF);




    close(serial_port);

       

    return 0;
}


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


    return 0;
}


int serialBegin(){

    // serial_port = open(_serialPath, O_RDWR | O_NOCTTY);
    serial_port = open(_serialPath, O_RDWR | O_NOCTTY);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
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

    return 0;
}


int serialRead(uint8_t* frame){

    char byteBuffer[1];
    char prevByte = _sbusFooter;
    int frameCouter = 0;

    // int available = 0;
    // if( ioctl(serial_port, FIONREAD, &available ) < 0 ) {
    //     printf("Error %i from ioctl FIONREAD: %s\n", errno, strerror(errno));
    // }
    // if(available >0){
    

    while(frameCouter < _payloadSize){
        int len = read(serial_port, byteBuffer, sizeof(byteBuffer));
        // printf("%s ", byteBuffer);
        
        if(byteBuffer[0] == _sbusHeader && prevByte == _sbusFooter){
            frameCouter = 0;
            // printf("\n");
        }else{
            frame[frameCouter] = (uint8_t)byteBuffer[0]; 
            // printf("%d: %1s  ",frameCouter, byteBuffer);
            frameCouter++;
        }

        prevByte = byteBuffer[0];
    }

    // printf("\t\t");



    return 0;
}