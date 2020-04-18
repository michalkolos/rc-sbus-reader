 
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

    int serial_port = -1;


int main(int argc, char const *argv[]){

    serialBegin();

    uint16_t channels[_numChannels];

    uint8_t frame[_payloadSize];

    while(1){
        // char buffer[1256];
        serialRead(frame);

        for(int i = 0; i < _payloadSize; i++){
            printf("%3d ", frame[i]);
        }
        printf("\n");
    }




    // // Create new termios struc, we call it 'tty' for convention
    // struct termios tty;
    // memset(&tty, 0, sizeof (tty));

    // // Read in existing settings, and handle any error
    // if(tcgetattr(serial_port, &tty) != 0) {
    //     printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    // }


    // // Modify configuration
    // tty.c_cflag |= PARENB;          // Set parity bit, enabling parity
    // tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication
    // tty.c_cflag |= CS8;             // 8 bits per byte
    // // tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control
    // tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    // tty.c_lflag &= ~ICANON;         // Disable canonical mode
    // tty.c_lflag &= ~ECHO;           // Disable echo
    // tty.c_lflag &= ~ISIG;           // Disable interpretation of INTR, QUIT and SUSP

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Turn off s/w flow ctrl
    
    // // Disable any special handling of received bytes
    // tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 

    // // Prevent special interpretation of output bytes (e.g. newline chars)
    // tty.c_oflag &= ~OPOST; 

    // tty.c_oflag &= ~ONLCR;          // Prevent conversion of newline to carriage return/line feed
    
    // // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    // tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    // tty.c_cc[VMIN] = 0;

    // // Specifying a custom baud rate when using GNU C
    // cfsetispeed(&tty, 100000);
    // cfsetospeed(&tty, 100000);

    // // Save tty settings, also checking for error
    // if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    //     printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    // }

    // // Allocate memory for read buffer, set size according to your needs
    // char read_buf [256];
    // memset(&read_buf, '\0', sizeof(read_buf));

    // int channels[32];
    // memset(&channels, '\0', sizeof(channels) * 4);
    
    

    // // Read bytes. The behaviour of read() (e.g. does it block?,
    // // how long does it block for?) depends on the configuration
    // // settings above, specifically VMIN and VTIME
    

    // printf("Start reading. \n");


    // while(1){
        // int n = read(serial_port, &read_buf, sizeof(read_buf));

        // printf("read\n");

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.

        // if(n >= 0){

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


        //     for(int i = 0; i < strlen(read_buf); i++){
        //         printf("%2d/%3d ", i,  (int)read_buf[i]);
        //     }

        //     for(int i = 0; i < 16; i++){
        //         printf("%2d/%4X ", i,  channels[i]);
        //     }


        //     printf("\n");

        //     // printf("%s\n", read_buf);
        // }

        // printf("%s\n", read_buf);
    // }

    close(serial_port);

       

    return 0;
}


int serialBegin(){

    serial_port = open("/dev/ttyS0", O_RDWR | O_NOCTTY);

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

    // // reset the parser state if too much time has passed
    //     static elapsedMicros _sbusTime = 0;
    //     if (_sbusTime > SBUS_TIMEOUT_US) {_parserState = 0;}
    //     // see if serial data is available
    //     while (_bus->available() > 0) {
    //         _sbusTime = 0;
    //         _curByte = _bus->read();
    //         // find the header
    //         if (_parserState == 0) {
    //                 if ((_curByte == _sbusHeader) && ((_prevByte == _sbusFooter) || ((_prevByte & _sbus2Mask) == _sbus2Footer))) {
    //                     _parserState++;
    //                 } else {
    //                     _parserState = 0;
    //                 }
    //         } else {
    //             // strip off the data
    //             if ((_parserState-1) < _payloadSize) {
    //                 _payload[_parserState-1] = _curByte;
    //                 _parserState++;
    //             }
    //             // check the end byte
    //             if ((_parserState-1) == _payloadSize) {
    //                 if ((_curByte == _sbusFooter) || ((_curByte & _sbus2Mask) == _sbus2Footer)) {
    //                     _parserState = 0;
    //                     return 0;
    //                 } else {
    //                     _parserState = 0;
    //                     return -1;
    //                 }
    //             }
    //         }
    //         _prevByte = _curByte;
    //     }
    //     // return false if a partial packet
    //     return -1;

    return 0;
}