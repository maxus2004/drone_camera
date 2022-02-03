#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <queue>
#include "fc_serial.h"


static std::thread read_thread;
static cv::Vec4f rotation;
static int serial_port;
//linux serial port tutorial
//https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

static void send(uint8_t *data, int count)
{
    write(serial_port, data, count);
}

static void received(uint8_t *data, int count)
{
    if (count == 17)
    {
        float r_array[4];
        memcpy(r_array,data,16);
        cv::Vec4f r = cv::Vec4f(r_array[0],r_array[1],r_array[2],r_array[3]);
        rotation = r;
    }
}

static void readLoop()
{
    uint8_t read_buf[1024];
    std::queue<uint8_t> receive_queue;
    while (true)
    {
        int count = read(serial_port, &read_buf, sizeof(read_buf));
        if (count < 0)
        {
            printf("error reading data!\n");
            exit(0);
        }
        
        for(int i = 0;i<count;i++){
            receive_queue.push(read_buf[i]);
            int len = receive_queue.size();
            if(len>16 && read_buf[i]=='\n'){
                std::vector<uint8_t> buf;
                for(int j = 0;j<len;j++){
                    buf.push_back(receive_queue.front());
                    receive_queue.pop();
                }
                received(&buf[0], len);
            }
        }

        
    }
}

void init_serial()
{
    serial_port = open("/dev/ttyS0", O_RDWR);
    if (serial_port < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(0);
    }
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(0);
    }
    tty.c_cflag &= ~PARENB;        // no parity bit
    tty.c_cflag &= ~CSTOPB;        // one stop bit
    tty.c_cflag &= ~CSIZE;         // clear size bits
    tty.c_cflag |= CS8;            // set size to 8 bits
    tty.c_cflag &= ~CRTSCTS;       // disable RTS/CTS
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON; //disable canocical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST;                                                       // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;                                                       // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    //block until new character is read
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 1;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(0);
    }

    read_thread = std::thread(readLoop);
}

cv::Vec4f fc_getRotation()
{
    return rotation;
}