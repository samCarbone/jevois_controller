// Serial code adapted from 
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

#include <iostream>
// #include <thread>
// #include <fcntl.h> // File controls
// #include <termios.h> // POSIX terminal
// #include <unistd.h> //
// #include <errno.h> // Error

#include "conductor.h"

int main(int argc, char*argv[])
{

    // /* 
    // * Serial port setup copy-pasta from
    // * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    // */

    // struct termios tty;
    // struct termios tty_original;
    // memset(&tty, 0, sizeof(tty));

    // // Open the serial port.
    // int serial_port = open("/dev/ttyS0", 0_RDWR);

    // // Acquire non-blocking exclusive lock
    // if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
    //     throw std::runtime_error("Serial port with file descriptor " + 
    //         std::to_string(fd) + " is already locked by another process.");
    // }

    // // Read in existing settings
    // if(tcgetattr(serial_port, &tty_original) != 0) {
    //     std::cerr <<  "Error " << errno << " from tcgetattr: " 
    //     << strerror(errno) << std::endl;
    // }
    // if(tcgetattr(serial_port, &tty) != 0) {
    //     std::cerr <<  "Error " << errno << " from tcgetattr: " 
    //     << strerror(errno) << std::endl;
    // }

    // // Setup serial params
    // tty.c_cflag &= ~PARENB; // No parity bit
    // tty.c_cflag &= ~CSTOPB; // One stop bit
    // tty.c_cflag |= CS8; // 8 bits per byte
    // tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    // tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
    
    // tty.c_lflag &= ~ICANON;
    // tty.c_lflag &= ~ECHO; // Disable echo
    // tty.c_lflag &= ~ECHOE; // Disable erasure
    // tty.c_lflag &= ~ECHONL; // Disable new-line echo
    // tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT, SUSP
    // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    // tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    // tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    // tty.c_oflag &= ~ONLCR; // Prevent converson of NL to CR

    // tty.c_cc[VTIME] = 0;
    // tty.c_cc[VMIN] = 0;

    // // Set in/out baud rate to be 115200
    // //
    // // TODO: try setting to two different rates?
    // cfsetispeed(&tty, B115200);
    // cfsetospeed(&tty, B115200)

    // // Save tty settings, also checking for error
    // if (tcsetattr(serial_port, TCSANOW, &tty_original) != 0) {
    //     std::cerr << "Error " << errno << " from tcsetattr: "
    //     << strerror(errno) << std::endl;
    // }

    Conductor my_object("/dev/ttyS0");


    // // Set serial port params back to the original
    // if (tcsetattr(serial_port, TCSANOW, &tty_original) != 0) {
    //     std::cerr << "Error " << errno << " from tcsetattr: "
    //     << strerror(errno) << std::endl;
    // }

    close(serial_port);
}