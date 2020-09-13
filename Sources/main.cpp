// Serial code adapted from 
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

#include <iostream>
// #include <thread>
// #include <fcntl.h> // File controls
// #include <termios.h> // POSIX terminal
// #include <unistd.h> //
// #include <errno.h> // Error

#include "conductor.h"
#include <string>

int main(int argc, char* argv[])
{

    omp_set_num_threads(2); // Set two threads for use in OpenMP which is used by Eigen

    std::string file_name = "/dev/ttyS0";
    if(argc >= 3) {
        if(strcmp(argv[1], "-f") == 0) {
            file_name.assign(argv[2]);
            #ifdef IS_HOST
            std::cout << file_name << std::endl;
            #endif
        }
    }

    #ifdef IS_HOST
    std::cout << "Running on host" << std::endl;
    #endif
    
    unsigned int baud_rate = 57600;
    Conductor *my_object = new Conductor(file_name, baud_rate);


}
