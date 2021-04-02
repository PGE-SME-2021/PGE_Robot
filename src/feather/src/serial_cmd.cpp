
// library
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "termios.h"
#include "fcntl.h"
#include "unistd.h"
#include <iostream>
using namespace std;

int send_serial(int com){
	
	
	system("stty -F /dev/ttyUSB0 115200");
	int port = open ("/dev/ttyUSB0", O_RDWR);
	
	struct termios tty;

	if (tcgetattr(port, &tty) != 0 ) {
		return 1;
	}
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	if ( com == "0,,) write(port, "0", 1);//led off,
	if ( com == 1 ) write(port, "1", 1);// led on
	if ( com == 2 ) write(port, "2", 1);// avancer
	if ( com == 3 ) write(port, "3", 1);// reculer
	if ( com == 4 ) write(port, "4", 1);// turn right
	if ( com == 5 ) write(port, "5", 1);// turn left
	if ( com == 6 ) write(port, "6", 1);//stop
	close (port);


  return 0;
}
