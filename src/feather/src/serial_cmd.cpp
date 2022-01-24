
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
	
	
	
	int port = open ("/dev/ttyUSB0", O_RDWR);
	struct termios tty;

	if (tcgetattr(port, &tty) != 0 ) {
		return 1;
	}
	
	cfsetospeed(&tty, B115200);
	if ( com == 0 ) write(port, "0", 1);
	if ( com == 1 ) write(port, "1", 1);
	if ( com == 2 ) write(port, "2", 1);
	if ( com == 3 ) write(port, "3", 1);
	if ( com == 4 ) write(port, "4", 1);
	if ( com == 5 ) write(port, "5", 1);
	if ( com == 6 ) write(port, "6", 1);
	close (port);


  return 0;
}
