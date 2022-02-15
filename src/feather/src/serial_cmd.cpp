
// library
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "termios.h"
#include "fcntl.h"
#include "unistd.h"
#include <iostream>
#include "stdint.h"
#include <cstring>
#include <sstream>
#include "errno.h" // Error integer and strerror() function
#include "sys/ioctl.h"
#include "linux/serial.h"
using namespace std;


int send_serial_V1(int com){
	system("stty -F /dev/ttyUSB0 115200");
	int port = open ("/dev/ttyUSB0", O_RDWR);
	struct termios tty;
	if (tcgetattr(port, &tty) != 0 ) {
		return 1;
	}
	if ( com == 0) write(port, "0", 1);//led off,
	if ( com == 1 ) write(port, "1", 1);// led on
	if ( com == 2 ) write(port, "z", 1);// avancer
	if ( com == 3 ) write(port, "s", 1);// reculer
	if ( com == 4 ) write(port, "d", 1);// turn right
	if ( com == 5 ) write(port, "q", 1);// turn left
	if ( com == 6 ) write(port, "f", 1);//stop
	close (port);
  return 0;
}


//******************************************************************************
//************************Fonctions port****************************************
void set_rts(int fd, int actif){
	int bits;
	ioctl(fd, TIOCMGET, &bits);
	if ( actif ) {
		bits |= TIOCM_RTS;
		ioctl(fd, TIOCMSET, &bits);
		printf("TIOCM_RTS : %d \n",bits);
		
	} else {
		bits &= ~TIOCM_RTS;
		ioctl(fd, TIOCMSET, &bits);
		printf("TIOCM_RTS : %d \n",bits);
		
	}
	
}

void set_dtr(int fd, int actif){
	int bits;
	ioctl(fd, TIOCMGET, &bits);
	if ( actif ) {
 		bits |= TIOCM_DTR;
		ioctl(fd, TIOCMSET, &bits);
		printf("TIOCM_DTR : %d \n",bits);
		
	} else {
		bits &= ~TIOCM_DTR;
		ioctl(fd, TIOCMSET, &bits);
		printf("TIOCM_DTR : %d \n",bits);
		
	}
	
}
void set_cts(int fd, int actif){
	int bits;
	ioctl(fd, TIOCMGET, &bits);
	if ( actif ) {
 		bits |= TIOCM_CTS;
		ioctl(fd, TIOCMSET, &bits);
		printf("TIOCM_CTS : %d \n",bits);
		
	} else {
		bits &= ~TIOCM_CTS;
		ioctl(fd, TIOCMSET, &bits);
		printf("TIOCM_CTS : %d \n",bits);
		
	}
	
}
//******************************************************************************
//************************Fonctions ESP -> RASPBERRY****************************
/*void receive_speed(){
	uint8_t data[2],speed[2];
	int num_bytes = 0;
	int port = open ("/dev/ttyUSB0", O_RDONLY);
	system("stty -F /dev/ttyUSB0 115200");	
	struct termios tty;
	if (tcgetattr(port, &tty) != 0 ) printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	if (tcsetattr(port, TCSANOW, &tty) != 0) printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	set_dtr(port,1); // on est pret a recevoir
	set_rts(port,0); // on ne veut pas ecrit
	num_bytes = read(port, data, sizeof(data));
	printf("nombre de bytes recus : %d \n",num_bytes);
	if (num_bytes > 0 ) {
		for ( int i = 0 ; i < num_bytes ; i++ ) speed[i]=data[i];
		printf("Vitesse du moteur droite : %d m/s\n", data[0]);
		printf("Vitesse du moteur gauche : %d m/s\n", data[1]);
	} else {
      		printf("Error reading: %s", strerror(errno));	
	}
	close (port);
  	
}*/

//******************************************************************************
//************************Plan A Fonctions RASPBERRY -> ESP****************************************
/*int send_serial_cmd_vitesse_motor( uint8_t a, uint8_t b, uint8_t c, uint8_t d){
	uint8_t tab[4] = {a,b,c,d};
	uint8_t data[127];
	int num_bytes = 0;
	int port = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
	//int port = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
	system("stty -F /dev/ttyUSB0 115200");	
	struct termios tty;
	if (tcgetattr(port, &tty) != 0 ) printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	if (tcsetattr(port, TCSANOW, &tty) != 0) printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	set_dtr(port,0); // on est pas pret a recevoir
	//set_rts(port,1); 
	for ( int j = 0 ; j < 4 ; j++ ) {
		write(port,&tab[j],1); //envoi
		printf("%d ", tab[j]);
	}
	printf("\n");
	set_dtr(port,1); 
	close (port); 
  	return 0;
}
*/
	
int send_serial_cmd_motor(int com){
	int port = open ("/dev/ttyUSB0", O_WRONLY);
	system("stty -F /dev/ttyUSB0 115200");	
	struct termios tty;
	if (tcgetattr(port, &tty) != 0 ) printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	if (tcsetattr(port, TCSANOW, &tty) != 0) printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	set_dtr(port,0);
	write(port, &com, 1); 
	set_dtr(port,1);
	close (port);
  	return 0;
}

//******************************************************************************
//************************Plan B Fonctions RASPBERRY -> ESP****************************************
int send_serial_cmd_vitesse_motor( uint8_t a, uint8_t b){
	uint8_t tab[2] = {a,b};
	uint8_t data[127];
	int num_bytes = 0;
	int port = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
	//int port = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
	system("stty -F /dev/ttyUSB0 115200");	
	struct termios tty;
	if (tcgetattr(port, &tty) != 0 ) printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	if (tcsetattr(port, TCSANOW, &tty) != 0) printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	set_dtr(port,0); // on est pas pret a recevoir
	//set_rts(port,1); 
	for ( int j = 0 ; j < 2 ; j++ ) {
		write(port,&tab[j],1); //envoi
		printf("%d ", tab[j]);
	}
	printf("\n");
	set_dtr(port,1); 
	close (port); 
  	return 0;
}

/*	
int send_serial_cmd_motor(int in1,int in2, int in3, int in4){
	int tab[4] = {in1,in2,in3,in4};
	int port = open ("/dev/ttyUSB0", O_WRONLY);
	system("stty -F /dev/ttyUSB0 115200");	
	struct termios tty;
	if (tcgetattr(port, &tty) != 0 ) printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	if (tcsetattr(port, TCSANOW, &tty) != 0) printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	set_dtr(port,0);
	for ( int j = 0 ; j < 4 ; j++ ) {
		write(port,&tab[j],1); //envoi
		printf("%d ", tab[j]);
	}
	printf("\n");
	set_dtr(port,1); 
	close (port); 
  	return 0;
}*/

