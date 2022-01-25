#include <stdio.h>
#include <stdlib.h>
#include <iostream>

int serial_lain(char &port, int data){
    FILE *file;
    file = fopen(port,"w");  //Opening device file
        fprintf(file,"%d", data); //Writing to the file
    fclose(file);
}


int main()
{
    int data = 10;  //data we want to send
    char port[] = "/dev/ttyACM0";
    serial_lain(*port, data);
}
