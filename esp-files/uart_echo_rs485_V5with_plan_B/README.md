# UART RS485 DATA TRANSMISSION

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This code sends all the data it receives on UART to the sender in the RS485 network.
It uses the ESP-IDF UART software driver in RS485 half-duplex transmission mode and requires external connection of bus drivers.
A baud rate between the values 200 and 255 is sent. Then the movement is defined according to a function code:
* '1' --> Avancer 
* '2' --> Reculer
* '3' --> Tourner à droite
* '4' --> Tourner à gauche
* '5' --> Stop

## How to use example

### Hardware Required
PC + USB Serial adapter connected to USB port + RS485 line drivers + ESP32-S2 board.
The MAX485 line driver is used for example below but other similar chips can be used as well.

#### RS485 example connection circuit schematic:
```
         VCC ---------------+                               +--------------- VCC
                            |                               |
                    +-------x-------+               +-------x-------+
         RXD <------| RO            |               |             RO|-----> RXD
                    |              B|---------------|B              |
         TXD ------>| DI  MAX485    |    \  /       |    MAX485   DI|<----- TXD
ESP32               |               |   RS-485 side |               |  SERIAL ADAPTER SIDE
         RTS --+--->| DE            |    /  \       |             DE|---+
               |    |              A|---------------|A              |   |
               +----| /RE           |               |            /RE|---+-- RTS -- DTR
                    +-------x-------+               +-------x-------+
                            |                               |
                           ---                             ---
```

#### Connect an external RS485 serial interface to an ESP32 board
Connect USB to RS485 adapter to computer and connect its D+, D- output lines with the D+, D- lines of RS485 line driver connected to ESP32 (See picture above).
```
  --------------------------------------------------------------------------------------------------
  | ESP32 Interface       | #define            | Default ESP32-S2 Pins | External RS485 Driver Pin |
  | ----------------------|--------------------|-----------------------|---------------------------|
  | Transmit Data (TxD)   | CONFIG_MB_UART_TXD | GPIO20                | DI                        |
  | Receive Data (RxD)    | CONFIG_MB_UART_RXD | GPIO19                | RO                        |
  | Request To Send (RTS) | CONFIG_MB_UART_RTS | GPIO18                | ~RE/DE                    |
  | Ground                | n/a                | GND                   | GND                       | 
  --------------------------------------------------------------------------------------------------
```


### Configure the project
```
idf.py menuconfig
```

### Build and Flash
Build the project and flash it to the board, then run monitor tool to view serial output:
```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

#### Setup external terminal software
Refer to the example and set up a serial terminal program to the same settings as of UART in ESP32-S2 board.
Open the external serial interface in the terminal. By default if no any symbols are received, the application prints character `.` to check transmission side.
When typing message and push send button in the terminal you should see the message `RS485 Received: [ your message ]`,then, depending on the message, defines a speed for each motor or a movement action of the robot.
Verify if echo indeed comes from ESP32 by disconnecting either `TxD` or `RxD` pin. Once done there should be no any `.` displayed.

## Example Output
Example output of the application:
```
I (655020) RS485_ECHO_APP: Received 3 bytes:
[ 230 230 ]
2
Le robot recule
[ 230 230]
```
The received message is showed in uint8_t form in the brackets.

## Troubleshooting
When example software does not show the `.` symbol, the issue is most likely related to connection errors of the external RS485 interface.
Check the RS485 interface connection with the environment according to schematic above and restart the application.
Then start terminal software and open the appropriate serial port.

