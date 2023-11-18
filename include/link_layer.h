// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    READING_RCV,
    ESCAPE_RCV,
    STOP,
} llMachineState;

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char *buf, int bufSize, int fd);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char *packet, int fd);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int fd, LinkLayer connectionParameters);

int serialPortConnection(LinkLayer connectionParameters);

void alarmHandler(int signal);

int sendFrame(int fd, unsigned char adress, unsigned char control);

llMachineState tx_llopen_machinestate(int fd);

void rx_llopen_machinestate(int fd);

unsigned char trama_answer_machinestate(int fd);

llMachineState tx_llclose_machinestate(int fd);

void rx_llclose_machinestate(int fd);

#endif // _LINK_LAYER_H_
