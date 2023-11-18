#ifndef _MACROS_H_
#define _MACROS_H_

#define BAUDRATE 9600
#define N_TRIES 3
#define TIMEOUT 5

#define FLAG 0x7E
#define ADRESS1 0X03
#define ADRESS2 0X01
#define SET 0X03
#define UA 0X07

#define ESCAPE 0x7D

#define DISC 0X0B

#define FALSE 0
#define TRUE 1

// volatile int STOP_ = FALSE;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

#define NS(ns) (ns << 6)
#define RR(nr) ((nr << 7) | 0x05)
#define REJ(nr) ((nr << 7) | 0x01)

#endif