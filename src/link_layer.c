// Link layer protocol implementation

#include "link_layer.h"
#include "macros.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int alarmEnabled = FALSE;
int alarmCount = 0;

unsigned int tramaCtx = 0;
unsigned int tramaCrx = 1;
int nRetransmissions = 0;
int timeout = 0;

int serialPortConnection(LinkLayer connectionParameters)
{   
    const char *serialPortName = connectionParameters.serialPort;
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    return fd;
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE; 
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}
 
int sendFrame(int fd, unsigned char adress, unsigned char control){
    unsigned char buf[5] = {FLAG, adress, control, adress ^ control, FLAG};
    int byteswritten = write(fd, buf, 5);
    return byteswritten;
}


int llopen(LinkLayer connectionParameters){
    int fd = serialPortConnection(connectionParameters);
    if(fd < 0){
        return -1;
    }

    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    llMachineState currentstate = START;

    switch (connectionParameters.role){
        case LlTx:{
            currentstate = tx_llopen_machinestate(fd);
            if(currentstate != STOP) return -1;
            break;
        }

        case LlRx:{
            rx_llopen_machinestate(fd);
            break;
        }

        default:{
            return -1;
            break;
        }
    }

    return fd;
}

int llwrite(const unsigned char *buf, int bufSize, int fd)
{
    int tramaSize = bufSize + 6; // (F,A,C,BCC1, bufdata, BCC2, F)
    unsigned char* informtrama = (unsigned char*) malloc(tramaSize);
   
    unsigned char BCC2 = 0;
    for(int i = 0; i < bufSize; i++){
        BCC2 ^= buf[i];
    }

    informtrama[0] = FLAG;
    informtrama[1] = ADRESS1;
    informtrama[2] = NS(tramaCtx);
    informtrama[3] = informtrama[1] ^ informtrama[2];

    int dataindx = 4;
    for(int i = 0; i < bufSize; i++){
        if(buf[i] == FLAG){
            tramaSize++;
            informtrama = realloc(informtrama, tramaSize);
            informtrama[dataindx] = 0x7D; // FLAG for 0x7E
            dataindx++;
            informtrama[dataindx] = 0x5E;   
            dataindx++;
        }
        else if(buf[i] == ESCAPE){
            tramaSize++;
            informtrama = realloc(informtrama, tramaSize);
            informtrama[dataindx] = 0x7D; // FLAG for 0x7D
            dataindx++;
            informtrama[dataindx] = 0x5D; 
            dataindx++;
        }
        else{
            informtrama[dataindx] = buf[i];
            dataindx++;
        }
    }

    if(BCC2 == FLAG){
        tramaSize++;
        informtrama = realloc(informtrama, tramaSize);
        informtrama[dataindx] = 0x7D; // FLAG for 0x7E
        dataindx++;
        informtrama[dataindx] = 0x5E;   
        dataindx++;
    }
    else if(BCC2 == ESCAPE){
        tramaSize++;
        informtrama = realloc(informtrama, tramaSize);
        informtrama[dataindx] = 0x7D; // FLAG for 0x7E
        dataindx++;
        informtrama[dataindx] = 0x5D;   
        dataindx++;
    }
    else{
        informtrama[dataindx] = BCC2;
        dataindx++;
    }
    informtrama[dataindx] = FLAG;
    
    int nRetransmissions_aux = nRetransmissions;
    int rej = 0;
    int acc = 0;

    while(nRetransmissions_aux > 0){
        alarmEnabled = TRUE;
        alarm(timeout);
        rej = 0;
        acc = 0;
        while(alarmEnabled == TRUE && acc == 0 && rej == 0){
           int byteswritten = write(fd, informtrama, tramaSize);

           if (byteswritten < 0){
                printf("Error writing trama\n");
                exit(-1);
            }

            unsigned char answer = trama_answer_machinestate(fd);
            printf("Answer in hexadecimal: 0x%02X\n", answer);

            if (answer == RR(0) || answer == RR(1)){
                acc = 1;
                tramaCtx = (tramaCtx + 1) % 2;
            }
            else if (answer == REJ(0) || answer == REJ(1)){
                rej = 1;
            }
            else continue;  
        }
        if(acc) break;
        else if(rej) nRetransmissions_aux = nRetransmissions;
        else nRetransmissions_aux--;
    }
    free(informtrama);
    if(acc) return tramaSize;
    else return -1;
}

int llread(unsigned char *packet, int fd){
    unsigned char currbyte, field;
    int currentidx = 0;
    llMachineState currstate = START;
    
    while (currstate != STOP){
        if (read(fd, &currbyte, 1) > 0){
            switch (currstate){
                case START:{
                    if (currbyte == FLAG) currstate = FLAG_RCV;
                    break;
                }
                case FLAG_RCV:{
                    if (currbyte != FLAG){
                        if (currbyte == ADRESS1) currstate = A_RCV;
                        else currstate = START;
                    }
                    break;
                }
                case A_RCV:{
                    if (currbyte == FLAG) currstate = FLAG_RCV;
                    else if (currbyte == NS(0) || currbyte == NS(1)){
                        currstate = C_RCV;
                        field = currbyte;
                    }
                    else currstate = START;
                    break;
                }
                case C_RCV:{
                    if (currbyte == FLAG) currstate = FLAG_RCV;
                    else if (currbyte == (ADRESS1 ^ field)) currstate = READING_RCV;
                    else currstate = START;
                    break;
                }
                case READING_RCV:{
                    if (currbyte == FLAG){
                        unsigned char BCC2 = packet[currentidx - 1];
                        currentidx--;
                        unsigned char bccaux = 0;
                        for(int i = 0; i < currentidx; i++) bccaux ^= packet[i];
                        if(BCC2 == bccaux){
                            currstate = STOP;
                            if(NS(tramaCrx) != field){
                                sendFrame(fd, ADRESS1, RR(tramaCrx));
                                tramaCrx = (tramaCrx + 1) % 2;
                                printf("mandei um receiver ready\n");
                                return currentidx;
                            }
                            else{
                                sendFrame(fd, ADRESS1, RR(tramaCrx));
                                printf("mandei um receiver ready, trama repetida sem erros\n");
                                return 0;
                            }
                        }
                        else{
                            if(NS(tramaCrx) != field){
                                printf("mandei um reject\n");
                                sendFrame(fd, ADRESS1, REJ(tramaCrx));
                                return -1; 
                            }
                            else{
                                printf("mandei receiver ready, trama repetida com erros\n");
                                sendFrame(fd, ADRESS1, RR(tramaCrx));
                                return 0; 
                            }
                        }
                    }
                    else if (currbyte == ESCAPE) currstate = ESCAPE_RCV;
                    else packet[currentidx++] = currbyte;
                    break;
                }
                case ESCAPE_RCV:{
                    unsigned char identifier = packet[1];
                    printf("stufing no packet:  %d\n", identifier);
                    currstate = READING_RCV;
                    if(currbyte == 0x5E) packet[currentidx++] = FLAG;
                    else if(currbyte == 0x5D) packet[currentidx++] = ESCAPE;
                    break;
                }
                default:
                    break;
            }
        }
    }
    return -1;
}

int llclose(int fd, LinkLayer connectionParameters){
    llMachineState currentstate = START;

    switch (connectionParameters.role){
        case LlTx:{
            currentstate = tx_llclose_machinestate(fd);
            if(currentstate != STOP) return -1;
            sendFrame(fd, ADRESS1, UA);
            break;
        }

        case LlRx:{
            rx_llclose_machinestate(fd);
            break;
        }

        default:{
            return -1;
            break;
        }
    }

    return close(fd);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// MACHINE STATES //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

llMachineState tx_llopen_machinestate(int fd){
    llMachineState currentstate = START;
    unsigned char currbyte;
    int nRetransmissions_aux = nRetransmissions;

    (void)signal(SIGALRM, alarmHandler);
    while(nRetransmissions_aux > 0 && currentstate != STOP){
        sendFrame(fd, ADRESS1, SET);
        alarm(timeout);
        alarmEnabled = TRUE;
        while(alarmEnabled == TRUE && currentstate != STOP){
            if(read(fd, &currbyte, 1) > 0){
            switch(currentstate){
                case START:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    break;
                }
                case FLAG_RCV:{
                     if(currbyte != FLAG){
                        if(currbyte == ADRESS1) currentstate = A_RCV;
                        else currentstate = START;
                    }
                    break;
                }
                case A_RCV:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    else if(currbyte == UA) currentstate = C_RCV;
                    else currentstate = START;
                    break;
                }
                case C_RCV:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    else if(currbyte == (ADRESS1 ^ UA)) currentstate = BCC_OK;
                    else currentstate = START;
                    break;
                }
                case BCC_OK:{
                    if(currbyte == FLAG) currentstate = STOP;
                    else currentstate = START;
                    break;
                }
                default:
                    break;
            }
            }
        }
        nRetransmissions_aux--;
    }
    return currentstate;
}

void rx_llopen_machinestate(int fd){
    llMachineState currentstate = START;
    unsigned char currbyte;

    while(currentstate != STOP){
        if(read(fd, &currbyte, 1) > 0){
            switch(currentstate){
                case START:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    break;
                }
                case FLAG_RCV:{
                    if(currbyte != FLAG){
                        if(currbyte == ADRESS1) currentstate = A_RCV;
                        else currentstate = START;
                    }
                    break;
                }
                case A_RCV:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    else if(currbyte == SET) currentstate = C_RCV;
                    else currentstate = START;
                    break;
                }
                case C_RCV:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    else if(currbyte == (ADRESS1 ^ SET)) currentstate = BCC_OK;
                    else currentstate = START;
                    break;
                }
                case BCC_OK:{
                    if(currbyte == FLAG) currentstate = STOP;
                    else currentstate = START;
                    break;
                }
                default:
                    break;
            }
        }
    }
    sendFrame(fd, ADRESS1, UA);
}

unsigned char trama_answer_machinestate(int fd){
    unsigned char currbyte, answer = 0;
    llMachineState currentstate = START;
    
    while(currentstate != STOP && alarmEnabled == TRUE){
        if(read(fd, &currbyte, 1) > 0){
            switch(currentstate){
                case START:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    break;
                }
                case FLAG_RCV:{
                    if(currbyte != FLAG){
                        if(currbyte == ADRESS1) currentstate = A_RCV;
                        else currentstate = START;
                    }
                    break;
                }
                case A_RCV:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    else if(currbyte == RR(0) || currbyte == RR(1) || currbyte == REJ(0) || currbyte == REJ(1)) {
                        currentstate = C_RCV;
                        answer = currbyte;
                    }
                    else currentstate = START;
                    break;
                }
                case C_RCV:{
                    if(currbyte == FLAG) currentstate = FLAG_RCV;
                    else if(currbyte == (ADRESS1 ^ answer)) currentstate = BCC_OK;
                    else currentstate = START;
                    break;
                }
                case BCC_OK:{
                    if(currbyte == FLAG) currentstate = STOP;
                    else currentstate = START;
                    break;
                }
                default:
                    break;
            }
        }
    }
    return answer;
}

llMachineState tx_llclose_machinestate(int fd){
    llMachineState currentstate = START;
    unsigned char currbyte;
    int nRetransmissions_aux = nRetransmissions;

    (void) signal(SIGALRM, alarmHandler);
    while (nRetransmissions_aux > 0 && currentstate != STOP){
        sendFrame(fd, ADRESS1, DISC);
        alarm(TIMEOUT);
        alarmEnabled = TRUE;
        while (alarmEnabled == TRUE && currentstate != STOP){
            if(read(fd, &currbyte, 1) > 0){
                switch (currentstate){
                    case START:{
                        if (currbyte == FLAG) currentstate = FLAG_RCV;
                        break;
                    }
                    case FLAG_RCV:{
                        if (currbyte != FLAG){
                            if(currbyte == ADRESS2) currentstate = A_RCV;
                            else currentstate = START;
                        }
                        break;
                    }
                    case A_RCV:{
                        if (currbyte == FLAG) currentstate = FLAG_RCV;
                        else if (currbyte == DISC) currentstate = C_RCV;
                        else currentstate = START;
                        break;
                    }
                    case C_RCV:{
                        if (currbyte == FLAG) currentstate = FLAG_RCV;
                        else if (currbyte == (ADRESS2 ^ DISC)) currentstate = BCC_OK;
                        else currentstate = START;
                        break;
                    }
                    case BCC_OK:{
                        if (currbyte == FLAG) currentstate = STOP;
                        else currentstate = START;
                        break;
                    }
                    default:
                        break;
                }
            }
        }
        nRetransmissions_aux--;
    }
    return currentstate;
}

void rx_llclose_machinestate(int fd){
    llMachineState currentstate = START;
    unsigned char currbyte;

    while(currentstate != STOP){
        if(read(fd, &currbyte, 1) > 0){
            switch (currentstate){
                case START:{
                    if (currbyte == FLAG) currentstate = FLAG_RCV;
                    break;
                }
                case FLAG_RCV:{
                    if (currbyte != FLAG){
                        if(currbyte == ADRESS1) currentstate = A_RCV;
                        else currentstate = START;
                    }
                    break;
                }
                case A_RCV:{
                    if (currbyte == FLAG) currentstate = FLAG_RCV;
                    else if (currbyte == DISC) currentstate = C_RCV;
                    else currentstate = START;
                    break;
                }
                case C_RCV:{
                    if (currbyte == FLAG) currentstate = FLAG_RCV;
                    else if (currbyte == (ADRESS1 ^ DISC)) currentstate = BCC_OK;
                    else currentstate = START;
                    break;
                }
                case BCC_OK:{
                    if (currbyte == FLAG) currentstate = STOP;
                    else currentstate = START;
                    break;
                }
                default:
                    break;
            }
        }
    }
    sendFrame(fd, ADRESS2, DISC);
}