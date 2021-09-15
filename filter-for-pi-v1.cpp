
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <stdbool.h>
#include <sys/epoll.h>

#define MAX_EVENTS 1000

#define MAX_FILTERS 5


typedef unsigned char       byte;

byte GEAR_INFO_CHKSM = 0x00;
byte GEAR_INFO_COUNTER= 0x00;
byte GEAR_INFO = 0x06;
byte GEAR_INFO_DRIVE_LOGIC=0x00;
byte GEAR_INFO_ACTIVEGEAR=0x00;
bool GEAR_STATUS_PARK = false;
bool GEAR_STATUS_REVERSE = false;
bool GEAR_STATUS_DRIVE = false;
bool GEAR_ACTIVE=0x00;

int sCan0, sCan1;
struct epoll_event ev, events[MAX_EVENTS];

struct arbtest {
        int ID;
        struct timeval tval_before;
        double lastsec;
        double lastusec;

};

struct arbtest mytests[3];

/* SMGII GEAR DISPLAY
0 = Clear Screen
1 = 1st Gear
2 = 2nd Gear
3 = 3rd Gear
4 = 4th Gear
5 = Auto / D
6 = Neutral
7 = Reverse
9 = 5th Gear
10 = 6th Gear
*/


/* E46 GEAR INFO
0x00 = Neutral
0x01 = 1st gear
0x02 = 2nd gear
0x03 = 3rd gear
0x04 = 4th gear
0x05 = 5th gear
0x06 = 6th gear
0x07 = reverse gear
*/


void handle43F(struct can_frame rxframe) {
    struct can_frame txframe;


    GEAR_INFO=rxframe.data[1];

    if (GEAR_ACTIVE != rxframe.data[0] && rxframe.data[0] != 0x00 && rxframe.data[0] != 0x07) {
    switch (rxframe.data[0]) { /* ACTIVE GEAR*/
        case 0x01:
        GEAR_INFO=0x01;
        break;
        case 0x02:
        GEAR_INFO=0x02;
        break;
        case 0x03:
        GEAR_INFO=0x03;
        break;
        case 0x04:
        GEAR_INFO=0x04;
        break;
        case 0x05:
        GEAR_INFO=0x09;
        break;
        case 0x06:
        GEAR_INFO=0x10;
        break;
    }

    }

    GEAR_ACTIVE=rxframe.data[0];

    switch (rxframe.data[1]) { /* SELECTOR POSITION */
        case 0x15:    /* DRIVE */
        GEAR_INFO=0x05;
        break;
        case 0x16:    /* NUETRAL */
        GEAR_INFO=0x06;
        break;
        case 0x17:    /* REVERSE */
        GEAR_INFO=0x07;
        break;
        case 0x18:    /* PARK */
        GEAR_INFO=0; /* CLEAR SCREEN */
        break;

    }


    //Serial.print("GEAR_INFO         0x");
    //Serial.println(GEAR_INFO,HEX);

    //Serial.print("GEAR_INFO_COUNTER 0x");
    //Serial.println(GEAR_INFO_COUNTER,HEX);

    //Serial.print("GEAR_INFO_CHKSM   0x");
    //Serial.println(GEAR_INFO_CHKSM,HEX);


    GEAR_INFO_CHKSM = GEAR_INFO_COUNTER ^ GEAR_INFO;
    //Serial.print("xor               0x");
    //Serial.println(GEAR_INFO_CHKSM,HEX);

    GEAR_INFO_CHKSM ^= 0xFF;
    //Serial.print("negate            0x");
    //Serial.println(GEAR_INFO_CHKSM,HEX);

    GEAR_INFO_CHKSM = GEAR_INFO_CHKSM & 0x0F;
    //Serial.print("and 1111          0x");
    //Serial.println(GEAR_INFO_CHKSM,HEX);

    GEAR_INFO_CHKSM = GEAR_INFO_CHKSM << 4;
    //Serial.print("LSH 4             0x");
    //Serial.println(GEAR_INFO_CHKSM,HEX);

    GEAR_INFO_CHKSM = GEAR_INFO_CHKSM | GEAR_INFO_COUNTER;
    //Serial.print("or counter        0x");
    //Serial.println(GEAR_INFO_CHKSM,HEX);

    //Serial.println("");

    txframe.data[0] = 0;
    txframe.data[1] = GEAR_INFO;
    txframe.data[2] = GEAR_INFO_DRIVE_LOGIC;
    txframe.data[3] = GEAR_INFO_CHKSM;
    txframe.data[4] = 0x00;
    txframe.data[5] = 0x00;
    txframe.data[6] = 0x00;
    txframe.data[7] = 0x00;

    GEAR_INFO_COUNTER ++;
    GEAR_INFO_COUNTER = GEAR_INFO_COUNTER & 0x0F;


}

int gettestid(int checkarb) {
int x=0;
    for (x = 0;x <= 2;x++) {

        if ( checkarb == mytests[x].ID) {
//          printf("found %d\n",x);
            return x;
        }

    }
return -1;
}



int setupCanInterface(const char *caninterface) {
struct sockaddr_can addr;
struct ifreq ifr;
int s = -1;
        printf("setting up interface %s\n",caninterface);
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                perror("Socket");
                return 1;
        }

        strcpy(ifr.ifr_name, caninterface );
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                perror("Bind");
                return -1;
        }
return s;
}

void setupCanFilteration(int canSock) {
        struct can_filter rfilter[MAX_FILTERS];

        if (canSock == sCan0) {
        rfilter[0].can_id   = 0x615;
        rfilter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[1].can_id   = 0x613;
        rfilter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[2].can_id   = 0x610;
        rfilter[2].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        }

        if (canSock == sCan1) {
        rfilter[0].can_id   = 0x158;
        rfilter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[1].can_id   = 0x15F;
        rfilter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[2].can_id   = 0x316;
        rfilter[2].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[3].can_id   = 0x329;
        rfilter[3].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[4].can_id   = 0x43F;
        rfilter[4].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[5].can_id   = 0x545;
        rfilter[5].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);

        }

        setsockopt(canSock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

}


void handleCan(int canSock) {
        struct can_frame frame;
        int nbytes;
        struct timeval end;
        int i;
        nbytes = read(canSock, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
                perror("Read");
                return;
        }
        ioctl(canSock, SIOCGSTAMP, &end);

        int tstid = gettestid(frame.can_id);

        printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);

        for (i = 0; i < frame.can_dlc; i++)
            printf("%02X ",frame.data[i]);

        double delta_us = (double)(end.tv_usec - mytests[tstid].lastusec) / 1000000 + (double)(end.tv_sec - mytests[tstid].lastsec);

        printf("e: %.3f ms\n", delta_us);
        mytests[tstid].lastsec=end.tv_sec;
        mytests[tstid].lastusec=end.tv_usec;

        if (canSock == sCan0) {
        //printf("briding  can0 <-> can1\n");

        if (write(sCan1, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Bridge Write to Can1 Fail");
            return;
        }

        }
        if (canSock == sCan1) {
        //printf("briding  can1 <-> can0\n");

        if (write(sCan0, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Bridge Write to Can0 Fail");
            return;
        }

        }
        usleep(1);

}

void epollsocket(int epollfd,int canSock) {

    ev.events = EPOLLIN;
    ev.data.fd = canSock;
        if (epoll_ctl(epollfd, EPOLL_CTL_ADD, canSock, &ev) == -1) {
            perror("epoll_ctl: can0 socket");
            exit(EXIT_FAILURE);
        }
    printf("Added socket %d to ePoll.\n",canSock);
}


int main(int argc, char **argv)
{
        int i;

        //struct epoll_event ev, events[MAX_EVENTS];
        int listen_sock, conn_sock, nfds, epollfd;

        int nbytes;
        struct sockaddr_can addr;
        struct ifreq ifr;
        struct can_frame frame;
        struct timeval end;
        printf("CAN Filter 1.1\r\n");
        mytests[0].ID=0x613;
        mytests[1].ID=0x615;
        mytests[2].ID=0x610;

        sCan0=setupCanInterface("can0");
        sCan1=setupCanInterface("can1");


        epollfd = epoll_create1(0);
         if (epollfd == -1) {
            perror("epoll_create1");
            exit(EXIT_FAILURE);
        }

        epollsocket(epollfd,sCan0);
        epollsocket(epollfd,sCan1);

        while (1) {

        nfds = epoll_wait(epollfd, events, MAX_EVENTS, -1);

        if (nfds == -1) {
                perror("epoll_wait");
                exit(EXIT_FAILURE);
        }

            for (int n = 0; n < nfds; ++n) {
                handleCan(events[n].data.fd);
            }

        }

        return 0;
}
