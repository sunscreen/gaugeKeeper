
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

struct arbtest {
        int ID;
        struct timeval tval_before;
        double lastsec;
        double lastusec;

};

struct arbtest mytests[3];

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




int main(int argc, char **argv)
{
        int s, i;
        int nbytes;
        int prevARBID=0;
        struct sockaddr_can addr;
        struct ifreq ifr;
        struct can_frame frame;
        struct timeval tval_before;
        struct timeval begin, end;
        struct timeval tv;
        printf("CAN Filter 1.1\r\n");
        mytests[0].ID=0x613;
        mytests[1].ID=0x615;
        mytests[2].ID=0x610;

        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                perror("Socket");
                return 1;
        }

        strcpy(ifr.ifr_name, "can0" );
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                perror("Bind");
                return 1;
        }

        struct can_filter rfilter[3];

        rfilter[0].can_id   = 0x615;
        rfilter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[1].can_id   = 0x613;
        rfilter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
        rfilter[2].can_id   = 0x610;
        rfilter[2].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);


        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        while (1) {

        nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
                perror("Read");
                return 1;
        }
        ioctl(s, SIOCGSTAMP, &end);

        int tstid = gettestid(frame.can_id);



        printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);

        for (i = 0; i < frame.can_dlc; i++)
                printf("%02X ",frame.data[i]);

        double delta_us = (double)(end.tv_usec - mytests[tstid].lastusec) / 1000000 + (double)(end.tv_sec - mytests[tstid].lastsec);

        printf("e: %.3f ms\n", delta_us);
        mytests[tstid].lastsec=end.tv_sec;
        mytests[tstid].lastusec=end.tv_usec;

        usleep(1);
        }

        return 0;
}
