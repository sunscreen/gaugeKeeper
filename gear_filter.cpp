byte LAST_GEAR=0x00;
void gear_filter(struct can_frame *rxframe)
{
    GEAR_INFO_DRIVE_LOGIC=0;

    if ((rxframe->data[0] != 0x00) && (rxframe->data[0] != 0x09) && (rxframe->data[0] != 0x0A) && (rxframe->data[0] != 0x0B) && (rxframe->data[0] != 0x0C) && (rxframe->data[0] != 0x4D) && (rxframe->data[0] != 0x4C))
    {
        GEAR_INFO=rxframe->data[0];
//          CheckLVGS(rxframe);

        if (GEAR_INFO == 0x43)
        {
            GEAR_INFO=0x03;
        }
        if (GEAR_INFO == 0x44)
        {
            GEAR_INFO=0x04;
        }
        if (GEAR_INFO == 0x45)
        {
            GEAR_INFO=0x09;
        }
        if (GEAR_INFO == 0x05)
        {
            GEAR_INFO=0x09;    /* SMGII conversion */
        }

        printf("Gear Change to %u th\n",GEAR_INFO);
    }


    GEAR_TARGET |= 1UL << 5; /* set bit5 for small D */
    LAST_GEAR = GEAR_INFO;
    return;
}
