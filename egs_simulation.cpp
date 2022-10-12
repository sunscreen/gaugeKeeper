void simulation(int sCan)
{
    std::ifstream MyReadFile("43fdump");
    std::string linedata;
    system("clear");

    while (getline (MyReadFile, linedata))
    {
        std::vector<std::string> tokens;

        for (auto i = strtok(&linedata[0], " "); i != NULL; i = strtok(NULL, " "))
        {
            tokens.push_back(i);
        }

        std::string b1str=tokens[2];
        std::string b2str=tokens[3];
        std::string b3str=tokens[4];

        b1str=trim(b1str);
        b2str=trim(b2str);
        b3str=trim(b3str);

        uint b1=std::stoi(b1str,0,16);
        uint b2=std::stoi(b2str,0,16);
        uint b3=std::stoi(b3str,0,16);

        std::bitset<8> b1bits(b1);
        mySetCursorPosition(0,0);
        std::cout<<std::hex <<"gear_report: " << "current gear: "<< b1 << " selector pos: " <<b2<< " byte: "<<b3<<"\n";

//std::cout <<std::hex<< "foo: " << b1bits << '\n';
        can_frame frame;
        can_frame outframe;
        frame.can_id=0x43F;
        frame.can_dlc=8;
        frame.data[0]=b1;
        frame.data[1]=b2;
        frame.data[2]=b3;
        switch (frame.data[1])   /* SELECTOR POSITION */
        {
        case 0x05:
            gear_filter(&frame);
            break;
        case 0x15:    /* DRIVE */
            gear_filter(&frame);
            break;

        case 0x06:
            setGear(0x06,0x01);
            GEAR_TARGET=0;
            break;
        case 0x16:    /* NUETRAL */
            setGear(0x06,0x01);
            GEAR_TARGET=0;
            break;

        case 0x07:
            setGear(0x07,0x02);
            GEAR_TARGET=0;
            break;
        case 0x17:    /* REVERSE */
            setGear(0x07,0x02);
            GEAR_TARGET=0;
            break;
        case 0x08:
            setGear(0x00,0x00);
            GEAR_TARGET=0;
            break;
        case 0x18:    /* PARK */
            setGear(0x00,0x00);
            GEAR_TARGET=0;
            break;


        }
        GEAR_INFO_CHKSM = GEAR_INFO_COUNTER ^ GEAR_INFO;
        GEAR_INFO_CHKSM ^= 0xFF;
        GEAR_INFO_CHKSM = GEAR_INFO_CHKSM & 0x0F;
        GEAR_INFO_CHKSM = GEAR_INFO_CHKSM << 4;
        GEAR_INFO_CHKSM = GEAR_INFO_CHKSM | GEAR_INFO_COUNTER;
        outframe.can_id=0x43F;
        outframe.can_dlc=8;
        outframe.data[0]=GEAR_TARGET;
        outframe.data[1]=GEAR_INFO;
        outframe.data[2]=GEAR_INFO_DRIVE_LOGIC;
        outframe.data[3]=GEAR_INFO_CHKSM;
        outframe.data[4]=0;
        outframe.data[5]=0;
        outframe.data[6]=0;
        outframe.data[7]=0;

        GEAR_INFO_COUNTER ++;
        GEAR_INFO_COUNTER = GEAR_INFO_COUNTER & 0x0F;


        if (write(sCan, &outframe, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("Write to Can0 Fail");
            return;
        }

        usleep(10000);

    }

}
