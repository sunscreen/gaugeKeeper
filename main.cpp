#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
  #include <string.h>
  #include <Arduino.h>
#include <SD.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"

#include "ESP32CAN.h"
#include "CAN_config.h"
#include "wifipassword.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"


// You'll likely need this on vanilla FreeRTOS
//#include semphr.h

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Globals


#define MASTERDEVICE = true;

#ifdef MASTERDEVICE
#define ISMASTER

#define SERVERPORT 25030
#warning "-COMPILING AS MASTER-"
#else
#define ISSLAVE
#define SERVERPORT 25031
#warning "-COMPILING AS SLAVE-"
#endif

/* Slave variables */
#define SPI_Controller HSPI_HOST
#define TLEN 256

spi_slave_interface_config_t scfg;
esp_err_t spi_state;

/* Slave vars */
#define SPI_CHANNEL    HSPI_HOST
//#define SPI_CLOCK      20000000 // 20MHz SPI clock
#define SPI_CLOCK      SPI_MASTER_FREQ_20M

/*
 * SPI master modes for ESP32:
 * - Mode 0: CPOL = 0, CPHA = 0
 * - Mode 1: CPOL = 0, CPHA = 1
 * - Mode 2: CPOL = 1, CPHA = 1
 * - Mode 3: CPOL = 1, CPHA = 0
 */
#define SPI_MODE       0 // Default SPI mode 00

#define SPI_CLK_GPIO   GPIO_NUM_18 //
#define SPI_CS_GPIO    GPIO_NUM_12 //
#define SPI_MISO_GPIO  GPIO_NUM_19 //
#define SPI_MOSI_GPIO  GPIO_NUM_23 //


//DMA_ATTR uint16_t myRxBuffer[SPI_MAX_DMA_LEN] = {};
//DMA_ATTR uint16_t myTxBuffer[SPI_MAX_DMA_LEN] = {};

DMA_ATTR uint8_t myRxBuffer8[SPI_MAX_DMA_LEN] = {};
DMA_ATTR uint8_t myTxBuffer8[SPI_MAX_DMA_LEN] = {};


DMA_ATTR uint16_t myRxBuffer[SPI_MAX_DMA_LEN] = {};
DMA_ATTR uint16_t myTxBuffer[SPI_MAX_DMA_LEN] = {};
DMA_ATTR spi_slave_transaction_t trans;

spi_device_handle_t spi_handle;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;



CAN_device_t CAN_cfg;               // CAN Config
unsigned long MonWifi_previousMillis=0;

const int WifiMonInterval = 5000;
const int rx_queue_size = 1;       // Receive Queue size

const char *ssid = "ASUS";
const char *serverIP="192.168.1.10";

WiFiClient client; //Declare a client object to connect to the server
boolean WifiConnected = false;
boolean DebugerConnected = false;
boolean CanReady = false;
TaskHandle_t Task1;
TaskHandle_t Task2;
 
char candata[128];
char bla[128];
char hexstring[256];     
char *ptrtohex = hexstring;


#define MAX_CAN_LIST 10
uint16_t canList[MAX_CAN_LIST][10];
uint8_t currentCan = 0;
uint8_t currentMaxCan = 0;

#define MAX_CAN_SEND 10
#define CAN_FUNC 13
uint8_t canSender[MAX_CAN_SEND][CAN_FUNC];
uint8_t currentCanSender = 0;
uint8_t maxCanSender = 0;

#define MAX_SPI_LIST 10
uint8_t SPIList[MAX_SPI_LIST][10];
uint8_t currentSPI;
uint8_t currentMaxSPI;



uint32_t canTimestamp = 0;
boolean gotCan = false;
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;


byte randnum=0;
uint16_t cancounter=0x00;
uint8_t lastgear=0;

unsigned long previousMillis = 0;
unsigned long previousMillis_gearchange=0;
unsigned int simulationgear;
unsigned int drivelogicpos;

byte GEAR_INFO_CHKSM = 0x00;

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long interval = 20;           // interval at which to blink (milliseconds)
unsigned long interval_gearchange = 5000;

uint8_t clutchstatus=0;
// Globals
//static SemaphoreHandle_t mutex;
byte GEAR_INFO_COUNTER= 0x00;
byte GEAR_INFO = 0x06;
byte GEAR_INFO_DRIVE_LOGIC=0x00;
void setupSPI_slave() {
    buscfg.mosi_io_num=SPI_MOSI_GPIO;
    buscfg.miso_io_num=-1;
    buscfg.sclk_io_num=SPI_CLK_GPIO;
    
    scfg.spics_io_num=SPI_CS_GPIO;
    scfg.flags=0;
    scfg.queue_size=1;
    scfg.mode=0;
    

    GPIO.func_in_sel_cfg[SPI_CS_GPIO].sig_in_inv = 1; // CS of Display is Active High

    spi_state = spi_slave_initialize(SPI_Controller, &buscfg, &scfg, 1);
    switch (spi_state){
        case ESP_ERR_NO_MEM:
        case ESP_ERR_INVALID_STATE:
        case ESP_ERR_INVALID_ARG:
            printf("SPI initialsation failed!\n");
            break;
        case ESP_OK:
            printf("SPI initialsation success!\n");
            break;
    }
  
}

void ReadSPISlave(void * param) {
    
    WORD_ALIGNED_ATTR uint16_t buffer[TLEN];
    //WORD_ALIGNED_ATTR uint16_t buffer[TLEN];
    memset(buffer, 0xAA, TLEN); // fill buffer with some initial value
    trans.length=TLEN*16;
    trans.rx_buffer=buffer;
    GPIO.func_in_sel_cfg[SPI_CS_GPIO].sig_in_inv = 1; // CS of Display is Active High    
    
    
        spi_state = spi_slave_transmit(SPI_Controller, &trans,portMAX_DELAY);
        
        switch (spi_state){
            case ESP_ERR_NO_MEM:
            case ESP_ERR_INVALID_STATE:
            case ESP_ERR_INVALID_ARG:
                client.printf("SPI trans failed!\n");
                break;
            case ESP_OK:
                client.printf("SPI trans success!\n");                                         
                break;
        }
        
        client.printf("MSGID: 0x%03x byte0: 0x%02x byte1: 0x%02x byte2: 0x%02x byte3: 0x%02x byte4: 0x%02x byte5: 0x%02x byte6: 0x%02x byte7:0x%02x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);
}
// Initialize the SPI2 device in master mode
void SetupCan() {
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  
}

void spi_master_config(void) {	
	  buscfg.mosi_io_num=SPI_MOSI_GPIO;
    buscfg.miso_io_num=SPI_MISO_GPIO;
    buscfg.sclk_io_num=SPI_CLK_GPIO;
    buscfg.quadhd_io_num=-1;
    buscfg.quadwp_io_num=-1;
    buscfg.max_transfer_sz=SPI_MAX_DMA_LEN;
    // Configuration for the SPI master interface
	  devcfg.command_bits=0;
    devcfg.address_bits=0;
    devcfg.dummy_bits=0;
    devcfg.mode=SPI_MODE;
    devcfg.duty_cycle_pos=128;
    devcfg.cs_ena_pretrans=0;
    devcfg.cs_ena_posttrans= 3, // Keep the CS low 3 cycles after transaction, to stop the master from missing the last bit when CS has less propagation delay than CLK
    devcfg.clock_speed_hz=SPI_CLOCK;
    devcfg.spics_io_num=SPI_CS_GPIO;
    devcfg.queue_size=1;
    
    devcfg.flags |= SPI_DEVICE_HALFDUPLEX;
    
    #ifdef ISSLAVE 
    devcfg.flags |= SPI_DEVICE_NO_DUMMY ;
    #endif
    // Initialize and enable SPI
	  spi_state = spi_bus_initialize(SPI_CHANNEL, &buscfg, 1);
	  switch (spi_state){
        case ESP_ERR_NO_MEM:
        case ESP_ERR_INVALID_STATE:
        case ESP_ERR_INVALID_ARG:
            Serial.printf("SPI initialsation failed!\n");
            break;
        case ESP_OK:
            Serial.printf("SPI initialsation success!\n");
            break;
    }

  spi_state = spi_bus_add_device(SPI_CHANNEL, &devcfg, &spi_handle);

}

// Full buffer DMA transfer8
int32_t spi_dma_transfer_bytes(uint8_t *data, uint16_t size) {
	esp_err_t trans_result = ESP_OK;
	spi_transaction_t trans_t;

	// Prepare transaction parameters
	if (data == myRxBuffer8) {
		trans_t.rx_buffer = myRxBuffer;
		trans_t.tx_buffer = NULL;
	} else {
		trans_t.rx_buffer = NULL;
		trans_t.tx_buffer = myTxBuffer;
	}
	trans_t.rxlength = 0;
	trans_t.length = 8 * size;
	trans_t.flags = 0;
	trans_t.cmd = 0;
	trans_t.addr = 0;
	trans_t.user = NULL;

	// Perform transaction
	trans_result = spi_device_transmit(spi_handle, &trans_t);
	if (ESP_OK != trans_result) {
		return -1;
	}

	return size;
}
// Full buffer DMA transfer16
int32_t spi_dma_transfer_bytes16(uint16_t *data, uint16_t size) {
	esp_err_t trans_result = ESP_OK;
	spi_transaction_t trans_t;

	// Prepare transaction parameters
	if (data == myRxBuffer) {
		trans_t.rx_buffer = myRxBuffer;
		trans_t.tx_buffer = NULL;
	} else {
		trans_t.rx_buffer = NULL;
		trans_t.tx_buffer = myTxBuffer;
	}
	trans_t.rxlength = 0;
	trans_t.length = 16 * size;
	trans_t.flags = 0;
	trans_t.cmd = 0;
	trans_t.addr = 0;
	trans_t.user = NULL;

	// Perform transaction
	trans_result = spi_device_transmit(spi_handle, &trans_t);
	if (ESP_OK != trans_result) {
		return -1;
	}

	return size;
}

void encodeCanValue(const short inval,char *outResponse) {
  outResponse[0] = inval >> 8;
  outResponse[1] = inval & 0xff;
}



void send_debug(const char *mydata) {
  if (client.connected() || client.availableForWrite()) { //If it is connected or has received unread data
    //client.print(mydata);                    //Send data to the server
    client.write(mydata);
  }
}
void SendSMG2_EGSCan() {
//CAN_frame_t tx_frame;


if (WifiConnected==true && DebugerConnected == true) { 
  
 randnum++;
    if (randnum>10) {randnum=1;}

    switch(randnum) {
      case 1:
      randnum=1;
      break;
      case 2:
      randnum=2;
      break;
      case 3: 
      randnum=3;
      break;
      case 4:
      randnum=4;
      break;
      case 5:
      randnum=9;
      break;
      case 6:
      randnum=10;
      break;
      
    }

   //if (cancounter>254) {cancounter=0;}
    randnum=10;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x43F;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 10;
    tx_frame.data.u8[0] &= ~(1UL << 0);
    tx_frame.data.u8[0] &= ~(1UL << 1);
    tx_frame.data.u8[0] &= ~(1UL << 2);
    tx_frame.data.u8[0] &= ~(1UL << 3);
    tx_frame.data.u8[0] &= ~(1UL << 4);
    tx_frame.data.u8[0] &= ~(1UL << 5);
    tx_frame.data.u8[0] &= ~(1UL << 6);
    tx_frame.data.u8[0] &= ~(1UL << 7);

    tx_frame.data.u8[0] |= 1UL << 5;

    tx_frame.data.u8[1] = randnum;

    tx_frame.data.u8[2]=0x04;
    tx_frame.data.u8[2] &= ~(1UL << 0);
    tx_frame.data.u8[2] &= ~(1UL << 1);
    tx_frame.data.u8[2] &= ~(1UL << 2);
    tx_frame.data.u8[2] &= ~(1UL << 3);
    tx_frame.data.u8[2] &= ~(1UL << 4);
    tx_frame.data.u8[2] &= ~(1UL << 5);
    tx_frame.data.u8[2] &= ~(1UL << 6);
    tx_frame.data.u8[2] &= ~(1UL << 7);
    
    tx_frame.data.u8[2] |= 1UL << 2;
    tx_frame.data.u8[2] |= 1UL << 3;
    tx_frame.data.u8[2] |= 1UL << 4;
    tx_frame.data.u8[2] |= 1UL << 5;
    tx_frame.data.u8[2] |= 1UL << 6;
    tx_frame.data.u8[2] |= 1UL << 7;

 


//    tx_frame.data.u8[2]=(byte)( (tx_frame.data.u8[2] & 0xF0) | 0x06);
    //0x01 = Display off
    //0x01 = Neutal
    //0x02 = reverse
    //0x03 = little D
    //0x04 = Little D
    //0x05 = Nothing?
    //0x06 == Sport
    //0x07 == Sport 
    //0x08 == nothing
    //0x11 == N + last segment of drive logic!
    //0x12 == R + last segment of drive logic!


    //if (tx_frame.data.u8[2] > 254) {tx_frame.data.u8[2]=0x00;} 
    /* 0x00 = E */    
    /* A0 = Bank */        
    /* 0x20 = Manual */
    /* 0x40 = Sport */
    /* 0x60 Blank */
    /* 0x80 Automatic */


    //tx_frame.data.u8[2] &= ~(1UL << 4);
    //tx_frame.data.u8[2] &= ~(1UL << 5);
    //tx_frame.data.u8[2] &= ~(1UL << 6);
    //tx_frame.data.u8[2] &= ~(1UL << 7);
    
    
     uint8_t chksum = cancounter ^ randnum; //XOR
    chksum = ~chksum;
    chksum &= (uint8_t)0x0f;
    chksum =  chksum >> 4;
    chksum |=  randnum;
    
    tx_frame.data.u8[3] = chksum; /* gear checksum */


    tx_frame.data.u8[4] = 0x0E; /* Longitudinal Accelleration */
    
    tx_frame.data.u8[5] = 0x00; /* Fault indication */

    tx_frame.data.u8[5] &= ~(1UL << 2); /* TCU TYPE */
    tx_frame.data.u8[5] &= ~(1UL << 3); /* Transmission oil thermostat switch */
    tx_frame.data.u8[5] &= ~(1UL << 4); /* Gear oil overtemperature switch */
    tx_frame.data.u8[5] &= ~(1UL << 4); /* DTREINF */
    tx_frame.data.u8[5] &= ~(1UL << 4); /* DTREINF */

    tx_frame.data.u8[6] = 0xFF; /* DRIVETRAIN REENFORCEMENT 0xFF=None */

    tx_frame.data.u8[7] = 0xFF; /* Gearbox snapshot??*/


    ESP32Can.CANWriteFrame(&tx_frame);
    cancounter++;     
    //send_debug("Sending Can Frame!\n");
    
 
}


}


void sendSPICan(void * params) {
   // Release the mutex so that the creating function can finish
        //xSemaphoreGive(mutex);
        uint16_t * bla;
        bla=(uint16_t *)params;
        myTxBuffer[0]=bla[0];
        myTxBuffer[1]=bla[1];
        myTxBuffer[2]=bla[2];
        myTxBuffer[3]=bla[3];
        myTxBuffer[4]=bla[4];
        myTxBuffer[5]=bla[5];
        myTxBuffer[6]=bla[6];
        myTxBuffer[7]=bla[7];   
        myTxBuffer[8]=bla[8];   
        spi_dma_transfer_bytes16(myTxBuffer,10);
        
        //vTaskDelay(50 / portTICK_PERIOD_MS);        
        //vTaskDelete(NULL);
}


void msendCan(void * inbuf) {
        uint16_t * spibuf;
        spibuf = (uint16_t *)inbuf;
        tx_frame.FIR.B.DLC=8;
        tx_frame.FIR.B.FF=CAN_frame_std;
        tx_frame.MsgID=spibuf[0];
        tx_frame.data.u8[0]=spibuf[1];
        tx_frame.data.u8[1]=spibuf[2];
        tx_frame.data.u8[2]=spibuf[3];
        tx_frame.data.u8[3]=spibuf[4];
        tx_frame.data.u8[4]=spibuf[5];
        tx_frame.data.u8[5]=spibuf[6];
        tx_frame.data.u8[6]=spibuf[7];
        tx_frame.data.u8[7]=spibuf[8];
        ESP32Can.CANWriteFrame(&tx_frame);
}

void mReadCan() {
  //unsigned long currentMillis = millis();
  // Receive next CAN frame from queue

  if ( (CAN_cfg.rx_queue != NULL) && (uxQueueMessagesWaiting(CAN_cfg.rx_queue)) ) {
  
      if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

  
	        canList[0][0] = rx_frame.MsgID;
          canList[0][1] = rx_frame.data.u8[0];
          canList[0][2] = rx_frame.data.u8[1];
          canList[0][3] = rx_frame.data.u8[2];
          canList[0][4] = rx_frame.data.u8[3];
          canList[0][5] = rx_frame.data.u8[4];
          canList[0][6] = rx_frame.data.u8[5];
          canList[0][7] = rx_frame.data.u8[6];
          canList[0][8] = rx_frame.data.u8[7]; 
          sendSPICan((void *)&canList[0]);
      }
  }
 
 
}

void SendRpmCan() {
CAN_frame_t tx_frame;


if (WifiConnected==true && DebugerConnected == true) { 
  
   
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x316;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] |= 1UL << 0;
    tx_frame.data.u8[0] |= 1UL << 1;
    tx_frame.data.u8[0] |= 1UL << 2;
    tx_frame.data.u8[0] |= 1UL << 3;
    

    tx_frame.data.u8[0] = 0x09;
    tx_frame.data.u8[1] = 0x09;
    tx_frame.data.u8[2] = 0x09;
    tx_frame.data.u8[3] = 0x09;
    tx_frame.data.u8[4] = 0xFB;
    tx_frame.data.u8[5] = 0x05;
    tx_frame.data.u8[6] = 0x06;
    tx_frame.data.u8[7] = 0x07;
    ESP32Can.CANWriteFrame(&tx_frame);
   
    send_debug("Sending Can Frame!\n");
    delay(10);
 
}

}
int randInRange( int min, int max )
{
  double scale = 1.0 / (RAND_MAX + 1);
  double range = max - min + 1;
  return min + (int) ( rand() * scale * range );
}

void sendARBID(uint32_t ARBID,byte m_byte0,byte m_byte1,byte m_byte2,byte m_byte3,byte m_byte4,byte m_byte5,byte m_byte6,byte m_byte7) {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = ARBID;
    tx_frame.FIR.B.DLC = 8;
    
    tx_frame.data.u8[0] = m_byte0;
    tx_frame.data.u8[1] = m_byte1;
    tx_frame.data.u8[2] = m_byte2; 
    tx_frame.data.u8[3] = m_byte3;
    tx_frame.data.u8[4] = m_byte4;
    tx_frame.data.u8[5] = m_byte5;
    tx_frame.data.u8[6] = m_byte6;
    tx_frame.data.u8[7] = m_byte7;

    ESP32Can.CANWriteFrame(&tx_frame);
    vTaskDelay(10 / portTICK_RATE_MS);   

}

uint8_t gearmatrix[] = { 0x06,0x01,0x02,0x03,0x04,9,10};
uint8_t drivelogic_matrix[] = {0x00,0x30,0x50,0x70,0x90,0xB0,0xD0};
void gearchange(void * params) {
while (1) {
    unsigned long currentMillis_changegear = millis();
  if (currentMillis_changegear - previousMillis_gearchange >= interval_gearchange) {
    
    // change gear!
      previousMillis_gearchange = currentMillis_changegear;
      simulationgear++;
      drivelogicpos++;
      GEAR_INFO=gearmatrix[simulationgear];
      GEAR_INFO_DRIVE_LOGIC=drivelogic_matrix[drivelogicpos];
      
      if (drivelogicpos > 6) {
        drivelogicpos=0;

      }
      

      if (simulationgear > 5) {
          simulationgear=0;
          GEAR_INFO=gearmatrix[simulationgear];
      }          
    
  }  
vTaskDelay(10 / portTICK_RATE_MS);
}

}

void robloop(void *params) {

  while (1) {

  unsigned long currentMillis = millis();

  // Send CAN Message
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    CAN_frame_t tx_frame;
    
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x43F;
    tx_frame.FIR.B.DLC = 8;

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

    tx_frame.data.u8[0] = 0x00;
    bitSet(tx_frame.data.u8[0],5);
    tx_frame.data.u8[1] = GEAR_INFO;
    tx_frame.data.u8[2] = GEAR_INFO_DRIVE_LOGIC; 
    tx_frame.data.u8[3] = GEAR_INFO_CHKSM; 
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x00;
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;

    GEAR_INFO_COUNTER ++;
    GEAR_INFO_COUNTER = GEAR_INFO_COUNTER & 0x0F;
    
    ESP32Can.CANWriteFrame(&tx_frame);
  }
  vTaskDelay(5);
  }


}

void SendEGSCan2( void * params) {
    byte GEAR_INFO_CHKSM = 0x00;
    
    byte GEAR_INFO_START = 0x05;
    long sentgears=0;
    CAN_frame_t tx_frame;

    int rpm=1000;
    while (1) {
    
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x43F;
    tx_frame.FIR.B.DLC = 8;
    
    
    
    
    GEAR_INFO_CHKSM = GEAR_INFO_COUNTER ^ GEAR_INFO;
    GEAR_INFO_CHKSM = ~ GEAR_INFO_CHKSM;
    GEAR_INFO_CHKSM = GEAR_INFO_CHKSM & 0x0F;
    GEAR_INFO_CHKSM = GEAR_INFO_CHKSM << 4;
    GEAR_INFO_CHKSM = GEAR_INFO_CHKSM | GEAR_INFO;
    
    GEAR_INFO_COUNTER ++;
    GEAR_INFO_COUNTER = GEAR_INFO_COUNTER & 0x0F;

    tx_frame.data.u8[0] = GEAR_INFO_START;
    
    //bitSet(tx_frame.data.u8[0],5);
    
    bitSet(tx_frame.data.u8[0],3);
    
    bitSet(tx_frame.data.u8[0],5);
    bitSet(tx_frame.data.u8[0],7);
    bitClear(tx_frame.data.u8[0],6);


    

    tx_frame.data.u8[1] = GEAR_INFO;
    tx_frame.data.u8[2] = 0; 
    tx_frame.data.u8[3] = GEAR_INFO_CHKSM;
    tx_frame.data.u8[4] = 0x7E;
    tx_frame.data.u8[5] = 0x84;
    tx_frame.data.u8[6] = 0xE4;
    tx_frame.data.u8[7] = 0xFE;

    //SendRpmCan();
    //sendARBID(0x153,0x00,0x48,0x00,0xFF,0x00,0xFF,0xFF,0x00);
   sendARBID(0x329,0x80,0x64, 0xCF, 0x04, 0x00, 0x00, 0x00,0x00);
   if (sentgears < 128) {
   // sendARBID(0x329,0x80,0x64, 0xCF, 0x04, 0x00, 0x00, 0x00,0x00);
    //sendARBID(0x329,0x80,0x32, 0xCF, 0x04, 0x00, 0x00, 0x00,0x00);
    sentgears++;
    }
    if (sentgears >= 128) {
    sentgears=0;
    
    }
    
    //ESP32Can.CANWriteFrame(&tx_frame);
    //vTaskDelay(20 / portTICK_RATE_MS);    
    //robloop();

    sendARBID(0x545,0x02,0x00, 0x00, 0x60, 0x4A, 0x00, 0x00,0x00);
    rpm=rpm+200;
    if (rpm >9999) {
      rpm=1000;
      GEAR_INFO++;
      if (GEAR_INFO > 8) {GEAR_INFO=1;}
      if (GEAR_INFO == 5 || GEAR_INFO == 6 || GEAR_INFO == 7 || GEAR_INFO == 8) {
        GEAR_INFO=9;

      }  
      
    }

    int number = rpm / 1000;
		byte txdata;
    number = number * 16;
		// Set Bits according to Bitmap
		for(int n = 4; n < 7; n++){
			int bit = (number >> n) & 1U;
			txdata ^= (-bit ^ txdata) & (1UL << n);
		}

    sendARBID(0x316,0x0D, 0x00, 0xAE, txdata, 0x56, 0x34, 0x00,0x00);

    //sendARBID(0x1F0,0x09,0x60,0x09,0x00,0x09,0x00,0x09,0x08);
    sendARBID(0x1F3,0x09,0x60,0x09,0x00,0x09,0x00,0x09,0x08);
    //sendARBID(0x1F5,0x42,0x80,0x00,0x00,0x80,0x11,0x09,0x08);
    sendARBID(0x153,0x00,0x48,0x00,0xFF,0x00,0xFF,0xFF,0x80);
    //sendARBID(0x316,0x0D, 0x09, 0x09, 0x00, 0x56, 0x34, 0x00,0x00);
    
    //sendARBID(0x43D,0x03,0x05,0x00,0xFF,0x00,0xFF,0xFF,0xFF);
    }
}


void SendEGSCan(void * param) {

    while (1) {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x43F;
    tx_frame.FIR.B.DLC = 8;
    //tx_frame.FIR.B.RTR=CAN_RTR;
    //tx_frame.data.u8[0] =  random(255); /* random desired gear (0x0 we want nuetral) while LVS = 0 */
    randnum=0x05;

    tx_frame.data.u8[0]= randnum;

    bitWrite(tx_frame.data.u8[0],3,1); /*LVS = 0*/
    //bitWrite(tx_frame.data.u8[0],4,0); /*LVS = 0*/
    //bitWrite(tx_frame.data.u8[0],5,1); /*LVS = 0*/
    
  
    tx_frame.data.u8[1] = randnum;
    
    
    tx_frame.data.u8[2]=0xA0; /* some of the bits in this control the small drive logic icons.. other random in byte0 makes it flash */
                                /* 0x20 - 0xA0 DRIVE LOGIC SCREENS INC DSP EXTRA FRAME*/

    bitWrite(tx_frame.data.u8[2],0,0); /*GEAR SEL POSITION= 1*/                                
    
    uint8_t CHKSM_GEAR_INFO = 0x00;
    CHKSM_GEAR_INFO = cancounter^randnum;
    CHKSM_GEAR_INFO = ~CHKSM_GEAR_INFO;
    CHKSM_GEAR_INFO = CHKSM_GEAR_INFO & 0x0F;
    CHKSM_GEAR_INFO = CHKSM_GEAR_INFO << 4; //<-- 0x4 ?
    CHKSM_GEAR_INFO = CHKSM_GEAR_INFO | randnum;

    tx_frame.data.u8[3] = CHKSM_GEAR_INFO;

    tx_frame.data.u8[4] = 0x50; /* ACCELEROMETER very light lol*/
    
    
    tx_frame.data.u8[5] = 0x00; /* SMG ERROR STATUS */
    

    
    tx_frame.data.u8[6] = 0x00;  /* DRIVE TRAIN REINFORCEMENT */

    tx_frame.data.u8[7] = 0x00;  /* TQ_CLU TORQUE ON CLUTCH */

    // Create mutex before starting tasks

    // Take the mutex
    
    ESP32Can.CANWriteFrame(&tx_frame);
     // Release the mutex so that the creating function can finish
    
    cancounter++;
    //client.printf("going to gear %x checksum is %x\n",randnum,tx_frame.data.u8[3]);
    
    //mReadCan();
    //delay(1);
    vTaskDelay(10 / portTICK_RATE_MS);
    }
    
}


void devConnection() {
    Serial.println("Try to access the server");
    if (client.connect(serverIP, SERVERPORT)) //Try to access the target address
    {
        Serial.println("Visit successful");
        #ifdef ISMASTER
        client.printf("MASTER DEBUGER ONLINE\n");                    //Send data to the server        
        #else
        client.printf("SLAVE DEBUGGER ONLINE\n");
        #endif

        DebugerConnected=true;
        // Init CAN Module        
        CanReady = true;
    }
    else
    {
        Serial.println("Access failed");        
        client.stop(); //Close the client
    }
    delay(1000);
}

void scanWIFI() {
  Serial.println("scan start");
    int netfound=false;
    while (netfound == false) {
      // WiFi.scanNetworks will return the number of networks found
      int n = WiFi.scanNetworks();
      Serial.println("scan done");
      if (n == 0) {
          Serial.println("no networks found");
      } else {
            Serial.print(n);
            Serial.println(" networks found");
            for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            /*Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));*/

            if (WiFi.SSID(i) == "ASUS") {netfound = true; break;}
            /* Serial.print(WiFi.RSSI(i)); */
            /* Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*"); */
            delay(10);
            }
            //if (netfound ==true ) break;
      }
    Serial.println("Connected!");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); //Turn off wifi sleep in STA mode to improve response speed
       
    WiFi.begin(ssid, WIFIPASSWORD);
    
    while (WiFi.status() != WL_CONNECTED)
    {
     
         if (WiFi.status() == WL_NO_SSID_AVAIL) {printf("WL: what no microwave radiation!?\n"); }
         if (WiFi.status() == WL_CONNECTION_LOST) {printf("WL: We lost connection to the AP\n"); }
         if (WiFi.status() == WL_CONNECT_FAILED) {printf("WL: We failed to connect to the AP\n"); }
         if (WiFi.status() == WL_DISCONNECTED) {printf("WL:We are disconnected\n"); }
        delay(500);
        
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
    WifiConnected=true;
    Serial.println("big delay for slow wifi\n");

}
  // Wait a bit before trying to connect.
  delay(4000);
}
void ProcessTCP() {
    if (client.connected() || client.available()) //If it is connected or has received unread data
        {
            if (client.available()) //If there is data to read
            {
                String line = client.readStringUntil('\n'); //Read data to newline
                Serial.print("Read data:");
                Serial.println(line);
                
                //client.write(line.c_str()); //Send the received data back
            }
        } else {
          Serial.println("CONNECTION PROBLEM...");
        }
}

void Coreloop() {
  
    while (1) {      
        
        #ifdef ISMASTER         
          if (CanReady == true) { mReadCan(); }
        #endif
             
        #ifdef ISSLAVE 
          ReadSPISlave(NULL);
        #endif

        

        if (WifiConnected == true && DebugerConnected == true) { 
        
          #ifdef ISMASTER
 //         SendSMG2_EGSCan();
          #endif
          //ProcessTCP();         
      
        }
        
  }
delay(10);
}

void setup() {
    vTaskDelay(3000);
    
    Serial.begin(115200);
    //scanWIFI();
    //devConnection();
    
    #ifdef ISMASTER
    spi_master_config();
    SetupCan();            
    ESP32Can.CANInit();   
     
   
    CanReady=true; 
    #endif
    
    #ifdef ISSLAVE
    setupSPI_slave();    
    #endif
   xTaskCreatePinnedToCore(
                    robloop,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    2,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
    return;
    
    xTaskCreatePinnedToCore(
                    SendEGSCan2,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    2,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
}

void loop() {
 
  //Coreloop();
gearchange(NULL);
}