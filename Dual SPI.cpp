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
#include "ESP32CAN.h"
#include "CAN_config.h"
#include "wifipassword.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/ledc.h"

#define CAN_DLC 8


// Globals
//#define RANDOCAN = true;
//#define MASTERDEVICE = true;

#ifdef MASTERDEVICE
#define ISMASTER
#define SERVERPORT 25030
#define WIFI_DEBUGGER
#warning "-COMPILING AS MASTER-"
#else
#define ISSLAVE
#define SERVERPORT 25031
#define WIFI_DEBUGGER
#warning "-COMPILING AS SLAVE-"
#endif


#ifdef RANDOCAN
#undef ISMASTER
#undef ISSLAVE
#undef WIFI_DEBUGGER
//#define WIFI_DEBUGGER
#define SERVERPORT 25032
#endif

/* Slave variables */
#define SPI_Controller HSPI_HOST
#define SPI_Controller2 VSPI_HOST
#define TLEN 64

spi_slave_interface_config_t scfg;
spi_slave_interface_config_t scfg2;
esp_err_t spi_state;

/* Master vars */
#define SPI_CHANNEL    HSPI_HOST
#define SPI_CHANNEL2   VSPI_HOST
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


#define SD_CS 15
#define SD_SCK 14
#define SD_MISO 27
#define SD_MOSI 13

#define GEAR_SEL_AUTO 5


DMA_ATTR uint8_t myRxBuffer8[SPI_MAX_DMA_LEN] = {};
DMA_ATTR uint8_t myTxBuffer8[SPI_MAX_DMA_LEN] = {};


DMA_ATTR uint16_t myRxBuffer[SPI_MAX_DMA_LEN] = {};
DMA_ATTR uint16_t myTxBuffer[SPI_MAX_DMA_LEN] = {};
DMA_ATTR spi_slave_transaction_t trans;

spi_device_handle_t spi_handle;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;


spi_device_handle_t spi_handle2;
spi_bus_config_t buscfg2;
spi_device_interface_config_t devcfg2;


CAN_device_t CAN_cfg;               // CAN Config
unsigned long MonWifi_previousMillis=0;
const uint16_t WifiMonInterval = 5000;

const uint8_t rx_queue_size = 1;       // Receive Queue size

const char *ssid = "ASUS";
const char *serverIP="192.168.1.4";

WiFiClient client; //Declare a client object to connect to the server
boolean WifiConnected = false;
boolean DebugerConnected = false;
boolean CanReady = false;


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


unsigned long previousMillis = 0;
unsigned long previousMillis_gearchange=0;
uint8_t simulationgear=0;
uint8_t drivelogicpos=0;

byte GEAR_INFO_CHKSM = 0x00;
byte GEAR_INFO_COUNTER= 0x00;
byte GEAR_INFO = 0x06;
byte GEAR_INFO_DRIVE_LOGIC=0x00;
byte GEAR_INFO_ACTIVEGEAR=0x00;
bool GEAR_STATUS_PARK = true;
bool GEAR_STATUS_REVERSE = false; 
bool GEAR_STATUS_DRIVE = false; 


// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long interval = 20;           // interval at which to send 0x43F (milliseconds)
unsigned long interval_gearchange = 2000;


unsigned long previousMillis_blinkled = 0;
unsigned long blink_interval=250;
bool ledState;

byte current_gear_matrix_std_46[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
byte gear_selector_matrix_std_46[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};


void spi_master_config(void) {	
	  buscfg.mosi_io_num=SPI_MOSI_GPIO;
    buscfg.miso_io_num=SPI_MISO_GPIO;
    buscfg.sclk_io_num=SPI_CLK_GPIO;
    buscfg.quadhd_io_num=-1;
    buscfg.quadwp_io_num=-1;
    buscfg.max_transfer_sz=SPI_MAX_DMA_LEN;

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
    
    // Initialize and enable SPI
	  spi_state = spi_bus_initialize(SPI_CHANNEL, &buscfg, 1);
	  switch (spi_state){
        case ESP_OK:
            //Serial.printf("SPI initialsation success!\n");
            break;
        case ESP_ERR_NO_MEM:
        case ESP_ERR_INVALID_STATE:
        case ESP_ERR_INVALID_ARG:
            //Serial.printf("SPI initialsation failed!\n");
            break;

    }

  spi_state = spi_bus_add_device(SPI_CHANNEL, &devcfg, &spi_handle);

}

void spi_master_config2(void) {	
	  buscfg2.mosi_io_num=SD_MOSI;
    buscfg2.miso_io_num=SD_MISO;
    buscfg2.sclk_io_num=SD_SCK;
    buscfg2.quadhd_io_num=-1;
    buscfg2.quadwp_io_num=-1;
    buscfg2.max_transfer_sz=SPI_MAX_DMA_LEN;
    // Configuration for the SPI master interface
	  devcfg2.command_bits=0;
    devcfg2.address_bits=0;
    devcfg2.dummy_bits=0;
    devcfg2.mode=SPI_MODE;
    //devcfg.duty_cycle_pos=128;
    devcfg2.duty_cycle_pos=128;
    devcfg2.cs_ena_pretrans=0;
    devcfg2.cs_ena_posttrans= 3, // Keep the CS low 3 cycles after transaction, to stop the master from missing the last bit when CS has less propagation delay than CLK
    devcfg2.clock_speed_hz=SPI_CLOCK;
    devcfg2.spics_io_num=SD_CS;
    devcfg2.queue_size=1;
    
    devcfg2.flags |= SPI_DEVICE_HALFDUPLEX;
    
    
    // Initialize and enable SPI
	  spi_state = spi_bus_initialize(SPI_CHANNEL2, &buscfg2, 2);
	  switch (spi_state){
        case ESP_OK:
            if (client.connected()) { 
              client.printf("Secondary SPI initialsation success!\n"); 
              }

            break;
        case ESP_ERR_NO_MEM:
        case ESP_ERR_INVALID_STATE:
        case ESP_ERR_INVALID_ARG:
            if (client.connected()) {
              client.printf("Secondary SPI initialsation failed!\n");
            }
            break;
    }

  spi_state = spi_bus_add_device(SPI_CHANNEL2, &devcfg2, &spi_handle2);

}


void spi_slave_config() {
    buscfg.mosi_io_num=SPI_MOSI_GPIO;
    buscfg.miso_io_num=-1;
    buscfg.sclk_io_num=SPI_CLK_GPIO;
    
    scfg.spics_io_num=SPI_CS_GPIO;
    scfg.flags=0;
    scfg.queue_size=1;
    scfg.mode=0;
    

    spi_state = spi_slave_initialize(SPI_Controller, &buscfg, &scfg, 1);
    switch (spi_state){
        case ESP_OK:
            //printf("SPI initialsation success!\n");
            break;
        case ESP_ERR_NO_MEM:
        case ESP_ERR_INVALID_STATE:
        case ESP_ERR_INVALID_ARG:
            //printf("SPI initialsation failed!\n");
            break;

    }
  
}


void spi_slave_config2() {
    buscfg2.mosi_io_num=SD_MOSI;
    buscfg2.miso_io_num=-1;
    buscfg2.sclk_io_num=SD_SCK;
    
    scfg2.spics_io_num=SD_CS;
    scfg2.flags=0;
    scfg2.queue_size=1;
    scfg2.mode=0;
    
    spi_state = spi_slave_initialize(SPI_Controller2, &buscfg2, &scfg2, 2);
    switch (spi_state){
        case ESP_OK:
            if (client.connected()) {
              client.printf("Secondary SPI Slave initialsation success!\n");
            } 
            break;
        case ESP_ERR_NO_MEM:
        case ESP_ERR_INVALID_STATE:
        case ESP_ERR_INVALID_ARG:
        if (client.connected()) {
            client.printf("Secondary SPI Slave initialsation failed!\n");
        }
            break;
    }
  
}



void ReadSPISlave(void * param) {
    
    WORD_ALIGNED_ATTR uint16_t buffer[TLEN];
    CAN_frame_t recvframe;
    memset(buffer, 0xAA, TLEN); // fill buffer with some initial value
    trans.length=TLEN*16;
    trans.rx_buffer=buffer;
  
        spi_state = spi_slave_transmit(SPI_Controller, &trans,20 / portTICK_PERIOD_MS);
        
        switch (spi_state){
            case ESP_OK:
                //client.printf("SPI trans success!\n");                                         
                break;
            case ESP_ERR_TIMEOUT:
                return; /* try again when data is available */
            case ESP_ERR_NO_MEM:
            case ESP_ERR_INVALID_STATE:
            case ESP_ERR_INVALID_ARG:
                //client.print("SPI trans failed!!!!!!!!!!!!!!!\n");
                return;
        }

          
          
        recvframe.MsgID=buffer[0];
        recvframe.FIR.B.DLC=8;
        recvframe.FIR.B.FF = CAN_frame_std;


        if (buffer[0] == 0xAAAA) {
          //client.print("SPI SENT NOTHING!!!!!!!!!!!!!!!\n");
          return;
        }
        if (buffer[0] > 0xFFF) {
          client.print("SPI SENT OUT OF BOUND ARBID!!!!!!!!!!!!!!!!!!!!!!\n");
          return;
        }
        for (uint8_t c=0;c<8;c++) recvframe.data.u8[c]=buffer[c+1];
        
        //if (recvframe.MsgID > 4095) {return;}


        
        if (buffer[0] == 0x43F) {
            //GEAR_INFO_ACTIVEGEAR=recvframe.data.u8[0];
            GEAR_INFO=recvframe.data.u8[0];
              
              if (recvframe.data.u8[1] ==0x15) {
                  GEAR_STATUS_DRIVE=true; 
                  GEAR_INFO_DRIVE_LOGIC=0x01;
                  
              }

              if (recvframe.data.u8[1] ==0x16) { /* NUETRAL*/
                  GEAR_INFO_DRIVE_LOGIC=0x01;
                  GEAR_STATUS_DRIVE=false; 
              }

              if (recvframe.data.u8[1] ==0x17) { /* REVERSE*/
                  GEAR_INFO_DRIVE_LOGIC=0x02;
                  GEAR_STATUS_DRIVE=false; 
              }

              if (recvframe.data.u8[1] ==0x18) { /* PARK*/
                  GEAR_INFO_DRIVE_LOGIC=0x00;
                  GEAR_INFO=0x00;
                  GEAR_STATUS_DRIVE=false; 
              }
              ESP32Can.CANWriteFrame(&recvframe);

              //return;
        }
        
        
        #ifdef WIFI_DEBUGGER
        //client.printf("MSGID: 0x%03x byte0: 0x%02x byte1: 0x%02x byte2: 0x%02x byte3: 0x%02x byte4: 0x%02x byte5: 0x%02x byte6: 0x%02x byte7:0x%02x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);
          if (client.connected() ) {
          client.printf("ID: 0x%03x b0: 0x%02x b1: 0x%02x b2: 0x%02x b3: 0x%02x b4: 0x%02x b5: 0x%02x b6: 0x%02x b7:0x%02x\n",buffer[0],recvframe.data.u8[0],recvframe.data.u8[1],recvframe.data.u8[2],recvframe.data.u8[3],recvframe.data.u8[4],recvframe.data.u8[5],recvframe.data.u8[6],recvframe.data.u8[7]);
          }
        #endif
}


void ReadSPISlave2(void * param,spi_host_device_t spi_controller) {
    
    WORD_ALIGNED_ATTR uint16_t buffer[TLEN];
    CAN_frame_t recvframe;
    //WORD_ALIGNED_ATTR uint16_t buffer[TLEN];
    memset(buffer, 0xAB, TLEN); // fill buffer with some initial value
    trans.length=TLEN*16;
    trans.rx_buffer=buffer;

        spi_state = spi_slave_transmit(spi_controller, &trans,20 / portTICK_PERIOD_MS);
        
        switch (spi_state){
            case ESP_OK:
                //client.printf("SPI trans success!\n");                                         
                break;
            case ESP_ERR_TIMEOUT:
                return; /* try again when data is available */
            case ESP_ERR_NO_MEM:
            case ESP_ERR_INVALID_STATE:
            case ESP_ERR_INVALID_ARG:
                //client.print("SPI trans failed!!!!!!!!!!!!!!!\n");
                return;
        }
        
        
        recvframe.MsgID=buffer[0];
        recvframe.FIR.B.DLC=8;
        recvframe.FIR.B.FF = CAN_frame_std;
            
        if (buffer[0] == 0xABAB) {
          //client.printf("SPI SENT NOTHING!!!!!!!!!!!!!!! error was %d\n",spi_state);
          //We dont know what to do yet 
          return;
        }
        if (buffer[0] > 0xFFF) {
          if (client.connected()) {client.print("SPI SENT OUT OF BOUND ARBID!!!!!!!!!!!!!!!!!!!!!!\n"); }

          return;
        }
        for (uint8_t c=0;c<8;c++) recvframe.data.u8[c]=buffer[c+1];
        
       ESP32Can.CANWriteFrame(&recvframe);

        
        
        #ifdef WIFI_DEBUGGER        
        if (client.connected()) {
           client.printf("ID: 0x%03x b0: 0x%02x b1: 0x%02x b2: 0x%02x b3: 0x%02x b4: 0x%02x b5: 0x%02x b6: 0x%02x b7:0x%02x\n",buffer[0],recvframe.data.u8[0],recvframe.data.u8[1],recvframe.data.u8[2],recvframe.data.u8[3],recvframe.data.u8[4],recvframe.data.u8[5],recvframe.data.u8[6],recvframe.data.u8[7]);
        }        
        #endif
}



void SetupCan() {
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  
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
int32_t spi_dma_transfer_bytes16(uint16_t *data, uint16_t size,spi_device_handle_t m_spihandle) {
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
	trans_t.rxlength = 16 * size;
	trans_t.length = 16 * size;
	trans_t.flags = 0;
	trans_t.cmd = 0;
	trans_t.addr = 0;
	trans_t.user = NULL;

	// Perform transaction
	trans_result = spi_device_transmit(m_spihandle, &trans_t);
	if (ESP_OK != trans_result) {
		return -1;
	}

	return size;
}

void encodeCanValue(const short inval,char *outResponse) {
  outResponse[0] = inval >> 8;
  outResponse[1] = inval & 0xff;
}

void sendSPICan(void * params, spi_device_handle_t m_handle) {

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
        uint16_t txsize = sizeof(uint16_t) * (CAN_DLC + 1);
        spi_dma_transfer_bytes16(myTxBuffer,txsize,m_handle);
        
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
          ledcWrite(1, 256);
          ledcWrite(2, 256); 
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
          //if (canList[0][0] == 0) {send_debug("WTTTTTTTTTTTTTTTTF!!!\n");}
          //if ( WifiConnected == true && DebugerConnected) { client.printf("RECV CAN %x\n",canList[0][0] ); }
          #ifdef RANDOCAN
          //client.printf("RECV CAN %x\n",canList[0][0] );
          #endif

          #ifdef ISMASTER
        
          if (canList[0][0] == 0x615 || canList[0][0] == 0x613) {
            client.printf("can frame rejected!!..\n");
            return; /* do Not rebroadcast cluster ARBIDs */
          }
          sendSPICan((void *)&canList[0],spi_handle);
          ledcWrite(1, 255 - 65);
          ledcWrite(2, 255 - 35);
          
          #endif

          #ifdef ISSLAVE
          sendSPICan((void *)&canList[0],spi_handle2);
          #endif
      }
  }
 
 
}


uint8_t GEAR_INFO_TOPGEAR=7;
uint8_t gearmatrix[] = { 0x06,0x01,0x02,0x03,0x04,9,10,0x07};
//uint8_t drivelogic_matrix[] = {0x01,0x30,0x50,0x70,0x90,0xB0,0xD0,0x12};

//uint8_t drivelogic_matrix[] = {0x01,0x36,0x56,0x76,0x96,0xB6,0xD6,0x12}; /* SPORT EXTENDED */
//uint8_t drivelogic_matrix[] = {0x01,0x26,0x46,0x66,0x86,0xA6,0xD6}; /* SPORT NCD MODE*/
uint8_t drivelogic_matrix[] = {0x01,0x26,0x46,0x66,0x86,0xA6,0xD6,0x02}; /* REVERSE SPORT NCD MODE*/

void gearchange(void * params) {

while (1) {  
    unsigned long currentMillis_changegear = millis();
  if (currentMillis_changegear - previousMillis_gearchange >= interval_gearchange) {
    
    // change gear!
      previousMillis_gearchange = currentMillis_changegear;
      


      GEAR_INFO=gearmatrix[simulationgear];
      GEAR_INFO_DRIVE_LOGIC=drivelogic_matrix[drivelogicpos];
      
      if (drivelogicpos > GEAR_INFO_TOPGEAR) {
        drivelogicpos=0;
        GEAR_INFO_DRIVE_LOGIC=drivelogic_matrix[drivelogicpos];
      }
      

      if (simulationgear > GEAR_INFO_TOPGEAR) {
          simulationgear=0;
          GEAR_INFO=gearmatrix[simulationgear];
      }               
      

      simulationgear++;
      drivelogicpos++;
  }  
vTaskDelay(10 / portTICK_RATE_MS);
}

}

void robloop(void *params) {

  while (1) {

  unsigned long currentMillis = millis();

  // Send CAN Message
  if (currentMillis - previousMillis >= interval && CanReady == true) {
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

    tx_frame.data.u8[0] = 0;

    #ifdef GEAR_SIMULATOR
    if (simulationgear > 1) {
      bitSet(tx_frame.data.u8[0],GEAR_SEL_AUTO);
    } else {
      bitClear(tx_frame.data.u8[0],GEAR_SEL_AUTO);
    }
    #endif

   if (GEAR_STATUS_DRIVE == true) {
     bitSet(tx_frame.data.u8[0],GEAR_SEL_AUTO);
   } else {
      bitClear(tx_frame.data.u8[0],GEAR_SEL_AUTO);
   }


 


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


void devConnection() {
    if (WifiConnected == false) { return; }
    Serial.println("Try to access the server");
    if (client.connect(serverIP, SERVERPORT)) //Try to access the target address
    {
        Serial.println("Visit successful");
        #ifdef ISMASTER
        if (client.connected()) { client.printf("MASTER DEBUGER ONLINE\n"); }                    //Send data to the server        
        #else
        if (client.connected()) { client.printf("SLAVE DEBUGGER ONLINE\n"); }
        #endif
        DebugerConnected=true;

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
    uint8_t netfound=false;
    uint8_t failcount=0;
    while (netfound == false) {
      // WiFi.scanNetworks will return the number of networks found
      uint8_t n = WiFi.scanNetworks();
      Serial.println("scan done");
      if (n == 0) {
          Serial.println("no networks found");
      } else {
            Serial.print(n);
            Serial.println(" networks found");
            for (uint8_t i = 0; i < n; ++i) {
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
     
         if (WiFi.status() == WL_NO_SSID_AVAIL) { printf("WL: what no microwave radiation!?\n"); }
         if (WiFi.status() == WL_CONNECTION_LOST) { printf("WL: We lost connection to the AP\n"); }
         if (WiFi.status() == WL_CONNECT_FAILED) { printf("WL: We failed to connect to the AP\n"); }
         if (WiFi.status() == WL_DISCONNECTED) { printf("WL:We are disconnected\n"); }
        delay(500);
        failcount++;
        if (failcount > 50) { break; }
    }
    if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
    WifiConnected=true;
    } else {
      WifiConnected = false;
    }

}
  // Wait a bit before trying to connect.
  delay(1000);
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
          
          if (CanReady == true) { 
          mReadCan(); 
          ReadSPISlave2(NULL,SPI_Controller2);
          }
        #endif
             
        #ifdef ISSLAVE 
          if (CanReady == true) { 
          mReadCan(); 
          ReadSPISlave(NULL);
        }
         
        #endif

        #ifdef RANDOCAN
        if (CanReady == true ) {mReadCan();}
        vTaskDelay(5);
        #endif
      
        #ifdef WIFI_TERMINAL
        if (WifiConnected == true && DebugerConnected == true) { 
        
       //ProcessTCP();         
        
        }
        #endif
  }
delay(10);
}

void setup() {
  //Serial.begin(115200);
    pinMode(0,OUTPUT);
    pinMode(34,OUTPUT);
    pinMode(35,OUTPUT);


    //ledcAttachPin(0, LEDC_CHANNEL_0);
    ledcAttachPin(2, LEDC_CHANNEL_1);
    ledcAttachPin(34, LEDC_CHANNEL_2);
    ledcAttachPin(35, LEDC_CHANNEL_3);
    //ledcSetup(LEDC_CHANNEL_0, 0, LEDC_TIMER_12_BIT);
    //ledcSetup(LEDC_CHANNEL_1, 0, LEDC_TIMER_12_BIT);
    ledcSetup(LEDC_CHANNEL_1, 12000, 8);
    ledcSetup(LEDC_CHANNEL_2, 12000, 8);
    ledcSetup(LEDC_CHANNEL_3, 12000, 8);
    //edcSetup(LEDC_CHANNEL_2, 12000, 8);

    ledcWrite(1, 256);
    ledcWrite(2, 256);
    ledcWrite(3, 256);
    //ledcWrite(2, 0);
    //ledcWrite(2, 0);
    delay(2000);    
    ledcWrite(1, 255 - 65);
    ledcWrite(2, 255 - 35);
    ledcWrite(3, 255 - 55);
    delay(2000);    
    ledcWrite(2, 256 - 65);
    ledcWrite(1, 256 - 35);
    ledcWrite(3, 255 - 85);

    delay(2000);

    #ifdef WIFI_DEBUGGER
    scanWIFI();
    devConnection();
    #endif
    
    SetupCan();            
      
   

    #ifdef ISMASTER
    spi_master_config();              
    spi_slave_config2();
    ESP32Can.CANInit(); 
    CanReady=true; 
    
    #endif
    
    #ifdef ISSLAVE
    spi_slave_config();
    spi_master_config2();
    ESP32Can.CANInit(); 
    CanReady=true; 


    #endif
    
  
    #ifdef RANDOCAN
    ESP32Can.CANInit(); 
    CanReady=true; 
    
    xTaskCreatePinnedToCore(
                    robloop,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    4,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
    
    xTaskCreatePinnedToCore(
                    gearchange,   /* Function to implement the task */
                    "gearTask", /* Name of the task */
                    5000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    2,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
    

    #endif 

       
    
}

void loop() {
 Coreloop();
//gearchange(NULL);
}