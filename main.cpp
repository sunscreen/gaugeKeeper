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
#define SPI_CLOCK      20000000 // 20MHz SPI clock

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
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
unsigned long MonWifi_previousMillis=0;
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
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


void setupSPI_slave() {
    buscfg.mosi_io_num=SPI_MOSI_GPIO;
    buscfg.miso_io_num=-1;
    buscfg.sclk_io_num=SPI_CLK_GPIO;
    
    scfg.spics_io_num=SPI_CS_GPIO;
    scfg.flags=0;
    scfg.queue_size=1;
    scfg.mode=0;
    
    //Serial.begin(115200);
    //delay(5000);
    
    

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
    //delay(500);

}

void ReadSPISlave(void * param) {
    
    
    WORD_ALIGNED_ATTR uint16_t buffer[TLEN];
    memset(buffer, 0xAA, TLEN); // fill buffer with some initial value
    trans.length=TLEN*16;
    trans.rx_buffer=buffer;
    GPIO.func_in_sel_cfg[SPI_CS_GPIO].sig_in_inv = 1; // CS of Display is Active High    

//    while (1) {
    
    
    
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
        
        client.printf("byte0: 0x%03x byte1: 0x%02x byte2: 0x%02x byte3: 0x%02x byte4: 0x%02x byte5: 0x%02x byte6: 0x%02x byte7: 0x%02x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
}
// Initialize the SPI2 device in master mode
void SetupCan() {
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  
}

void spi_master_config(void) {
	// Configuration for the SPI bus
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

// Full buffer DMA transfer
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
// Full buffer DMA transfer
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
        spi_dma_transfer_bytes16(myTxBuffer,8 * 16);
        
        //vTaskDelay(50 / portTICK_PERIOD_MS);        
        //vTaskDelete(NULL);
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
        
          ProcessTCP();         
      
        }
        if (!client.connected()) {
          Serial.println();
          Serial.println("disconnecting.");
          client.stop();
        }
        
  }
delay(500);
}

void setup() {
    vTaskDelay(5000);
    
    Serial.begin(115200);
    scanWIFI();
    devConnection();
    
    #ifdef ISMASTER
    spi_master_config();
    SetupCan();        
    CanReady=true; 
    ESP32Can.CANInit();   
    #endif
    
    #ifdef ISSLAVE
    setupSPI_slave();    
    #endif
}

void loop() {
  Coreloop();
}