#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SoftwareSerial.h>

#include <WiFi.h>
#include <SD.h>
#include "wifipassword.h"

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
unsigned long MonWifi_previousMillis=0;
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int WifiMonInterval = 5000;
const int rx_queue_size = 10;       // Receive Queue size

const char *ssid = "ASUS";
//const char *password = "x";
const char *serverIP="192.168.1.9";

//const IPAddress serverIP(192,168,1,9); //Address to visit
uint16_t serverPort = 25030;         //Server port number
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

#define MAX_CAN_LIST 50
uint16_t canList[MAX_CAN_LIST][10];
uint8_t currentCan = 0;
uint8_t currentMaxCan = 0;

#define MAX_CAN_SEND 50
#define CAN_FUNC 13
uint16_t canSender[MAX_CAN_SEND][CAN_FUNC];
uint8_t currentCanSender = 0;
uint8_t maxCanSender = 0;

uint32_t canTimestamp = 0;
boolean gotCan = false;
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;

void decodeCanValue(const char *testResponse,short outval) {
//char testResponse[2] = {0x4B, 0x6B};
unsigned int tmp = (unsigned)testResponse[0] << 8 | (unsigned)testResponse[1];
outval = tmp;  // implementation-defined behaviour
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

void printCan() {

  const char *label_id="ID: ";
  const char *label_data=" Data: ";
	uint16_t offset = 9;
	for(uint8_t	i = 0; i < currentMaxCan; i++) {
		if(canList[currentCan][0] == canList[i][0]) {
			offset += i*9;
			break;
		}
	}
  

  memset(hexstring,0,sizeof(hexstring));
	send_debug (label_id);
	
  snprintf(hexstring,sizeof(hexstring), "0x%03x", canList[currentCan][0]); 
	//send_debug(canList[currentCan][0], HEX);
  send_debug(ptrtohex);

	send_debug(label_data);
	for(uint8_t i = 0; i < canList[currentCan][9]; i++) {

		send_debug(" ");

    memset(hexstring,0,sizeof(hexstring));
    snprintf(hexstring,sizeof(hexstring), "0x%02x", canList[currentCan][i+1]); 
		send_debug(ptrtohex);
    //send_debug(canList[currentCan][i+1], HEX);
	}
	send_debug("\n");
}


void SetupCan() {
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  
}


void sendCan() {
	for(uint8_t canIterator = 0; canIterator < maxCanSender; canIterator++) {
		if(canSender[canIterator][0]) {
			canSender[canIterator][0]--;
			tx_frame.FIR.B.FF = CAN_frame_std;
			tx_frame.MsgID = canSender[canIterator][1];
			tx_frame.FIR.B.DLC = canSender[canIterator][2];
			for(uint8_t i = 0; i < canSender[canIterator][2]; i++) {
				tx_frame.data.u8[i] = canSender[canIterator][4+i];
			}
			ESP32Can.CANWriteFrame(&tx_frame);
			delay(canSender[canIterator][3]);
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


void SendEGSCan() {
CAN_frame_t tx_frame;


if (WifiConnected==true && DebugerConnected == true) { 
  
   
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x43F;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0x00;
    tx_frame.data.u8[0] &= ~(1UL << 0);
    tx_frame.data.u8[0] &= ~(1UL << 1);
    tx_frame.data.u8[0] &= ~(1UL << 2);
    tx_frame.data.u8[0] &= ~(1UL << 3);
    tx_frame.data.u8[0] &= ~(1UL << 4);
    tx_frame.data.u8[0] &= ~(1UL << 5);
    tx_frame.data.u8[0] &= ~(1UL << 6);
    tx_frame.data.u8[0] &= ~(1UL << 7);

    
    tx_frame.data.u8[1] = 0x08;

    tx_frame.data.u8[2]=0xFF; /*smg gear selector possition */
    //if (tx_frame.data.u8[2] > 254) {tx_frame.data.u8[2]=0x00;} 
    /* 0x00 = E */    
    /* A0 = Bank */        
    /* 0x20 = Manual */
    /* 0x40 = Sport */
    /* 0x60 Blank */
    /* 0x80 Automatic */
    
    tx_frame.data.u8[3] = 0xFF; /* torque reduction 0xFF=None */


    tx_frame.data.u8[4] = 0x00; /* output speed */
    tx_frame.data.u8[5] = 0x80; /* Fault indication */

    tx_frame.data.u8[5] &= ~(1UL << 2); /* TCU TYPE */
    tx_frame.data.u8[5] &= ~(1UL << 3); /* Transmission oil thermostat switch */
    tx_frame.data.u8[5] &= ~(1UL << 4); /* Gear oil overtemperature switch */
    tx_frame.data.u8[5] &= ~(1UL << 4); /* DTREINF */
    tx_frame.data.u8[5] &= ~(1UL << 4); /* DTREINF */

    tx_frame.data.u8[6] = 0xFF; /* DRIVETRAIN REENFORCEMENT 0xFF=None */

    tx_frame.data.u8[7] = 0x00; /* Gearbox snapshot??*/


    ESP32Can.CANWriteFrame(&tx_frame);
   
    send_debug("Sending Can Frame!\n");
    delay(10);
 
}

}


void mReadCan() {


  //unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

   gotCan = true;
		boolean newCan = true;
		for(uint8_t i = 0; i < currentMaxCan && i < MAX_CAN_LIST; i++) {
			if(canList[i][0] == rx_frame.MsgID) {
				newCan = false;
				currentCan = i;
				break;
			}
		}
		if(newCan) canList[currentCan = currentMaxCan++][0] = rx_frame.MsgID;
		
		if(rx_frame.FIR.B.RTR != CAN_RTR) {
			if(rx_frame.FIR.B.FF == CAN_frame_std) {
				canList[currentCan][9] = (uint8_t) rx_frame.FIR.B.DLC;
				for(uint8_t i = 0; i < canList[currentCan][9]; i++){
					canList[currentCan][i+1] = rx_frame.data.u8[i];
				}
			} else {
				canList[currentCan][9] = (uint8_t) rx_frame.FIR.B.DLC*4;
				for(uint8_t i = 0; i < canList[currentCan][9]; i++){
					uint8_t j = i > 3 ? 1 : 0;
					canList[currentCan][i+1] = ((uint8_t *)&rx_frame.data.u32[j])[i - j*4];
				}
			}
		}		
  
  }
  //sendCan();
  
  if(gotCan) { 
    printCan();
    SendEGSCan();
  }
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
        }
}

void Coreloop() {
  //printf("yo\n");
  while (1) {
      if (WifiConnected==true && DebugerConnected == true) { 
        
        ProcessTCP(); 
        if (CanReady == true) { mReadCan(); }
        
      //printf("looping\n");
      }
  
  }
delay(500);
}


void devConnection() {
    Serial.println("Try to access the server");
    if (client.connect(serverIP, serverPort)) //Try to access the target address
    {
        Serial.println("Visit successful");

        client.print("DEBUGER ONLINE\n");                    //Send data to the server
        
        DebugerConnected=true;
        // Init CAN Module        
        CanReady = true;
    }
    else
    {
        Serial.println("Access failed");        
        client.stop(); //Close the client
    }
    
}




void MonWifi(void * pvParameters ) {
  while (1) {
  int MonWifi_currentMillis=millis();
  if (MonWifi_currentMillis - MonWifi_previousMillis >= WifiMonInterval) {  
  MonWifi_previousMillis=MonWifi_currentMillis;
  if (WiFi.status() == WL_NO_SSID_AVAIL) {printf("what no microwave radiation!?\n"); }
  if (WiFi.status() == WL_CONNECTION_LOST) {printf("We lost connection to the AP\n"); }
  if (WiFi.status() == WL_CONNECT_FAILED) {printf("We failed to connect to the AP\n"); }
  if (WiFi.status() == WL_DISCONNECTED) {printf("We are disconnected\n"); }   
  if (WiFi.status() == WL_CONNECTED) {printf("WIFI Connection OK.\n"); }   
  //if (WiFi.status() == WL_CONNECTED && DebugerConnected == false ) { devConnection(); }   
  }
  vTaskDelay(100);
  }  
}

void loop(){

Coreloop();
}

void setup()
{
   
    Serial.begin(115200);
    Serial.println();
    delay(10000); /* give us a chance to jump on the serial connection */
    Serial.println("scan start");
    int netfound=false;
    while (1) {
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

            if (WiFi.SSID(i) == "ASUS") {netfound =true; break;}
            /* Serial.print(WiFi.RSSI(i)); */
            /* Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*"); */
            delay(10);
            }
            if (netfound ==true ) break;
      }
    Serial.println("");

}
  // Wait a bit before trying to connect.
  delay(5000);

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
    delay(5000);
    SetupCan();
    devConnection();
    ESP32Can.CANInit();   




 
 }







