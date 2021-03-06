#include "LoRaWan_APP.h"
#include <Region.h>
#include "Arduino.h"
#include "Seeed_BME280.h"


/*
   Define your Settings below
*/

#define BME_280 1
#define BMP_280 0


const char myDevEui[] = { 0x00, 0x89, 0x76, 0xE8, 0xD7, 0xD2, 0x76, 0x1C } ;
const char myAppEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x7E, 0xF2 };
const char myAppKey[] = { 0xFA, 0x54, 0x0C, 0x83, 0xC6, 0xF8, 0xAA, 0xC0, 0xE0, 0x0A, 0x8B, 0xC3, 0x89, 0x24, 0x54, 0x92 };

// The interrupt pin is attached to D4/GPIO1
#define INT_PIN GPIO1
bool accelWoke = false; //setting to true denotes the water-leak


#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#ifndef AT_SUPPORT
#define AT_SUPPORT 0
#endif

#ifndef ACTIVE_REGION
#define ACTIVE_REGION LORAMAC_REGION_EU868
#endif

#ifndef LORAWAN_CLASS
#define LORAWAN_CLASS CLASS_A
#endif

#ifndef LORAWAN_NETMODE
#define LORAWAN_NETMODE 0
#endif

#ifndef LORAWAN_ADR
#define LORAWAN_ADR 1
#endif

#ifndef LORAWAN_Net_Reserve
#define LORAWAN_Net_Reserve 1
#endif

DeviceClass_t  CLASS = LORAWAN_CLASS;

/*OTAA or ABP*/
bool OVER_THE_AIR_ACTIVATION = LORAWAN_NETMODE;

/* LoRaWAN Adaptive Data Rate */
bool LORAWAN_ADR_ON = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool KeepNet = LORAWAN_Net_Reserve;

/*LoraWan REGION*/
LoRaMacRegion_t REGION = ACTIVE_REGION;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool IsTxConfirmed = false;

uint8_t ConfirmedNbTrials = 8;

/* Application port */
#define DEVPORT 2  // Used inside the preparingthedata to swith which data to send. Data will be send in normal state 
#define APPPORT 1   // Data will be send when an interuppt occurs.

uint8_t AppPort = 1;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t APP_TX_DUTYCYCLE = (600000); // send every 10 mints Formuala divide the time value by 60000

//uint32_t APP_TX_DUTYCYCLE = (24 * 60 * 60 * 1000); // 24h


//bool BME_280_e[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // 2
uint8_t sensortype = 0;
float Temperature, Humidity, Pressure, lux, co2, tvoc;
BME280 bme280;

/* Prepares the payload of the frame */
static bool prepareTxFrame( uint8_t port )
{
  AppDataSize = 0; //AppDataSize max value is 64
  int pnr = 0;
  AppPort = port;
  switch (port) {
    case 1: // woke up from interrupt- Water leak detected
      Serial.println("Sending data packet");
        AppData[AppDataSize++] = pnr; // First byte
        AppData[AppDataSize++] = 1; // second byte
      AppData[AppDataSize++] = 0x01; // Third byte Send the decimal one to denotes the leak
      break;
    case 2: //  send environmental data
      Serial.println("Sending environmental data");
      /*
        BME280
      */
      

   
        sensortype = 2;


        if (!bme280.init())
        {
          Serial.println("  BME280 error!");
        }
        delay(1000);
        Temperature = bme280.getTemperature();
        Pressure = bme280.getPressure() / 100.0;
        Humidity = bme280.getHumidity();

        AppData[AppDataSize++] = pnr;
        AppData[AppDataSize++] = 2;

        AppData[AppDataSize++] = (uint8_t)((int)((Temperature + 100.0) * 10.0) >> 8);
        AppData[AppDataSize++] = (uint8_t)((int)((Temperature + 100.0) * 10.0));

        AppData[AppDataSize++] = (uint8_t)((int)(Humidity * 10.0) >> 8);
        AppData[AppDataSize++] = (uint8_t)((int)(Humidity * 10.0));

        AppData[AppDataSize++] = (uint8_t)((int)(Pressure * 10.0) >> 8);

        AppData[AppDataSize++] = (uint8_t)((int)(Pressure * 10.0));

        Serial.print("  BME280: T = ");
        Serial.print(Temperature);
        Serial.print("C, RH = ");
        Serial.print(Humidity);
        Serial.print(" %, Pressure = ");
        Serial.print(Pressure);
        Serial.println(" hPA");
      break;
}
uint16_t BatteryVoltage = GetBatteryVoltage();
  AppData[AppDataSize++] = (uint8_t)(BatteryVoltage >> 8);
  AppData[AppDataSize++] = (uint8_t)BatteryVoltage;
  Serial.print("BatteryVoltage: ");
  Serial.print(BatteryVoltage);
  Serial.println();
  return true;
}

extern uint8_t DevEui[];
extern uint8_t AppEui[];
extern uint8_t AppKey[];
extern bool IsLoRaMacNetworkJoined;

void accelWakeup()
{
  accelWoke = true;
}

void setup() {
   Serial.begin(115200);
     Wire.begin();  
   pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(500);
 

  delay(200); // wait for stable
  accelWoke = false;
  memcpy(DevEui, myDevEui, sizeof(myDevEui));
  memcpy(AppEui, myAppEui, sizeof(myAppEui));
  memcpy(AppKey, myAppKey, sizeof(myAppKey));



  BoardInitMcu();

  DeviceState = DEVICE_STATE_INIT;
  LoRaWAN.Ifskipjoin();

  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, accelWakeup, RISING);
  Serial.println("Interrupts attached");
}

void loop()
{
  if (accelWoke) {
    uint32_t now = TimerGetCurrentTime();
    Serial.print(now); Serial.println("Water leak detected... inside loop");
  }

  switch ( DeviceState )
  {
    case DEVICE_STATE_INIT:
      {
        Serial.printf("LoRaWan Class%X test start! \r\n", CLASS + 10);
#if(AT_SUPPORT)
        Enable_AT();
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.Init(CLASS, REGION);
        DeviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.Join();
        break;
      }
    case DEVICE_STATE_SEND: // a send is scheduled to occur, usu. daily status
      {
        prepareTxFrame( DEVPORT );
        LoRaWAN.Send();
        DeviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.Cycle(TxDutyCycleTime);
        DeviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          if (IsLoRaMacNetworkJoined) {
            if (prepareTxFrame(APPPORT)) {
              LoRaWAN.Send();
            }
          }
          accelWoke = false;
        }
        LoRaWAN.Sleep();
       // Serial.println("After calling LoraWan.sleep()");
        break;
      }
    default:
      {
        DeviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
void DownLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++) {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++) {
    if (mcpsIndication->Buffer[i] == 220) { // DC for APP_TX_DUTYCYCLE; 0D BB A0  for 900000 (15min); 04 93 E0 for 300000 (5min)
      APP_TX_DUTYCYCLE = mcpsIndication->Buffer[i++]<<32|mcpsIndication->Buffer[i++]<<16|mcpsIndication->Buffer[i++]<<8|mcpsIndication->Buffer[i++];
      Serial.print("  new DutyCycle received: ");
      Serial.print(APP_TX_DUTYCYCLE);
      Serial.println("ms");
      SaveDr();
    }
  }
}
