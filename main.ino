#include <FB_Const.h>
#include <FB_Error.h>
#include <FB_Network.h>
#include <FB_Utils.h>
#include <Firebase.h>
#include <FirebaseFS.h>
#include <Firebase_ESP_Client.h>

//#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
// #include <ESPAsyncWebServer.h>
//#include <AsyncTCP.h>

#include <ESPmDNS.h>
#include <WiFi.h>
#include <Preferences.h>

#include <EEPROM.h>

 #include <BluetoothSerial.h>
#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>
#include "esp_bt.h"       // <-- Needed for esp_bt_controller_mem_release
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

BluetoothSerial SerialBT;
Preferences pref;

String btName = "ESP32_Relay";

#define EEPROM_Write_EnabledForButtonPress true
#define EEPROM_Write_EnabledFor_BLE true
#define EEPROM_SIZE 64

//// Firebase credentials
//#define API_KEY "AIzaSyAZ1h5quFXX7z5fQEFUCtGwFpIrFPq_7Ic"
//#define DATABASE_URL "https://esp32-based-coffeemaker-default-rtdb.europe-west1.firebasedatabase.app"
//#define USER_EMAIL "psairaja113@gmail.com"
//#define USER_PASSWORD "Psai*12_3"

// Firebase credentials
#define API_KEY "AIzaSyB2cqjifeLRw8zYLRmBc_tsijvUF7kx2_Q"
#define DATABASE_URL "https://esp32-relay-iot-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "kishore@gmail.com"
#define USER_PASSWORD "kishore@99"

// Wi-Fi credentials
const char* ssid = "Airtel_Thulir_5G";  //WiFi Name
const char* password = "8939004047";  //WiFi Password

// Firebase setup
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

int milk_Count = 0;
int tication_Count = 0;
int dual_Count = 0;

int r1_ONtimeMemoryAddress = 0;
int r2_ONtimeMemoryAddress = 10;
int r3_ONtimeMemoryAddress = 20;
int r4_ONtimeMemoryAddress = 30;
int milkDelayTimeAddress = 40;
int ticationDelayTimeAddress = 50;
unsigned long pressDuration = 0;

volatile bool optedForMilk = false;
volatile bool optedForBlackCoffee = false;
volatile bool optedForMilk_BlackCoffee = false;

      volatile bool milk_IotFlag = false;
      volatile bool tication_IotFlag = false;
      volatile bool Dual_IotFlag = false;


int button1State = LOW;
int button2State = LOW;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long lastBtn2InterruptTime = 0;


const unsigned long deBounceDelay = 500;
unsigned long deBounceDelay_us = 250000; 

volatile unsigned long lastButton1Press = 0;
volatile unsigned long lastButton2Press = 0;
volatile unsigned long lastButton3Press = 0;
volatile unsigned long lastButton4Press = 0;

volatile bool button1PressedOnce = false;
volatile bool button2PressedOnce = false;
volatile bool button3PressedOnce = false;
volatile bool button4PressedOnce = false;
unsigned int btn1Status = 0;
unsigned int btn2Status = 0;

volatile unsigned long Milk_LongPress = 5000;
volatile unsigned long Tication_LongPress = 5000;

//const char *ssid = "Airtel_Thulir_5G";
//const char *password = "8939004047";
bool dualMode = false;
volatile bool MilkCleaningProcess = false;
volatile bool TicationCleaningProcess = false;

const int button1Pin = 15;  // the Button controls the operation of Relay 1&2
const int button2Pin = 2;   // the Button controls the operation of Relay 3&4
const int button3Pin = 4;   // the Button controls the operation of Relay 5&6
const int button4Pin = 22;  // the Button controls the operation of Relay 7&8

unsigned long btn1_RisingEdgeTime = 0;
unsigned long btn1_FallingEdgeTime = 0;

unsigned long btn2_RisingEdgeTime = 0;
unsigned long btn2_FallingEdgeTime = 0;

unsigned long btn3_RisingEdgeTime = 0;
unsigned long btn3_FallingEdgeTime = 0;

unsigned long current_Time = 0;


volatile bool btn1_RisingEdgeDetected = false;
volatile bool btn1_FallingEdgeDetected = false;

volatile bool btn2_RisingEdgeDetected = false;
volatile bool btn2_FallingEdgeDetected = false;

volatile bool btn3_RisingEdgeDetected = false;
volatile bool btn3_FallingEdgeDetected = false;

bool btn3_Interrupt_flag = false;
volatile bool nTimesFlag = false;
float count_val = 0.0;
float prev_count_val = 0.0;



//AsyncWebServer server(80);
//
//AsyncEventSource events("/events");

enum MotorState
{
  IDLE,
  FWD_ON,
  WAIT_DELAY,
  REV_ON,
  DONE
};

typedef struct 
{
  int buttonPin;
  volatile bool valid_interrupt = false;
  volatile bool cleaning_Running = false;
  unsigned long prev_Millis = 0;
  int btnStatus;
  volatile bool btnShortPress = false;
  int RelayPin;
  
}cleaning_Process;


struct ESP32_Relay
{
  const int Relay_id;
  const int GPIO_pin;
//  const int btnPin;
  int prevState;
  int status; //0:off,1:on 
};
struct ESP32_Relay R1 = { 1, 19, 0, 0};
struct ESP32_Relay R2 = { 2, 18, 0, 0};
struct ESP32_Relay R3 = { 3, 5,0, 0};
struct ESP32_Relay R4 = { 4, 17, 0, 0};
struct ESP32_Relay R5 = { 5, 32, 0, 0};
struct ESP32_Relay R6 = { 6, 33, 0, 0};
struct ESP32_Relay R7 = { 7, 25, 0, 0};
struct ESP32_Relay R8 = { 8, 14, 0, 0};

// Milk -> {R1,R2} (ON Time) {R3, R4} (Rev Time)
// Tication -> {R5,R6} (ON Time) {R7, R8} (Rev Time)
ESP32_Relay *relays[] = {&R1, &R2, &R3, &R4, &R5, &R6, &R7, &R8};
cleaning_Process Milk_CleaningProcess = { button1Pin,false,false,0,LOW,button1PressedOnce,relays[0]->GPIO_pin };
cleaning_Process Tication_CleaningProcess = {button2Pin,false,false,0,LOW,button2PressedOnce,relays[2]->GPIO_pin};



struct MotorGroup {
  ESP32_Relay* forwardRelay1;
//  ESP32_Relay* forwardRelay2;
  ESP32_Relay* reverseRelay1;
//  ESP32_Relay* reverseRelay2;

  unsigned long onTime;
  unsigned long Delay;
  unsigned long revTime;
  const char* name;  
};

MotorGroup Milk = {
  &R1, &R2,
//  &R3, &R4,
  2000,
  500,
  1000,
  "Milk"
};

MotorGroup Tication = {
  &R3, &R4,
//  &R7, &R8,
  2000,
  500,
  1000,
  "Tication"
};

typedef struct MotorProcess
{
  MotorGroup *group;
  MotorState state = IDLE;
  unsigned long prevMillis = 0;
  bool running = false;
}Process;

Process milkProcess{ &Milk, IDLE,0,false };
Process ticationProcess{ &Tication,IDLE,0,false };

// Track connection status
bool deviceConnected = false;

void onBTConnect() {
  Serial.println("✅ Bluetooth Device Connected");
//  isAuthenticated = false;   // Reset password requirement only when reconnected
  deviceConnected = true;
}

void onBTDisconnect() {
  Serial.println("❌ Bluetooth Device Disconnected");
  deviceConnected = false;
}


volatile unsigned long btnLastInterrupt = 0;
volatile bool btnEdgeDetected = false;

bool lastStableState = LOW;   // because you are using INPUT_PULLDOWN
unsigned long pressStartTime = 0;
volatile bool _2times_Flag = false;
volatile bool _3times_Flag = false;
volatile bool _4times_Flag = false;
volatile bool _5times_Flag = false;

static unsigned long lastMilkTime = 0;
static unsigned long lastDecoctionTime = 0;


void IRAM_ATTR handleButtonISR()
{
    unsigned long now = micros();

    if (now - btnLastInterrupt > 5000)   // 5 ms debounce inside ISR
    {
        btnEdgeDetected = true;
        btnLastInterrupt = now;
    }
}

//void IRAM_ATTR handleButton1Interrupt()
//{
//    if (milkProcess.running || ticationProcess.running || dualMode)
//        return;
//
//    unsigned long now = millis();
//
//    if (now - lastButton1Press > 50)  // debounce using millis
//    {
//        Milk_CleaningProcess.valid_interrupt = true;
//        Milk_CleaningProcess.prev_Millis = now;
//        lastButton1Press = now;
//    }
//}

//void IRAM_ATTR handleButton2Interrupt() // Opted for Tication Process
//{
//  if(milkProcess.running || ticationProcess.running || dualMode)
//  {
//    return;
//  }
//            unsigned long now = micros();
////      int state = digitalRead(15);
//      if((now - lastBtn2InterruptTime) < 5000)
//      {
//        return;
//      }
//        lastBtn2InterruptTime = now;
//        if(digitalRead(button2Pin) == HIGH)
//        {
//          // Rising Edge Detected
//          btn2_RisingEdgeDetected = true;
//          btn2_RisingEdgeTime = micros();
//        }
//        else if(digitalRead(button2Pin) == LOW)
//        {
//          // Falling edge Detected
//          btn2_FallingEdgeDetected = true;
//          btn2_FallingEdgeTime = micros();
//        }
//}

void IRAM_ATTR handleButton3Interrupt() // Opted for Dual Process
{
    unsigned long now = millis();
      if(milkProcess.running || ticationProcess.running || dualMode)
      {
        return;
      }

  {
      if((now - lastButton3Press) > deBounceDelay)
      {
        button3PressedOnce = true;
        lastButton3Press = now;
      }
  }

}


//void updateBtn()
//{
//    if (!btnEdgeDetected)
//        return;
//
//    btnEdgeDetected = false;
//
//    bool state = digitalRead(button1Pin);
//
//    // ======= PRESS DOWN (rising edge for pulldown) =======
//    if (state == HIGH && lastStableState == LOW)
//    {
//        pressStartTime = millis();
//    }
//
//    // ======= RELEASE (falling edge for pulldown) =======
//    else if (state == LOW && lastStableState == HIGH)
//    {
//        unsigned long pressTime = millis() - pressStartTime;
//
//        if (pressTime >= 50 && pressTime < 4000)
//        {
//            // SHORT PRESS
////            Milk_CleaningProcess.btnShortPress = true;
//              buttonPress
//        }
//        else if (pressTime >= 4000)
//        {
//            // LONG PRESS
//            Milk_CleaningProcess.cleaning_Running = !Milk_CleaningProcess.cleaning_Running;
//
//            if (Milk_CleaningProcess.cleaning_Running)
//                digitalWrite(relays[0]->GPIO_pin, LOW);   // start
//            else
//                digitalWrite(relays[0]->GPIO_pin, HIGH);  // stop
//        }
//    }
//
//    lastStableState = state;
//}

//void updateCleaningProcess(cleaning_Process &p)
//{
//    if (!p.valid_interrupt)
//        return;
//
//    unsigned long elapsed = millis() - p.prev_Millis;
//
//    // Short press = 50 ms to 3000 ms
//    if (elapsed >= 50 && elapsed <= 3000)
//    {
//        p.btnShortPress = true;
//        Serial.println("✅ Short Press Detected!");
//    }
//
//    // Reset
//    p.valid_interrupt = false;
//    p.prev_Millis = 0;
//}
void updateMotorProcess(Process &p)
{
  if(!dualMode)
  {
    if(!p.running) return;
  }

  unsigned long now = millis();

  switch(p.state)
  {
    case IDLE:
      p.running = true;
      digitalWrite(p.group->forwardRelay1->GPIO_pin, LOW);
      p.group->forwardRelay1->status = 1;

      //sendRelayUpdate(p.group->forwardRelay1);
      
//      digitalWrite(p.group->forwardRelay2->GPIO_pin, LOW);
//      p.group->forwardRelay2->status = 1;
      //sendRelayUpdate(p.group->forwardRelay2);
      p.prevMillis = now;
      p.state = FWD_ON;
      break;

    case FWD_ON:
      if(now - p.prevMillis >= p.group->onTime)
      {
        digitalWrite(p.group->forwardRelay1->GPIO_pin,HIGH);
        p.group->forwardRelay1->status = 0;
//        sendRelayUpdate(p.group->forwardRelay1);
//        digitalWrite(p.group->forwardRelay2->GPIO_pin, HIGH);
//        p.group->forwardRelay2->status = 0;
//        sendRelayUpdate(p.group->forwardRelay2);
        p.prevMillis = now;
        p.state = WAIT_DELAY;
      }
      break;
    case WAIT_DELAY:
        if(now - p.prevMillis >= p.group->Delay)
        {
            digitalWrite(p.group->reverseRelay1->GPIO_pin,LOW);
            p.group->reverseRelay1->status = 1;
//            sendRelayUpdate(p.group->reverseRelay1);
//            digitalWrite(p.group->reverseRelay2->GPIO_pin, LOW);
//            p.group->reverseRelay2->status = 1;
//            sendRelayUpdate(p.group->reverseRelay2);
            p.prevMillis = now;
            p.state = REV_ON;
        }
        break;
    case REV_ON:
        if(now - p.prevMillis >= p.group->revTime)
        {
            digitalWrite(p.group->reverseRelay1->GPIO_pin,HIGH);
            p.group->reverseRelay1->status = 0;
//            sendRelayUpdate(p.group->reverseRelay1);
//            digitalWrite(p.group->reverseRelay2->GPIO_pin,HIGH);
//            p.group->reverseRelay2->status = 0;
//            sendRelayUpdate(p.group->reverseRelay2);
            p.state = DONE;
        }
        break;
    case DONE:
        p.running = false;
//        milk_IotFlag = false;
//        tication_IotFlag = false;
//        Dual_IotFlag = false;
        Firebase.RTDB.setBool(&fbdo, "2/Milk/milk_bool", LOW);
        Firebase.RTDB.setBool(&fbdo, "2/Decoction/decoction_bool", LOW);
        Firebase.RTDB.setBool(&fbdo, "2/Dual/dual_bool", LOW);
//        Firebase.RTDB.setBool(&fbdo, "/relay4", LOW);

        break;
  }
}

//String processor(const String &var) {
//  if (var == "btn1txt") {
//    return milkProcess.running == true ? "ON" : "OFF";
//  } else if (var == "btn2txt") {
//    return ticationProcess.running == true ? "ON" : "OFF";
//  } else if (var == "btn3txt") {
////    return R3.status == 0 ? "OFF" : "ON";
//      return dualMode == true ? "ON" : "OFF";
//  } else if (var == "btn4txt") {
////    return R4.status == 0 ? "OFF" : "ON";
//      return (!milkProcess.running && !ticationProcess.running) ? "ON" : "OFF";
//  } else if (var == "btn1class") {
//    return milkProcess.running == true ? "button" : "button2";
//  } else if (var == "btn2class") {
//    return ticationProcess.running == true ? "button" : "button2";
//  } else if (var == "btn3class") {
//
//    
////    return R3.status == 0 ? "button" : "button2";
//      return dualMode == true ? "button" : "button2";
//  } else if (var == "btn4class") {
////    return R4.status == 0 ? "button" : "button2";
//      return (!milkProcess.running && !ticationProcess.running) ? "button" : "button2";
//      
//  }
//
//  return String();
//}


//const char index_html[] PROGMEM = R"rawliteral(
//  <!DOCTYPE HTML><html>
//  <head>
//    <meta name='viewport' content='width=device-width, initial-scale=1'>
//    <title>ESP32 Relay Switch</title>
//    <style>
//    html { font-family: arial; display: inline-block; margin: 0px auto; text-align: center;}
//    .button {
//      background-color: mediumseagreen;
//      border: none;
//      color: white;
//      padding: 10px 15px;
//      text-decoration: none;
//      font-size: 24px;
//      cursor: pointer;
//      margin: 3px;
//    }
//    .button2 {
//      background-color: gray;
//      border: none;
//      color: white;
//      padding: 10px 15px;
//      text-decoration: none;
//      font-size: 24px;
//      cursor: pointer;
//      margin: 3px;
//    }
//    .button3 {
//      background-color: crimson;
//      border: none;
//      color: white;
//      padding: 5px 10px;
//      text-decoration: none;
//      font-size: 22px;
//      cursor: pointer;
//      margin: 2px;
//    }
//    </style>
//  </head>
//  <body>
//      <h1>CSK Master!</h1>
//      <h3 style='color: red;'\>Milk</h3>
//      <p><a href='/set?button_id=1'><button id='btn1' class='%btn1class%'>%btn1txt%</button></a></p>
//      <h3 style='color: green;'>Tication</h3>
//      <p><a href='/set?button_id=2'><button id='btn2' class='%btn2class%'>%btn2txt%</button></a></p>
//      <h3 style='color: blue;'>Dual</h3>
//      <p><a href='/set?button_id=3'><button id='btn3' class='%btn3class%'>%btn3txt%</button></a></p>
//      <h3 style='color: orange;'>Reset</h3>
//      <p><a href='/set?button_id=4'><button id='btn4' class='%btn4class%'>%btn4txt%</button></a></p>
//      <p><a href='/reset'><button class='button3'>Reset ALL</button></a></p>
//      <script>
//      if (!!window.EventSource) {
//        var source = new EventSource('/events');
//        source.addEventListener('toggleState', function(e) {
//          console.log(e.data);
//          let jsonData = JSON.parse(e.data);
//          const element = document.getElementById(jsonData.id);
//          if(jsonData.status == 1){
//            element.innerHTML = 'OFF';
//            element.className = "button2";    
//          }else{
//            element.innerHTML = 'ON';
//            element.className = "button";
//          }
//        }, false);
//      }
//</script>
//</body>
//</html>)rawliteral";

int n = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  Serial.println("Scanning...");
delay(750);
n = WiFi.scanNetworks();
for(int u = 0;u<10;u++)
{
  pinMode(button1Pin, INPUT_PULLDOWN);
  pinMode(button2Pin, INPUT_PULLDOWN);
  pinMode(button3Pin, INPUT_PULLDOWN);
  delay(200);
}
//  pinMode(button4Pin, INPUT_PULLDOWN);

  //Button1 Interrupt for Relay 1
//  attachInterrupt(button1Pin, handleButtonISR, CHANGE);
 
//  attachInterrupt(digitalPinToInterrupt(button1Pin),handleButton1Interrupt,RISING);
  
//Button2 Interrupt for Relay 2 
//  attachInterrupt(digitalPinToInterrupt(button2Pin),handleButton2Interrupt,RISING);

//Button3 Interrupt for Relay 3
  attachInterrupt(digitalPinToInterrupt(button3Pin),handleButton3Interrupt,RISING);


//delay(550);

      pref.begin("btConfig", false);
      btName = pref.getString("name",btName);
  if (!SerialBT.begin(btName, false)) 
  {
    Serial.println("Bluetooth failed to start!");
    while (true);
  }

  SerialBT.register_callback([](esp_spp_cb_event_t event, esp_spp_cb_param_t *param) 
  {
    if (event == ESP_SPP_SRV_OPEN_EVT) 
    {
      onBTConnect();
    } 
    else if (event == ESP_SPP_CLOSE_EVT) 
    {
      onBTDisconnect();
    }
  });

    // initialize the pushbutton pin as input:
    if(!EEPROM.begin(EEPROM_SIZE))
    {
      Serial.println("Failed to initialize EEPROM");
      while(1);
    }
    else
    {
      Serial.println("EEPROM Initialized Successfully"); 
    }

//  if(n>0)
//  {
//      for (int i = 0; i < n; i++) 
//      {
//        Serial.println(WiFi.SSID(i));
//      }
//        Serial.println("-------------------");
//  }
//  else
//  {
//    Serial.print("No WiFi Detected");
//    Serial.println("-------------------");
//  }

for(int j = 0;j<4;j++)
{
  for(int i = 0; i<8;i++)
  {
    pinMode(relays[i]->GPIO_pin, OUTPUT);
    digitalWrite(relays[i]->GPIO_pin, HIGH); // Makes All Relay->OFF
    delay(100);
  }
}
//delay(600);


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
//  Serial.println("");
//
//  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(700);
    Serial.print(".");
  }

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);


  // Wait for Connection

//  Serial.println("");

      EEPROM.get(r1_ONtimeMemoryAddress,Milk.onTime);
      if(Milk.onTime <= 250)
      {

        Milk.onTime = 2000;
        EEPROM.put(r1_ONtimeMemoryAddress,Milk.onTime);
        EEPROM.commit();

      }
      EEPROM.get(r2_ONtimeMemoryAddress,Milk.revTime);
      if(Milk.revTime<=250)
      {
        Milk.revTime = 2000;
        EEPROM.put(r2_ONtimeMemoryAddress,Milk.revTime);
        EEPROM.commit();


      }
      
      EEPROM.get(r3_ONtimeMemoryAddress,Tication.onTime);
      if(Tication.onTime <= 250)
      {
        Tication.onTime = 2000;
        EEPROM.put(r3_ONtimeMemoryAddress,Tication.onTime);
        EEPROM.commit();

      }
      
      EEPROM.get(r4_ONtimeMemoryAddress,Tication.revTime);
      if(Tication.revTime <= 250)
      {
        Tication.revTime = 2000;
        EEPROM.put(r4_ONtimeMemoryAddress,Tication.revTime);
        EEPROM.commit();

      }

      EEPROM.get(milkDelayTimeAddress,Milk.Delay);
      if(Milk.Delay <= 250)
      {
        Milk.Delay = 1250;
        EEPROM.put(milkDelayTimeAddress,Milk.Delay);
        EEPROM.commit();

      }
      EEPROM.get(ticationDelayTimeAddress,Tication.Delay);
      if(Tication.Delay <= 250)
      {
        Tication.Delay = 1250;
        EEPROM.put(ticationDelayTimeAddress,Tication.Delay);
        EEPROM.commit();
      }


  Serial.print("Connected to");
  Serial.println(ssid);
//  Serial.print("IP Address :");
//  Serial.println(WiFi.localIP());

  // Set up mDNS responder: "esp32.local"  to access webpage
//  if(!MDNS.begin("esp32"))
//  {
//    Serial.println("Error setting up MDNS Responder!");
//    while(1)
//    {
//      delay(1000);
//    }
//  }
//  Serial.println("mDNS responder started");
//  // Add service to MDNS-SD
//  MDNS.addService("http","tcp",80);
//
//  // Handle Web Server Root
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//    request->send_P(200, "text/html", index_html, processor);
//  });
//
//  // Send a GET request to <IP>/set?button_id=<id>
//  // GET Request Created by html page based on user Input, 
//  // so this function will react to anything like /set?button_id=1
//  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
//    if (request->hasParam("button_id")) {
//      int idValue = request->getParam("button_id")->value().toInt();
//      if (idValue == 1) {
////        togglePinState(&d1);
//          if(!milkProcess.running && !ticationProcess.running)
//          {
//            milkProcess.running = true;
//            milkProcess.state = IDLE;
//            milkProcess.prevMillis = millis();
//            sendButtonUpdate(1,1);
//            sendButtonUpdate(2,0);
//            sendButtonUpdate(3,0);
//            sendButtonUpdate(4,0);
//
////          updateMotorProcess(milkProcess);
//          }
//      } else if (idValue == 2) {
////        togglePinState(&d2);
//          if(!ticationProcess.running && !milkProcess.running)
//          {
//            ticationProcess.running = true;
//            ticationProcess.state = IDLE;
//            ticationProcess.prevMillis = millis();
//            sendButtonUpdate(1,0);
//            sendButtonUpdate(2,1);
//            sendButtonUpdate(3,0);
//            sendButtonUpdate(4,0);
//
////            updateMotorProcess(ticationProcess);
//          }
//      } else if (idValue == 3) {
//        if(!milkProcess.running && !ticationProcess.running)
//        {
//          dualMode = true;
//          milkProcess.running = true;
//          milkProcess.state = IDLE;
//          milkProcess.prevMillis = millis();
//
//          ticationProcess.running = true;
//          ticationProcess.state = IDLE;
//          ticationProcess.prevMillis = millis();
//          sendButtonUpdate(3,1);
//        }
//          
////        togglePinState(&d3);
//
//      } else if (idValue == 4) {
////        togglePinState(&d4);
//        milkProcess.running = false;
//        ticationProcess.running = false;
//        milkProcess.state = IDLE;
//        ticationProcess.state = IDLE;
//        sendButtonUpdate(1,0);
//        sendButtonUpdate(2,0);
//        sendButtonUpdate(3,0);
//        sendButtonUpdate(4,1);
//      }
//    }
//    request->send_P(200, "text/html", index_html, processor);
//  });

// So Whenever User Presses the Reset Button, GET Request Generated
// The ESP32 Responds with this function to any GET Request received such as http://192.168.1.50/reset

//server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
//  resetAll();
//  request->send_P(200, "text/html", index_html, processor);
//});

  //Handle Web Server Events
  //This function happens when the Source Client Disconnect and reconnects
//  events.onConnect([](AsyncEventSourceClient *client) 
//  {
//    if(client->lastId())  
//    {
//      Serial.printf("Client recconnected! Last message ID that it got is: %u\n", client->lastId());
//      
//    }
//    client->send("hello!", NULL, millis(), 10000);
//    
//  });
//  server.addHandler(&events);
//  server.begin();
//  Serial.println("HTTP server started"); 
  
}

boolean debounceButton(boolean state, int btnPin)
{
  boolean stateNow = digitalRead(btnPin);
  if(state != stateNow)
  {
    delay(10);
    stateNow = digitalRead(btnPin);
    
  }
  return stateNow;
}

void updateButton(cleaning_Process *cp, int *prevState, int pin, volatile bool *buttonPressedOnce)
{
      if (milkProcess.running || ticationProcess.running || dualMode)
        return;

    int currentState = debounceButton(*prevState, pin);

    // No change → no edge → exit
    if (currentState == *prevState) return;

    unsigned long now = millis();

    // ========== RISING EDGE (LOW → HIGH) ==========
    if (currentState == HIGH && *prevState == LOW)
    {
        cp->prev_Millis = now;     // Start timing
    }

    // ========== FALLING EDGE (HIGH → LOW) ==========
    else if (currentState == LOW && *prevState == HIGH)
    {
        unsigned long elapsed = now - cp->prev_Millis;

        // ---- Short Press (10ms – 3s) ----
        if (elapsed >= 10 && elapsed < 3000)
        {
            if (!cp->cleaning_Running)
            {
              
//                cp->btnShortPress = true;          // short-press action
                  *buttonPressedOnce = true;
            }
            else
            {
                digitalWrite(cp->RelayPin, HIGH); // Turn OFF
                cp->cleaning_Running = false;
            }
        }

        // ---- Long Press (>= 3s) ----
        else if (elapsed >= 3000)
        {
            digitalWrite(cp->RelayPin, LOW);  // Turn ON
            cp->cleaning_Running = true;
        }

        cp->prev_Millis = 0;
    }

    // Update previous state
    *prevState = currentState;
}

void loop() 
{
  
  // put your main code here, to run repeatedly:

if(SerialBT.available()) 
  {
    String received = SerialBT.readStringUntil('\n');  // Read till newline
    received.trim();  // Remove extra spaces or \r

    Serial.print("Received: ");
    Serial.println(received);

    // --- Respond based on message ---
    if (received.equalsIgnoreCase("HELLO")) 
    {
      SerialBT.println("Hi from ESP32!");
    }
    else if (received.equalsIgnoreCase("STATUS")) 
    {
      SerialBT.print("Current Relay1 ON Time: ");
      SerialBT.print(((float)Milk.onTime)/1000,2);
      SerialBT.println(" Sec");

      SerialBT.print("Current Relay2 ON Time: ");
      SerialBT.print(((float)Milk.revTime/1000),2);
      SerialBT.println(" Sec");
      
      SerialBT.print("Current Relay3 ON Time: ");
      SerialBT.print(((float)Tication.onTime/1000),2);
      SerialBT.println(" Sec");

      SerialBT.print("Current Relay4 ON Time: ");
      SerialBT.print(((float)Tication.revTime/1000),2);
      SerialBT.println(" Sec");
    
      SerialBT.println("All systems OK ✅");
    }
        if (received.startsWith("NAME="))
        {
          String newName;
          newName = received.substring(5);
          newName.trim();
          pref.putString("name", newName);
          SerialBT.end();
          delay(500);
          SerialBT.begin(newName,false);
        } 
    if(received.length() > 0 && isDigit(received.charAt(0)))
    {
      int i = 0;
      while(i<received.length() && isDigit(received.charAt(i)))
      {
        i++;
      }
      count_val = received.substring(0,i).toFloat();
      String rx_str = received.substring(i);
      if(rx_str.startsWith("R"))
      {
        if(!nTimesFlag)
        {
          nTimesFlag = true;
          EEPROM.get(r1_ONtimeMemoryAddress,Milk.onTime);
          EEPROM.get(r3_ONtimeMemoryAddress,Tication.onTime);
          //Increase Milk, Tication On Time n * OnTime
          Milk.onTime = count_val*Milk.onTime;
          Tication.onTime = count_val*Tication.onTime;
           prev_count_val = count_val;        
        }
        else
        {
          if(prev_count_val == count_val)
          {
              count_val = 0.0;
              nTimesFlag = false;
              EEPROM.get(r1_ONtimeMemoryAddress,Milk.onTime);
              EEPROM.get(r2_ONtimeMemoryAddress,Milk.revTime);
              EEPROM.get(r3_ONtimeMemoryAddress,Tication.onTime);
              EEPROM.get(r4_ONtimeMemoryAddress,Tication.revTime);
          }
        }
      }
    }
    received.toUpperCase();
    
    if(received.startsWith("R1") || received.startsWith("R2") 
        || received.startsWith("R3") || received.startsWith("R4") 
        || received.startsWith("D1") || received.startsWith("D2"))
    {
      
//            if(!isAuthenticated)
//            {
//              SerialBT.println("\nPlease Enter Password:");     
//              // Wait until user sends something
//              while (!SerialBT.available()) 
//              {
//                  delay(10); // small wait to reduce CPU usage
//              }
//              String pwd = SerialBT.readStringUntil('\n');  // Read till newline
//              pwd.trim();  // Remove extra spaces or \r
//              if(pwd == "1234")
//              {
//                
//              isAuthenticated = true;
//                SerialBT.println("Password Verified Successfully");
//              }
//            }
                  if(received.equalsIgnoreCase("R1 ON"))
                  {
                                      optedForMilk = true;
                  //volatile bool optedForBlackCoffee = false;
                  //volatile bool optedForMilk_BlackCoffee = false;
                            if(!milkProcess.running && !ticationProcess.running)
                            {
//                              milkProcess.running = true;
//                              milkProcess.state = IDLE;
//                              milkProcess.prevMillis = millis();
                              button1PressedOnce = true;

//                              sendButtonUpdate(1,1);
//                              sendButtonUpdate(2,0);
//                              sendButtonUpdate(3,0);
//                              sendButtonUpdate(4,0);
                  
                  //          updateMotorProcess(milkProcess);
                            }

                  }
                  else if(received.equalsIgnoreCase("R2 ON"))
                  {
                    if(!milkProcess.running && !ticationProcess.running)
                    {
                      button2PressedOnce = true;
                    }
                  }
                  else if(received.equalsIgnoreCase("R3 ON"))
                  {
                    if(!milkProcess.running && !ticationProcess.running)
                    {
                      button3PressedOnce = true;
                    }
                  }
//                  else if(received.equalsIgnoreCase("R4 ON"))
//                  {
//                    button4PressedOnce = true;
//                  }
                  else
                  {
//                        char sign = received.charAt(2);
                          char sign = '\0';
                          float value = 0.0;
                          for(int i = 0;i<received.length();i++)
                          {
                            char c = received.charAt(i);
                            if(c == '+' || c == '-' || c == '=')
                            {
                              sign = c; // stop at first catch
                              value = received.substring(i+1).toFloat();
                              break;
                            }
                          }
                        float changeMS = value * 1000.0;
                        if(received.startsWith("R1"))
                        {
                            if(sign == '+') // If condition true then User opted for Milk Time Increment
                            {
                              Milk.onTime += (long int)changeMS;
                              if(EEPROM_Write_EnabledFor_BLE)
                              {
                                EEPROM.put(r1_ONtimeMemoryAddress,Milk.onTime);
                                EEPROM.commit();
                              }
                              SerialBT.print("Relay 1 ON Time is increased by");
                              SerialBT.print((float)changeMS/1000.0,2);
                              SerialBT.println(" Sec");
                            }
                            if(sign == '-')
                            {
                                  if(Milk.onTime >500)
                                  {
                                  Milk.onTime -= (int)changeMS;
                                  SerialBT.print("Relay 1 ON Time is decreased by");
                                  SerialBT.print((float)changeMS/1000.0,2);
                                  SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r1_ONtimeMemoryAddress,Milk.onTime);
                                      EEPROM.commit();
                                    }
                                  }
                                  else
                                  {
                                    SerialBT.println("Minimum Limit Reached");
                                  }        
                            }
                            if(sign == '=')
                            {
                              Milk.onTime = (int)changeMS;
                              if(EEPROM_Write_EnabledFor_BLE)
                              {
                                EEPROM.put(r1_ONtimeMemoryAddress,Milk.onTime);
                                EEPROM.commit();
                              }
                              
                            }
                        }
                        else if(received.startsWith("R2")) // User Option for Milk Reverse time
                        {
                            if(sign == '+')
                            {
                              Milk.revTime += (int)changeMS;
                              SerialBT.print("Relay 2 ON Time is increased by");
                              SerialBT.print((float)changeMS/1000.0,2);
                              SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r2_ONtimeMemoryAddress,Milk.revTime);
                                      EEPROM.commit();
                                    }

                            }
                            if(sign == '-')
                            {
                                  if(Milk.revTime >500)
                                  {
                                  Milk.revTime -= (long int)changeMS;
                                  SerialBT.print("Relay 2 ON Time is decreased by");
                                  SerialBT.print((float)changeMS/1000.0,2);
                                  SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r2_ONtimeMemoryAddress,Milk.revTime);
                                      EEPROM.commit();
                                    }
                                  }
                                  else
                                  {
                                    SerialBT.println("Minimum Limit Reached");
                                  }        
                            }
                            if(sign == '=')
                            {
                              Milk.revTime = (int)changeMS;
                              if(EEPROM_Write_EnabledFor_BLE)
                              {
                                EEPROM.put(r2_ONtimeMemoryAddress,Milk.revTime);
                                EEPROM.commit();
                              }
                              
                            }

                        }
                        else if(received.startsWith("R3")) // User Option for Tication On time
                        {
                            if(sign == '+')
                            {
                              Tication.onTime += (int)changeMS;
                              SerialBT.print("Relay 3 ON Time is increased by");
                              SerialBT.print((float)changeMS/1000.0,2);
                              SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r3_ONtimeMemoryAddress,Tication.onTime);
                                      EEPROM.commit();
                                    }

                            }
                            if(sign == '-')
                            {
                                  if(Tication.onTime >500)
                                  {
                                  Tication.onTime -= (long int)changeMS;
                                  SerialBT.print("Relay 3 ON Time is decreased by");
                                  SerialBT.print((float)changeMS/1000.0,2);
                                  SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r3_ONtimeMemoryAddress,Tication.onTime);
                                      EEPROM.commit();
                                    }
                                  }
                                  else
                                  {
                                    SerialBT.println("Minimum Limit Reached");
                                  }        
                            }
                                  if(sign == '=')
                                  {
                                    Tication.onTime = (int)changeMS;
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r3_ONtimeMemoryAddress,Tication.onTime);
                                      EEPROM.commit();
                                    }
                                    
                                  }

                        }
                        else if(received.startsWith("R4")) // User Option for Tication Reverse Time
                        {
                            if(sign == '+')
                            {
                              Tication.revTime += (int)changeMS;
                              SerialBT.print("Relay 7 & 8 ON Time is increased by");
                              SerialBT.print((float)changeMS/1000.0,2);
                              SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r4_ONtimeMemoryAddress,Tication.revTime);
                                      EEPROM.commit();
                                    }

                            }
                            if(sign == '-')
                            {
                                  if(Tication.revTime >500)
                                  {
                                  Tication.revTime -= (long int)changeMS;
                                  SerialBT.print("Relay 4 ON Time is decreased by");
                                  SerialBT.print((float)changeMS/1000.0,2);
                                  SerialBT.println(" Sec");
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r4_ONtimeMemoryAddress,Tication.revTime);
                                      EEPROM.commit();
                                    }
                                  }
                                  else
                                  {
                                    SerialBT.println("Minimum Limit Reached");
                                  }        
                            }
                            if(sign == '=')
                                  {
                                    Tication.revTime = (int)changeMS;
                                    if(EEPROM_Write_EnabledFor_BLE)
                                    {
                                      EEPROM.put(r4_ONtimeMemoryAddress,Tication.revTime);
                                      EEPROM.commit();
                                    }
                                    
                                  }

                        }
                        else if(received.startsWith("D1"))
                        {
//                              EEPROM.get(milkDelayTimeAddress,Milk.Delay);
//                              EEPROM.get(ticationDelayTimeAddress,TicationDelayTime);

                            if(sign == '=')
                            {
                              Milk.Delay = (int)changeMS;
                              if(EEPROM_Write_EnabledFor_BLE)
                              {
                                EEPROM.put(milkDelayTimeAddress,Milk.Delay);
                                EEPROM.commit();
                              }
                            }
                        }
                        else if(received.startsWith("D2"))
                        {
                          if(sign == '=')
                          {
                            Tication.Delay = (int)changeMS;
                              if(EEPROM_Write_EnabledFor_BLE)
                              {
                                EEPROM.put(ticationDelayTimeAddress,Tication.Delay);
                                EEPROM.commit();
                              }
                          } 
                        }
                }
              }
              else
              {
//                SerialBT.println("Password is incorrect! Try Again");
                received.clear(); 
              }
    }

//      if(btn1_RisingEdgeDetected)
//      {
//        // Rising Edge Detected
//
//          btn1_RisingEdgeDetected = false;
//      }
//      if(btn1_FallingEdgeDetected)
//      {
//        btn1_FallingEdgeDetected = false;
//        unsigned long pressDuration = (btn1_FallingEdgeTime - btn1_RisingEdgeTime);
//        if(pressDuration >= 3000000)
//        {
//                      digitalWrite(Milk.forwardRelay1->GPIO_pin,LOW);
//                      MilkCleaningProcess = true;
//                      pressDuration = 0;
//        }
//        else if(pressDuration >= 100000 && pressDuration < 3000000)
//        {
//          if(!MilkCleaningProcess)
//          {
//                      button1PressedOnce = true;
//          }
//          else
//          {
//            digitalWrite(Milk.forwardRelay1->GPIO_pin,HIGH);
//            MilkCleaningProcess = false;
//          }
//          pressDuration = 0;
//
//        }
//        btn1_RisingEdgeTime = 0;
//        btn1_FallingEdgeTime = 0;
//      }
      if(btn2_RisingEdgeDetected)
      {
        btn2_RisingEdgeDetected = false;
      }
      if(btn2_FallingEdgeDetected)
      {
        btn2_FallingEdgeDetected = false;

        unsigned long pressDuration = (btn2_FallingEdgeTime - btn2_RisingEdgeTime);
        if(pressDuration >= 3000000)
        {
                      digitalWrite(Tication.forwardRelay1->GPIO_pin,LOW);
                      TicationCleaningProcess = true;
//                      pressDuration = 0;
        }
        else if(pressDuration >= 100000 && pressDuration < 3000000)
        {
          if(!TicationCleaningProcess)
          {
                      button2PressedOnce = true;
          }
          else
          {
            digitalWrite(Tication.forwardRelay1->GPIO_pin,HIGH);
            TicationCleaningProcess = false;
          }
          pressDuration = 0;

        }
        btn2_RisingEdgeTime = 0;
        btn2_FallingEdgeTime = 0;
      }

      
// 
    if (Firebase.ready())
    {
          if(Firebase.RTDB.getFloat(&fbdo, "2/Milk/milk_time"))
          {
              float newTimeSec = fbdo.floatData();
              unsigned long newTimeMs = (unsigned long)(newTimeSec * 1000.0);
            
            if(newTimeMs != lastMilkTime)
            {
//              newTime = fbdo.floatData();
              Milk.onTime = newTimeMs;
              Serial.println("Milk Time :");
              Serial.println(Milk.onTime);

              EEPROM.put(r1_ONtimeMemoryAddress,Milk.onTime);
              
              EEPROM.commit();
              lastMilkTime = newTimeMs;
            }
          }
          if(Firebase.RTDB.getFloat(&fbdo, "2/Decoction/decoction_time"))
          {
              float newTimeSec = fbdo.floatData();
              unsigned long newTimeMs = (unsigned long)(newTimeSec * 1000.0);
            if(newTimeMs != lastDecoctionTime)
            {
              Tication.onTime = newTimeMs;
//              newTime = fbdo.floatData();
              Serial.println("Tication Time :");
              Serial.println(Tication.onTime);

              EEPROM.put(r3_ONtimeMemoryAddress,Tication.onTime);
              EEPROM.commit();
              lastDecoctionTime = newTimeMs;
            }

          }
          if (!milkProcess.running && !ticationProcess.running && Firebase.RTDB.getBool(&fbdo, "2/Milk/milk_bool")) 
          {
            milk_IotFlag = fbdo.boolData();
//            digitalWrite(RELAY1, r1 ? LOW : HIGH);
            if(milk_IotFlag == true)
            {
                milk_IotFlag = false;
                milkProcess.running = true;
                milkProcess.state = IDLE;
                milkProcess.prevMillis = millis();
//            Firebase.RTDB.setBool(&fbdo, "/relay1", HIGH);
            }
          }
          if (!milkProcess.running && !ticationProcess.running && Firebase.RTDB.getBool(&fbdo, "2/Decoction/decoction_bool"))
          {
            tication_IotFlag = fbdo.boolData();
            if(tication_IotFlag)
            {
              tication_IotFlag = false;
              ticationProcess.running = true;
              ticationProcess.state = IDLE;
              ticationProcess.prevMillis = millis();
//            Firebase.RTDB.setBool(&fbdo, "/relay2", HIGH);
            }

          }
          if (!milkProcess.running && !ticationProcess.running && Firebase.RTDB.getBool(&fbdo, "2/Dual/dual_bool"))
          {
            Dual_IotFlag = fbdo.boolData();
            if(Dual_IotFlag)
            {
              Dual_IotFlag = false;
              dualMode = true;
              milkProcess.running = true;
              milkProcess.state = IDLE;
              milkProcess.prevMillis = millis();
    
              ticationProcess.running = true;
              ticationProcess.state = IDLE;
              ticationProcess.prevMillis = millis();
//              Firebase.RTDB.setBool(&fbdo, "/relay3", HIGH);

//              sendButtonUpdate(3,1);
            }
          }      
    }
    
      if(button1PressedOnce) // Opted For Milk
      {
          //if(!milkProcess.running && !ticationProcess.running)
          {
            milkProcess.running = true;
            milkProcess.state = IDLE;
            milkProcess.prevMillis = millis();
//            Firebase.RTDB.setBool(&fbdo, "/relay1", HIGH);

          }
          button1PressedOnce = false;


//          updateMotorProcess(milkProcess);
      }
      if(button2PressedOnce)
      {
          //if(!milkProcess.running && !ticationProcess.running)
          {
            ticationProcess.running = true;
            ticationProcess.state = IDLE;
            ticationProcess.prevMillis = millis();
//            Firebase.RTDB.setBool(&fbdo, "/relay2", HIGH);

          }
          button2PressedOnce = false;

//          updateMotorProcess(ticationProcess);
      }
      if(button3PressedOnce)
      {
            if(!milkProcess.running && !ticationProcess.running)
            {
              dualMode = true;
              milkProcess.running = true;
              milkProcess.state = IDLE;
              milkProcess.prevMillis = millis();
    
              ticationProcess.running = true;
              ticationProcess.state = IDLE;
              ticationProcess.prevMillis = millis();
//              Firebase.RTDB.setBool(&fbdo, "/relay3", HIGH);

//              sendButtonUpdate(3,1);
            }
            button3PressedOnce = false;

      }
      updateMotorProcess(milkProcess);
      updateMotorProcess(ticationProcess);
//      updateCleaningProcess(Milk_CleaningProcess);
updateButton(&Milk_CleaningProcess, &button1State, button1Pin,&button1PressedOnce);
updateButton(&Tication_CleaningProcess, &button2State,button2Pin,&button2PressedOnce); 
//int currentState = debounceButton(buttonState, button1Pin);
//
//if (currentState != buttonState)     // state changed → edge detected
//{
//    unsigned long now = millis();
//
//    // ========== RISING EDGE (LOW → HIGH) ==========
//    if (currentState == HIGH && buttonState == LOW)
//    {
//        Milk_CleaningProcess.prev_Millis = now;   // start timing
//    }
//
//    // ========== FALLING EDGE (HIGH → LOW) ==========
//    else if (currentState == LOW && buttonState == HIGH)
//    {
//        unsigned long Time_Elapsed = now - Milk_CleaningProcess.prev_Millis;
//
//        // ---- Short Press (50ms - 4s) ----
//        if (Time_Elapsed >= 10 && Time_Elapsed < 3000)
//        {
//            if (!Milk_CleaningProcess.cleaning_Running)
//            {
//                button1PressedOnce = true;  // trigger short action
//            }
//            else
//            {
//                digitalWrite(relays[0]->GPIO_pin, HIGH); // relay OFF
//                Milk_CleaningProcess.cleaning_Running = false;
//            }
//        }
//
//        // ---- Long Press (>= 4s) ----
//        else if (Time_Elapsed >= 3000)
//        {
//            digitalWrite(relays[0]->GPIO_pin, LOW); // relay ON
//            Milk_CleaningProcess.cleaning_Running = true;
//        }
//
//        Milk_CleaningProcess.prev_Millis = 0;
//    }
//
//    // finally update previous state
//    buttonState = currentState;
//}



  if(dualMode)
  {
    if(!milkProcess.running && !ticationProcess.running)
    {
      dualMode = false;
    }
  }

}

//void sendRelayUpdate(ESP32_Relay *r)
//{
//  char data[100];
//  snprintf(data,100, "{\"id\":\"btn%d\",\"status\":%d}", r->Relay_id, r->status);
//  events.send(data, "toggleState", millis());
//}
//
//void sendButtonUpdate(int btn, int status)
//{
//  char data[50];
//  snprintf(data, 50, "{\"id\":\"btn%d\",\"status\":%d}",btn,status);
//  events.send(data,"toggleState",millis());
//}
