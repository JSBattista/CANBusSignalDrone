/*
      MM_SF2-SIGDrone.
      Console Signal Response Generator for testing during assembly.   
 
 * 
 * https://forum.arduino.cc/t/measuring-time/96602/4
 * https://stackoverflow.com/questions/1163624/memcpy-with-startindex
 * https://stackoverflow.com/questions/19867227/convert-double-type-into-string-type-in-arduino-sketch
 * https://lastminuteengineers.com/oled-display-arduino-tutorial/


  Notes: Signal Drones should be powered by 12VDC supplies capable of delivering enough amperage so as not to "fall flat" during the HORN test. 
         Weaker units can drop to less than 12VDC during this test and give a false failure during the ON state test. 2A should be the minimum
         but even with that capability it might still fall flat. Subsystem TVS components will compensate by trying to make more current flow.
         Power supplies should be tested individually.
         A power supply can droop to around 11.8 volts and the drone and console - even a premier - may still function. Drones might vary.

        A premier model during horn test can draw up to 1A load. 

        In cases where the crystal for the CAN transceiver is only 8MZz, specifying 500kbps gives 250. In the use of the 2 channel system, 
        an error will result from that when using a 250kpbs system. So for a 250 kbps system, specify 250 kbps.
  
        All button and light entities have the same structure for ID and command - not entirely necessary to store entire arrays. 
         
        Only trims differ and have DLC of 4. 
         
        Individual SS buttons have only light entites, output is analog tracked : NOTE use 5V protective Zener diodes in the voltage divider 
        in the event a greater than 12VDC power supply is used.

        Some sample code to put readings to the small screen
            char SSreadbuff[7];
            sprintf(SSreadbuff, "%i,%i", sensorValueHORN_OFF, sensorValueHORN_ON);
            showTestStatus("!DEB100!", String(SSreadbuff), 1);
            delay(20000);


  
 */
#include <mcp_can.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels.0
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MCP_CAN CAN0(9);           
MCP_CAN CAN1(10);
unsigned long marktime = 0;
// All Light entities have the same address. 
// The data sent to that address differs in ID and Command, all are DLC 7
// Individual SS buttons differ in that they all have the same ID in the fourth element , but the fifth element determines which is lit.
long unsigned int lightIO_ADDR = 0x1410D190;
unsigned char lightIO_templ[8] {0x0, 0x1, 0x0, 0x32, 0x00, 0x00, 0x1, 0x00 };  // Elements 5 and 6 of this array varies by ID and Command (0xF on/ 0x0 off)
unsigned int lightIO_dlc = 7;
unsigned char at1              =  0x3A;        
unsigned char at2              =  0x37;
unsigned char at3              =  0x38;
unsigned char at4              =  0x39;
unsigned char at5              =  0x35;
unsigned char ATredLED         =  0x33;
unsigned char ATAmberLED       =  0x34;
unsigned char TOAmberLED       =  0x9;          
unsigned char TXFRAmberLED     =  0x6;
unsigned char OneLEverAmberLED =  0x8;
unsigned char QSAmberLED       =  0xA;
unsigned char ssALL            =  0xE;
unsigned char ssALL_PNL        =  0xF;    // Note that BSS all for Panel is the same value
unsigned char neutPORT         =  0x2;
unsigned char neutSTBD         =  0x1;
unsigned char SS_STBD          =  0xF;
unsigned char SS_STBD_CENTER   =  0x11;
unsigned char SS_PORT_CENTER   =  0x12;
unsigned char SS_PORT          =  0x10;
unsigned char SS_ALL_SINGLE    =  0xF;
// Note an ID of 0x00 makes everything flash
int sensorPinPort             = A0;    
int sensorPinPortCenter       = A1;    
int sensorPinStarboardCenter  = A2;    
int sensorPinStarboard        = A3;    
int sensorPinHORN             = A6;        // Sensor Pin for HORN circuit   

int sensorValuePORT           = 0;            // These will store the ADC reads
int sensorValuePORTCENTER     = 0;
int sensorValueSTBDCENTER     = 0;
int sensorValueSTBD           = 0;

const int starboardLEDpin       = 4;
const int portLEDpin            = 5;
const int starboardCenterLEDpin = 6;
const int portCenterLEDpin      = 7;
const int wakeRELAYpin          = 8;
const int ThreshHORN_OFF        = 700;  // 757 on a 12VDC system
const int ThreshHORN_ON         = 100;  // roughly 3mV on a 12VDC system comes out to around 60. Allow for up to half a volt for wiggle room.
long unsigned int rxId;
unsigned char len = 0;            // Best to create these here to isolate and so as not to "carry over" from one test to the next.
unsigned char rxBuf[8];
// With buttons, a general query is sent that can also get backlight status too, but buttons responses are picked up.      
unsigned char blQuery[8] = {0x0, 0x1, 0x0, 0x32, 0x0, 0x0, 0x1, 0x0};  
// Button entities have different addresses: the address source of a button press is unique to each button
// Trim buttons have 2 addresses per: the same address for up and down, but data differs and is DLC 4
// Because these differences are multi-field, a struct is used.
// Buttons have "sections" where the address will match, but then also an ID in the array of bytes that comes in.
struct IO_ButtonEntity {
          long unsigned int button_ADDR;
          unsigned char bdat[8];    // Buttons see not only different addresses and dlc but data can differ too.
          unsigned int dlc;
          unsigned short lights;    // Some buttons have more than one light, or some sequence of lights should flash, or NO lights that need to be used to indicate press.
          unsigned char corr_LIGHT[5]; // Most buttons have a corresponding light, but there is room here to use some buttons to test unrelated lights.
};                
unsigned short digitalButtonCOUNT = 9;   // total count of buttons that put signals into the CAN bus
IO_ButtonEntity buttons[9] = {
                                    {0x141090D1, { 0x0, 0x1, 0x0, 0x35,0x2, 0x1, 0x1, 0x0 }, 7, 1, {TXFRAmberLED}},              // Transfer button
                                    {0x141090D1, { 0x0, 0x1, 0x0, 0x35,0x7, 0x1, 0x1, 0x0 }, 7, 1, {TOAmberLED}},                // Throttle Only
                                    {0x14107AD1, { 0x0, 0x1, 0x0, 0x35,0x33,0x1, 0x1, 0x0 }, 7, 2, {ATAmberLED, ATredLED}},      // Active Trim
                                    {0x141090D1, { 0x0, 0x1, 0x0, 0x35,0x6, 0x1, 0x1, 0x0 }, 7, 1, {OneLEverAmberLED}},          // One Lever
                                    {0x141090D1, { 0x0, 0x1, 0x0, 0x35,0x8, 0x1, 0x1, 0x0 }, 7, 1, {QSAmberLED}},                // Quick Steer
                                    {0x14107AD1, { 0x0, 0x1, 0x0, 0x35,0x36,0x1, 0x1, 0x0 }, 7, 5, {at1, at2, at3, at4, at5}},   // Active Trim Up
                                    {0x14107AD1, { 0x0, 0x1, 0x0, 0x35,0x37,0x1, 0x1, 0x0 }, 7, 5, {at5, at4, at3, at2, at1}},   // Active Trim Down
                                    {0x141090D1, { 0x0, 0x1, 0x0, 0x35,0xC, 0x1, 0x1, 0x0 }, 7, 1, {ssALL}},                     // Start/Stop All.   CAN Comms still exist for SS "ALL" buttons
                                    {0x160C1ED1, { 0x0, 0xFF,0xA9,0x1, 0x0, 0x0, 0x0, 0x0 }, 7, 1, {SS_ALL_SINGLE}},             // Start Stop ALL for Singles  
                              };
struct IO_TrimSWEntity {
          long unsigned int query_ADDR;     // Trim Switches get polled by a specific address
          long unsigned int resp_ADDR;
          //unsigned char bdat[8];    // Trim Switches have differing DLC and data placement. Returning data differs most betwen "up and down". 
          unsigned int dlc;
          unsigned short lights;    // Trim Switches have no light but any light can be "attached" to a trim switch to indicate message received. 
          unsigned char corr_LIGHT[5]; // Most buttons have a corresponding light, but there is room here to use some buttons to test unrelated lights.
          unsigned char trimdir;    //
};                
unsigned short trimSwitchCOUNT = 10;
IO_TrimSWEntity switches[10] = {          // Note: Query data is pretty much the same for all ups and downs, with [1]=3 and [2] being 0x15 for up or 0x16 for down
                                    {0x14920B00, 0x1493000B, /*{ 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at4, at5}, 0xF},             // Trim All UP    
                                    {0x14920B00, 0x1493000B, /*{ 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at2, at1}, 0x10},            // Trim All DOWN
                                    {0x14920B00, 0x1493000B, /*{ 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at4, at5}, 0xF},             // STBD trim UP  
                                    {0x14920B00, 0x1493000B, /*{ 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at2, at1}, 0x10},            // STBD trim DOWN  
                                    {0x14920C00, 0x1493000C, /*{ 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at4, at5}, 0xF},             // PORT trim UP
                                    {0x14920C00, 0x1493000C, /*{ 0x0, 0x8, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at2, at1}, 0x10},            // PORT trim DOWN            //all trims are dlc 8 actually
                                    {0x14920D00, 0x1493000D, /*{ 0x0, 0x10,0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at4, at5}, 0xF},             // STBD CTR trim UP
                                    {0x14920D00, 0x1493000D, /*{ 0x0, 0x20,0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at2, at1}, 0x10},            // STBD CTR trim DOWN
                                    {0x14920E00, 0x1493000E, /*{ 0x0, 0x40,0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at4, at5}, 0xF},             // PORT CTR trim UP
                                    {0x14920E00, 0x1493000E, /*{ 0x0, 0x80,0x0, 0x0, 0x0, 0x0, 0x1, 0x0 },*/ 8, 5, {neutPORT, neutSTBD, at3, at2, at1}, 0x10}             // PORT CTR trim UP
                                };           
                                // Any provision for "both" detection?          

struct IO_HORNEntity{
    long unsigned int control_ADDR;
    unsigned char horn__ON[8];
    unsigned char horn__OFF[8];
    unsigned int dlc;
};
IO_HORNEntity horn = {0x14900b00, {0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x64}, {0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, 8};

bool heart1 = false;
bool heart2 = false;
String portSTATUS = "";
String stbdSTATUS = "";
void showMessage(String s, int fs) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.setCursor(0,0);
          display.println("   STARFISH II");  
          display.setCursor(0,8);
          display.setTextSize(fs);
          display.setTextColor(WHITE);
          display.println(s);
          display.display();
}  // showMessage
void showTestStatus(String s1, String s2, int fs) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.setCursor(0,0);
          display.println("***STARFISH II***");  
          display.setCursor(0,8);
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.println(s1);
          display.setCursor(0,16);
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.println(s2);    
          if (heart1 == true){
              display.setCursor(0,24);
              display.write(3);      // displays heartbeat
          }  // if
          if (heart2 == true){
              display.setCursor(23,24);
              display.write(3);      // displays heartbeat
          }  // if          
          display.display();
}  // showTestStatus
void larsen(const int *leds, int c) {                       
          for (int incr = 0; incr < c; incr++){
              digitalWrite(leds[incr], HIGH);
              delay(100);
              digitalWrite(leds[incr], LOW);
          }  // for
          delay(500);
          for (int incr = c-1; incr > -1; incr--){
              digitalWrite(leds[incr], HIGH);
              delay(100);
              digitalWrite(leds[incr], LOW);
          }  // for
} // Hoff summoned
void flashall(const int *leds, int c, int d, int iterations) {       
          for (int rep = 0; rep < iterations; rep++){                
              for (int incr = 0; incr < c; incr++){
                  digitalWrite(leds[incr], HIGH);
              }  // for
              delay(d);  // Delay after turning LEDs on
              for (int incr = 0; incr < c; incr++){
                  digitalWrite(leds[incr], LOW);
              }  // for
          }  // for
} 
void setup() {
          if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32        
                      display.clearDisplay();
                      display.display();     
                  }  // if
          showTestStatus("STARTING", "init...", 1);
          delay(222);              
          pinMode(portLEDpin, OUTPUT);
          pinMode(portCenterLEDpin, OUTPUT);
          pinMode(starboardCenterLEDpin, OUTPUT);
          pinMode(starboardLEDpin, OUTPUT);
          pinMode(wakeRELAYpin, OUTPUT);
          digitalWrite(portLEDpin, HIGH);
          digitalWrite(portCenterLEDpin, HIGH);
          digitalWrite(starboardCenterLEDpin, HIGH);
          digitalWrite(starboardLEDpin, HIGH);
          delay(1000);
          digitalWrite(portLEDpin, LOW);
          digitalWrite(portCenterLEDpin, LOW);
          digitalWrite(starboardCenterLEDpin, LOW);
          digitalWrite(starboardLEDpin, LOW);
          larsen( (const int[4]){starboardLEDpin, starboardCenterLEDpin, portCenterLEDpin, portLEDpin},4);
          pinMode(2, INPUT);                            // Setting pin 2 for /INT input not necessarily used in this application.
          pinMode(3, INPUT);                            // Setting pin 3 for /INT input not necessarily used in this application.
          if(CAN0.begin(CAN_250KBPS) == CAN_OK) showTestStatus("CAN0", "GOOD", 1); //Serial.print("can init ok!!\r\n");
          delay(1000);
          if(CAN1.begin(CAN_250KBPS) == CAN_OK) showTestStatus("CAN1", "GOOD", 1); //Serial.print("can init ok!!\r\n");
          delay(2000);
          digitalWrite(wakeRELAYpin, HIGH);         // WAKE ON - this turns the UUT on
          delay(100);
          marktime = millis();
          showTestStatus("STBY", "MOD COMMS", 1);
          while(true){      // modules and CAN Pad generate a signal on powerup
              CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
              rxId = CAN0.getCanId();                    // This can miss a slow-to-start console, for whatever reason it's slow starting.
              if((rxId == 0x1491020B) || (rxId != 0x160C20D1)) break;  // either way there's indication of "life" in the console.
              delay(100);
              if ((millis() - marktime) > 60000)break;  // At this point the Center Module is dead or not connected. The sleepmode will kick in.
          }  // while
          delay(2000);
              // HORN FAULT detection. Here at first the voltage drop across the load is checked. When HORN is activated, the UUT will provide a ground through the load, and voltage drop is reduced to millivolts. 
              // When in the the OFF/Open state, there is a full voltage drop across the load. The board has a voltage divider that drops the volts down, plus a 5.1V Zener diode in case of an overvoltage.
              // A Fault is when the HORN is conducting without calling it to conduct through the CAN bus, or not conducting when called, or continues to conduct if the OFF state is commanded via CAN
              // but the current still flows. Current flow through 27R Ceramic resistors is roughly 436mA at 12VDC.  Resistors will get to around 200F after 2 minutes. 
          int sensorValueHORN_OFF = 0;
          int sensorValueHORN_ON  = 0;
          showTestStatus("HORNTEST", "HORN", 1); 
            // Horn load should have UUT maximum voltage drop across it and the VDIV on the board cuts it down to protect the microprocessor. If there is no drop, there's likely a short through that load.
          sensorValueHORN_OFF = analogRead(sensorPinHORN);    
          if (sensorValueHORN_OFF >= ThreshHORN_OFF) {    //  757 ADC Count at 12 VDC. Some leeway here for power supply variance. 
            CAN0.sendMsgBuf(horn.control_ADDR, 1, 8, horn.horn__ON);        // Now turn it ON, meaning provide a ground through the load, and check the voltage drop again.
            delay(1000);  // Time to stabilize
            sensorValueHORN_ON = analogRead(sensorPinHORN);  // With the HORN grounded through the blue connector, then current flows in the load, voltage drop should reduce to millivolts.
            CAN0.sendMsgBuf(horn.control_ADDR, 1, 8, horn.horn__OFF);        // Now turn it OFF, causing full voltage drop across the load, but the reading already occured.
            if (sensorValueHORN_ON > ThreshHORN_ON)goto HORNFAULT;  // Not enough voltage drop, or none at all, meaning it did not get the command, or the circuit has issues. 
          } else { // A "horn ON" condition is detected, or some fault with it as full voltage drop is not happening. 
HORNFAULT:      // Here is a code trap to ensure that nothing proceeds after power down. Is the HORN still on after that? For a ground fault, ensure the drone is using WAKE power and not BATT
                // to power the HORN load because a hard short in the system caused by a bent pin will not cut off with just a WAKE disconnect. WAKE is controlled by a relay. This protects the drone.
            digitalWrite(wakeRELAYpin, LOW);         // KILL the WAKE whcih will cut voltage (+) supply to the HORN load. 
            showTestStatus("!FAULT DETECTED!", "HORN FAULT!", 1);  
            while(1) {      // Go into a permanent loop, this unit needs to remain off.
              flashall( (const int[4]){starboardLEDpin, starboardCenterLEDpin, portCenterLEDpin, portLEDpin},4, 500, 3);  // Light all, half second, 3 times
              delay(1000);
              digitalWrite(wakeRELAYpin, LOW);         // Just to be sure
            }  // while
          }  // if
          showTestStatus("HORN TEST", "COMP", 1); 
          delay(1000);
          showTestStatus("READY", "STATUS IDLE", 1);
}  //Setup
bool sleepmode = false;
void loop() {
          if (sleepmode == true) {
                larsen( (const int[4]){starboardLEDpin, starboardCenterLEDpin, portCenterLEDpin, portLEDpin},4);
                delay(30000);
                goto SSBYPASS;
          }  //
          if ((millis() - marktime) > 60000) {
                unsigned char AT_1_ON[8] = {0x0, 0x1, 0x0, 0x32, 0x3A, 0xF, 0x1, 0x0};
                unsigned char AT_1_OFF[8] = {0x0, 0x1, 0x0, 0x32, 0x3A, 0x0, 0x1, 0x0};
                unsigned char AT_2_ON[8] = {0x0, 0x1, 0x0, 0x32, 0x37, 0xF, 0x1, 0x0};
                unsigned char AT_2_OFF[8] = {0x0, 0x1, 0x0, 0x32, 0x37, 0x0, 0x1, 0x0};
                unsigned char AT_3_ON[8] = {0x0, 0x1, 0x0, 0x32, 0x38, 0xF, 0x1, 0x0};
                unsigned char AT_3_OFF[8] = {0x0, 0x1, 0x0, 0x32, 0x38, 0x0, 0x1, 0x0};
                unsigned char AT_4_ON[8] = {0x0, 0x1, 0x0, 0x32, 0x39, 0xF, 0x1, 0x0};
                unsigned char AT_4_OFF[8] = {0x0, 0x1, 0x0, 0x32, 0x39, 0x0, 0x1, 0x0};
                unsigned char AT_5_ON[8] = {0x0, 0x1, 0x0, 0x32, 0x35, 0xF, 0x1, 0x0};
                unsigned char AT_5_OFF[8] = {0x0, 0x1, 0x0, 0x32, 0x35, 0x0, 0x1, 0x0};
                for (int incr = 0; incr < 3; incr++) {
                     // Larsen the AT lights (light test)
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_1_ON);  
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_2_ON); 
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_1_OFF);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_3_ON);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_2_OFF);
                    //delay(117);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_4_ON);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_3_OFF);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_5_ON);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_4_OFF);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_4_ON);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_5_OFF);
                    delay(117);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_3_ON); 
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_4_OFF);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_2_ON);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_3_OFF);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_1_ON);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_2_OFF);
                    delay(117);
                    CAN0.sendMsgBuf(0x1410D190, 1, 7, AT_1_OFF);
            } // for
            digitalWrite(wakeRELAYpin, LOW);  // Let the UUT have a graceful powerdown.
            CAN0.sendMsgBuf(horn.control_ADDR, 1, 8, horn.horn__OFF);        // Just to make sure that HORN is OFF to reduce current consumption and not burn the drone down
            heart1 = false;
            heart2 = false;
            showTestStatus("SLEEPING", "zzzzzzz", 1);
            delay(222);
            larsen( (const int[4]){starboardLEDpin, starboardCenterLEDpin, portCenterLEDpin, portLEDpin},4);
            sleepmode = true;
            goto SSBYPASS;
          }  // if
          CAN0.sendMsgBuf(0x1410D090, 1, 7, blQuery);    // So-called "Backlight query" can get all canpad buttons and SS all to respond.
BUTTONRESCAN:
          delay(111);
          CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
          rxId = CAN0.getCanId();                    // Get message ID
          if (heart1 == false) {
              if (rxId == 0x1602228C) heart1 = true;              // heartbeat
          }   // if
          if (heart2 == false){
                unsigned char rxBufCAN2[8];
                CAN1.sendMsgBuf(0x1410D090, 1, 7, blQuery);
                CAN1.readMsgBuf(&len, rxBufCAN2);             
                if(CAN1.getCanId() == 0x1602228C) heart2 = true;
          }  // if
          for (unsigned short incr = 0; incr < digitalButtonCOUNT; incr++) {   // Go through data structure of the buttons and check if there is a button sending a message
            if (rxId == 0x141000D1) goto BUTTONRESCAN;          // filter out backlight hits
            if ((rxId == buttons[incr].button_ADDR ) && ((len == 7)? (rxBuf[4] == buttons[incr].bdat[4]) :  (rxBuf[6] == buttons[incr].bdat[6])) ) { // Button press detected...
              showTestStatus("ACTIVE", "CANBUTT RESP", 1);
              delay(50);
              marktime = millis();
              for (unsigned short lightseq = 0; lightseq < buttons[incr].lights; lightseq++){
                lightIO_templ[4] = buttons[incr].corr_LIGHT[lightseq];   // Current light for the button or its indicator
                lightIO_templ[5] = 0xF;  // ON
                CAN0.sendMsgBuf(lightIO_ADDR, 1, lightIO_dlc, lightIO_templ);    // on
                delay(100);
                lightIO_templ[5] = 0x0;  // OFF
                CAN0.sendMsgBuf(lightIO_ADDR, 1, lightIO_dlc, lightIO_templ);    // off
                delay(100);
              }  // for
              goto SSBYPASS;    
            } // else if (rxId == buttons[incr].button_ADDR ) goto RETRY_BUTTONS;  // if
          }  // for
          // For trim switches, they are queried individually
          for (unsigned short incr = 0; incr < trimSwitchCOUNT; incr++) {  
              unsigned char trQUERY[8] = {0x0, 0x3, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0};
              trQUERY[2] = switches[incr].trimdir;        // Set query data for up (0xF) or down (0x10)
 RETRY_TRIM001:  //  this turns the loop back around to focus on detecting these parameters which are sent rapid-fire instead of having to go through everything else again.
              CAN0.sendMsgBuf(switches[incr].query_ADDR, 1, switches[incr].dlc, trQUERY);
              delay(22);   // 22 milliseconds appears to be ideal
              CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
              if((rxBuf[6] == 0) && (rxBuf[7] == 0))     continue;                // no data..
              rxId = CAN0.getCanId();                    // Get message ID
              if (rxId == 0x141000D1){goto RETRY_TRIM001; }        // Backlight... 
              if (rxId == 0x1602228C){ goto RETRY_TRIM001;}          // heartbeat 369238668
              if ((rxId == switches[incr].resp_ADDR ) &&  (rxBuf[2] == switches[incr].trimdir)) {    //if ((rxId == switches[incr].resp_ADDR ) && (rxBuf[6] == switches[incr].bdat[6]) && (rxBuf[2] == switches[incr].trimdir)) { 
                  showTestStatus("ACTIVE", "TRIM SW RESP", 1);
                  delay(50);
                  marktime = millis();
                  for (unsigned short lightseq = 0; lightseq < switches[incr].lights; lightseq++){
                    lightIO_templ[4] = switches[incr].corr_LIGHT[lightseq];   // Current light for the button or its indicator
                    lightIO_templ[5] = 0xF;  // ON
                    CAN0.sendMsgBuf(lightIO_ADDR, 1, lightIO_dlc, lightIO_templ);    // on
                    delay(100);
                    lightIO_templ[5] = 0x0;  // OFF
                    CAN0.sendMsgBuf(lightIO_ADDR, 1, lightIO_dlc, lightIO_templ);    // off
                    delay(100);
                  }  // for
                  goto SSBYPASS;    
                  break;      // Having found it and done the indication work, break out of the loop
              } else if (rxId == switches[incr].resp_ADDR ) goto RETRY_TRIM001; // if  
            }  // for
          delay(100);
          sensorValuePORT = analogRead(sensorPinPort);
          delay(1);      // ADC reads are approx 200uS so 1mS should be good enough.
          sensorValuePORTCENTER = analogRead(sensorPinPortCenter);
          delay(1);
          sensorValueSTBDCENTER = analogRead(sensorPinStarboardCenter);
          delay(1);
          sensorValueSTBD = analogRead(sensorPinStarboard);


          // may have to adapt some things here due to floaties. It depends on the AD side between UUT and the sensor port. If it's not pulled low, then raise the number. Not-pulled-low results are still very small numbers roughly 1 to 5 out of 1024.
          if (sensorValuePORT > 600) {           // "healthy" systems will run over 700 (around 735 - 755) on the ADC reading with 12VDC input. Ultimately 9.4 - 9.5 VDC reaches the VDC while the indication LED is getting around 3VDC
            digitalWrite(portLEDpin, HIGH);       // But be aware that the vdiv before the ADC can affect this overall some some adjustment of the code may be necessary if different resistors were used. 
            lightIO_templ[4] = SS_PORT;   // Current light for the button or its indicator
            goto SSCALLBACK;
          }  // if
          else if (sensorValuePORTCENTER > 600) {
            digitalWrite(portCenterLEDpin, HIGH);
            lightIO_templ[4] = SS_PORT_CENTER;   // Current light for the button or its indicator
            goto SSCALLBACK;
          }  // if
          else if (sensorValueSTBDCENTER > 600) {
            digitalWrite(starboardCenterLEDpin, HIGH);
            lightIO_templ[4] = SS_STBD_CENTER;   // Current light for the button or its indicator
            goto SSCALLBACK;
          } // if
          else if (sensorValueSTBD > 600) {
            digitalWrite(starboardLEDpin, HIGH);
            lightIO_templ[4] = SS_STBD;   // Current light for the button or its indicator
            goto SSCALLBACK;
          } // if    
          showTestStatus("NO INPUT", "IDLE", 1);
          delay(50);
          goto SSBYPASS;
SSCALLBACK:      // Handle individual Start/Stop button responses
         //  for (unsigned short lightseq = 0; lightseq < buttons[incr].lights; lightseq++){
                showTestStatus("ACTIVE", "SS SW RESP", 1);
                marktime = millis();
                lightIO_templ[5] = 0xF;  // ON
                CAN0.sendMsgBuf(lightIO_ADDR, 1, lightIO_dlc, lightIO_templ);    // on
                delay(100);
                lightIO_templ[5] = 0x0;  // OFF
                CAN0.sendMsgBuf(lightIO_ADDR, 1, lightIO_dlc, lightIO_templ);    // off
                delay(100);
          //    }  // for
SSBYPASS:
  ;     // Arduino IDE requires this :-)   
}  // loop
//--------------------------------------------------------------------------------------------------------------------------------------------------
// Artifacts of code
 /*   ---------------------SWING
   //---------------------------------------------------------------------------------------------------------------
//** Axis Track code

unsigned char axisquery[8] = { 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };           // Send to the module to find rotation/position information
unsigned char axisreturn_STBD[8] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };            // contains the information coming back from the query
unsigned char axisreturn_PORT[8] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };            // contains the information coming back from the query


long unsigned int STBDaddr = 0x14920B01;                            //Singles also use 
long unsigned int replySTBD = 0x1493010B;
long unsigned int PORTaddr = 0x14920C01;
long unsigned int replyPORT = 0x1493010C;

unsigned char* subarray (byte arn[], unsigned char len) {
  return NULL;
}
// Axis Track code**
//---------------------------------------------------------------------------------------------------------------

              Serial.print("DEB100 Swing Detect routines..1111..");
                  CAN0.sendMsgBuf(PORTaddr, 1, 7, axisquery);  // DLC appears to always be 8
                  delay(500);   // 22 milliseconds appears to be ideal
                  CAN0.readMsgBuf(&len, axisreturn_PORT);              // Read data: len = data length, buf = data byte(s)
                   String sb = "";      //------------------------------------------------------------------------------------------deb100 remove when done
                   for(int i = 0; i<len; i++)                // Print each byte of the data
                              {
                                if(axisreturn_PORT[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
                                {
                                  sb += "0";
                                  //Serial.print("0");
                                }  // if
                                sb += String(axisreturn_PORT[i], HEX) + " ";
                                //Serial.print(axisreturn_PORT[i], HEX);
                                //Serial.print(" ");
                              }  // for
        
            
                    Serial.print(sb + "," );  //deb100

                  Serial.print("DEB100 Swing Detect routines..2222..");
              
                  CAN0.sendMsgBuf(STBDaddr, 1, 7, axisquery);  // DLC appears to always be 8
                  delay(500);   
                  CAN0.readMsgBuf(&len, axisreturn_STBD);              // Read data: len = data length, buf = data byte(s)
                   for(int i = 0; i<len; i++)                // Print each byte of the data
                              {
                                if(axisreturn_STBD[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
                                {
                                  sb += "0";
                                  //Serial.print("0");
                                }  // if
                                sb += String(axisreturn_STBD[i], HEX) + " ";
                                //Serial.print(axisreturn_STBD[i], HEX);
                                //Serial.print(" ");
                              }  // for
        
            
                    Serial.print(sb + "," );  //deb100
                 Serial.print("DEB100 Swing Detect routines..3333..");
              //    ---------------------SWING
              //    ---------------------SWING
              //    ---------------------SWING
              //    ---------------------SWING
              Serial.print("DEB100 Swing Detect routines....");
              len = 0;
              double axisangle_PORT = 0;
              double axisangle_STBD = 0;
              if (portTRY++ < 50) { // Not every console has a PORT so don't continue to try it if it's not replying.
                  Serial.print("DEB100 Swing Detect routines PORT DETECTION");
                  CAN1.sendMsgBuf(PORTaddr, 1, 8, axisquery);  // DLC appears to always be 8
                  delay(22);   // 22 milliseconds appears to be ideal
                  CAN1.readMsgBuf(&len, axisreturn_PORT);              // Read data: len = data length, buf = data byte(s)
                  if (len > 0) { // Assuming that there is something
                    unsigned char positarray[4] = {};
                    memcpy(positarray, axisreturn_PORT+4, 4);
                    int posval = int((positarray[0]) << 24 |
                                     (positarray[1]) << 16 |
                                     (positarray[2]) << 8 |
                                     (positarray[3]));
                    axisangle_PORT = posval * 0.02197265625;
                    if(abs(axisangle_PORT - 84) < 6){
                      portSTATUS = "PORT_RWOT";
                    } else if (abs(axisangle_PORT - 221) < 6){
                      portSTATUS = "PORT_FWOT";
                    } else {    // Not falling into a sought extreme angle check, null it
                      portSTATUS = String(axisangle_PORT, 2);;
                    }  // if        
                    portTRY = 0;
                  } // if 
              } else {
                portSTATUS = "";
              } // if
              
              CAN1.sendMsgBuf(STBDaddr, 1, 8, axisquery);  // DLC appears to always be 8
              delay(500);   
              CAN1.readMsgBuf(&len, axisreturn_STBD);              // Read data: len = data length, buf = data byte(s)
              if (len > 0) { // Assuming that there is something
                Serial.print("DEB100 Swing Detect routines STBD DETECTION");
                unsigned char positarray[4] = {};
                memcpy(positarray, axisreturn_STBD+4, 4);
                int posval = int((positarray[0]) << 24 |
                                 (positarray[1]) << 16 |
                                 (positarray[2]) << 8 |
                                 (positarray[3]));
                axisangle_STBD = posval * 0.02197265625;
                stbdSTATUS = ((portTRY < 50) ? "STBD" : "SINGLE");
                if(abs(axisangle_STBD - 93) < 6){
                  stbdSTATUS += "_RWOT";
                } else if (abs(axisangle_STBD - 319) < 6){
                  stbdSTATUS += "_FWOT";
                } else {    // Not falling into a sought extreme angle check, null it
                  stbdSTATUS = String(axisangle_STBD, 2);
                }  // if
                    // Challenge here: deb100 if Port is offline for lack of port, then the result is "single R/FWOT"
                      //deb100
                      // from here see what the original .NET program does for parameters checking... 
                    //deb100 might want to display the angle instead? 
              } // if 
  

              //    ---------------------SWING
              //    ---------------------SWING
              //    ---------------------SWING
              //    ---------------------SWING
              //    ---------------------SWING */


  /*
              String sb = "";      //------------------------------------------------------------------------------------------deb100 remove when done
              for(int i = 0; i<len; i++)                // Print each byte of the data
                              {
                                if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
                                {
                                  sb += "0";
                                  //Serial.print("0");
                                }  // if
                                sb += String(rxBuf[i], HEX) + " ";
                                //Serial.print(rxBuf[i], HEX);
                                //Serial.print(" ");
                              }  // for
        
            
              Serial.print(sb + "," );  //deb100
            
              Serial.print(rxId, HEX);  //deb100
            
              Serial.print("..");
              Serial.print(switches[incr].resp_ADDR, HEX);  //deb100
              Serial.print("..");
              Serial.println(switches[incr].trimdir, HEX); 
              //----------------------------------------------------------------------------------------------------------deb100 remove when done
*/
