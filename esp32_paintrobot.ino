#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

#include "soc/rtc_wdt.h"
#include "esp_int_wdt.h"
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 300

//last change made from 1.2.0: a)changed vertical reset distance of top drive from 2.5m to 6.5m
//last change made from 1.2.1: a)interchanged mac ids of site blue and site yellow
//last change made from 1.2.2: a)during vertical move and paint. reset distance of top drive increased from 2.5m to 6m
//last change made from 1.2.3: a)during vertical reset carriages will end up on top
//last change made from 1.2.4: a)included curve negotiation in bottom of north and south side  
//last change made from 1.2.5: a)changed curve negotiation speed in bottom of north and south side to 30% from 50%
//                             b)accomodating separate clearances for north and south sides for top drive spray and move    
//last change made from 1.2.6: a)included option to stop south horizontal drive when it is running on behalf of north or top
//                             b)included option to engage south horizontal drive during reset when only top or north is selected     
String programversion="1.2.7";
int whethermaster;

//new master 10:97:BD:D2:D3:BC {0x10, 0x97, 0xBD, 0xD2, 0xD3, 0xBC}
//old master 24:6F:28:7B:56:08 {0x24, 0x6F, 0x28, 0x7B, 0x56, 0x08}

//on site
uint8_t broadcastAddress[6][6] = {{0x58, 0xBF, 0x25, 0x9F, 0x56, 0x8C},{0x24, 0x0A, 0xC4, 0x61, 0x2D, 0xFC},{0x78, 0xE3, 0x6D, 0x0B, 0x48, 0x8C},{0x78, 0xE3, 0x6D, 0x09, 0xCF, 0x88},{0x24, 0x62, 0xAB, 0xE0, 0x01, 0xB0},{0x10, 0x97, 0xBD, 0xD2, 0xD3, 0xBC}};
String ucname[6]={"Site-Blue-North","Site-Green-South","Site-Orange-Top","Site-Pink-West","Site-Yellow-East","Site-Master"};
//blue 58:BF:25:9F:56:8C green 24:0A:C4:61:2D:FC orange 78:E3:6D:0B:48:8C pink 78:E3:6D:09:CF:88 yellow 24:62:AB:E0:01:B0 black 10:97:BD:D2:D3:BC

//at home
//uint8_t broadcastAddress[6][6] = {{0x78, 0xE3, 0x6D, 0x09, 0x2F, 0x80},{0x40, 0x91, 0x51, 0xB2, 0xA9, 0x58},{0x24, 0x62, 0xAB, 0xE0, 0x83, 0x54},{0x24, 0x62, 0xAB, 0xB0, 0x81, 0xC4},{0x78, 0xE3, 0x6D, 0x09, 0xCF, 0x88},{0x10, 0x97, 0xBD, 0xD2, 0xD3, 0xBC}};
//String ucname[6]={"Home-Blue-North","Home-Green-South","Home-Orange-Top","Home-Pink-West","Home-Yellow-East","Home-Master"};
//blue 78:E3:6D:09:2F:80 green 40:91:51:B2:A9:58 orange 24:62:AB:E0:83:54 pink 24:62:AB:B0:81:C4 yellow 78:E3:6D:09:CF:88 black 10:97:BD:D2:D3:BC

//enable text on oled screen and serial monitor
boolean enableoled;
boolean enableserialmonitor;
boolean enablerefreshdisplay;

// related to limit bypassing
boolean whetherlimitsbypassed;
boolean templimitbypassstatus;

//declarations related to ESPnow
unsigned long begintime;
unsigned long endtime;

unsigned long deltimeout=100;
unsigned long rectimeout=500;

int mysenderid;
int mydriveindex;
int currtask;

String inputstring;

boolean whetherstopsignalreceived;
boolean whethercompletefeedbackreceived;

boolean awaiting_feedback;
boolean feedback_recieved;

boolean await_deliverystatus;
boolean deliverystatus;

//Structure to send data; Must match the receiver structure
typedef struct struct_message 
{
  int senderid;
  int messagetype;
  int travelvalue;
  int speedpercentvalue;
  char messagetxt[20];
} struct_message;

// Create a struct_message 
struct_message sentMessage;

// Create a struct_message to hold incoming sensor readings
struct_message incomingMessage;

esp_now_peer_info_t peerInfo;

//declarations related to oled screen
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//output speed at 50Hz (full frequency) is 21.1rpm 
//dia of the wheel is 200mm (hence circumference is pi*D which is 628mm)
//Hence at full frequency it operates at a speed of mm/sec
const float xfullspeed=220.847; //in mm/sec

//output speed at 50Hz (full frequency) is 39.7rpm 
//circumference is pi*D which is 252mm)
//Hence at full frequency it operates at a speed of mm/sec

const float yfullspeed=356.91; //in mm/sec

//related to input from serial from Rasp Pi
int recaxisspeed;
int recxaxisspeed;
int recyaxisspeed;
int recinchdistance;

boolean inchenable[5];
String inchenablestring;
boolean engageenable[5];
boolean jobcompletestatus[5];
boolean feedbackrecstatus[5];
String engageenablestring;
String jobcompletestatusstring;
int driveindexarray[]={4,3,0,1,2,5};

int inchdistance=100; // in mm
/* drive index list
 * 1 East   - Yellow  - 4
 * 2 West   - Pink    - 3
 * 3 North  - Blue    - 0
 * 4 South  - Green   - 1
 * 5 Top    - Orange  - 2
 * 6 Master - Black   - 5
 */

unsigned long starttime;
unsigned long currtime;
unsigned long timediff;
unsigned long reqtime;

int linesloaded;
String line[6];

//relay pins decalaration
const int horizontal_relay_direction_pin=0;
const int vertical_relay_direction_pin=17;
const int drive_engage_relay_pin=5;
const int plunger_engage_relay_pin=18;

//drive speed control pins declaration
const int horizontal_drive_speed_control_pin=23;
const int horizontal_drive_speed_reference_pin=19;
const int vertical_drive_speed_control_pin=16;
const int vertical_drive_speed_reference_pin=4;

//decalaration of input pins
const int horizontal_limit_extreme_reference_start_pin=36;
const int horizontal_limit_extreme_reference_end_pin=39;
const int horizontal_limit_coach_reference_start_pin=34;
const int horizontal_limit_coach_reference_end_pin=35;

const int vertical_limit_extreme_reference_start_pin=32;
const int vertical_limit_extreme_reference_end_pin=33;
const int vertical_limit_coach_reference_start_pin=25;
const int vertical_limit_coach_reference_end_pin=26;

const int spare_input_pin1=27;
const int spare_input_pin2=14;
const int spare_input_pin3=12;

//declaration relevant to pwm output
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;

const int testtimeinterval=2000;

void setup() 
{
  currtask=0;
  declarerelayoutputpins();
  declaredriveoutputpins();
  declareinputpins();
  //declareinputpinspulldown();
  //declareinputpinspullup();
  setupoleddisplay();
  setupespnow();  
  Serial.begin(115200);
  initenablearrays();
  displayscreen("Starting");
  displayscreen(programversion);
  displayscreen(ucname[mysenderid]);
  //testrelayboard();
  //testdrivespeedcontrol();
  //testhorizdrive();
  //horizdistancetravel(500,50,1);
  //scaninputpinsforhigh();
  //horizdistancetravel(5000,50,1);
  //vertdistancetravel(500,50,1);
  //vertdistancetravelandpaint(500,50,1,"250*400");
  //sendtestdata();
  if(whethermaster==1)
  {
    commtomaster();  
  }
  //esp_task_wdt_init(WDT_TIMEOUT, false); //enable panic so ESP32 restarts
  //esp_task_wdt_add(NULL); //add current thread to WDT watch

  //Serial.print("setup() running on core ");
  //Serial.println(xPortGetCoreID());

  //rtc_wdt_protect_off();
  //rtc_wdt_disable();
  //disableCore0WDT();
  //disableLoopWDT();
}
void initenablearrays()
{
  whetherlimitsbypassed=LOW;
  
  inchenablestring="00000";
  engageenablestring="00000";
  jobcompletestatusstring="00000";
  for(int i=0;i<5;i++)
  {
    inchenable[i]=LOW;
    engageenable[i]=LOW;
    jobcompletestatus[i]=LOW;
    feedbackrecstatus[i]=LOW;
  }
}
void allotinchenablearray(boolean value)
{
  inchenablestring="";
  for(int i=0;i<5;i++)
  {
    inchenable[i]=value;
    if(value==HIGH)
    {
      inchenablestring+="1";
    }
    else
    {
      inchenablestring+="0";
    }
  }
}
void allotengageenablearray(boolean value)
{
  engageenablestring="";
  for(int i=0;i<5;i++)
  {
    engageenable[i]=value;
    if(value==HIGH)
    {
      engageenablestring+="1";
    }
    else
    {
      engageenablestring+="0";
    }
  }
}
void commtomaster()
{
  displayscreen("stat*commstart");
  delay(1000);
  displayscreen("stat*noresettpos");
}
void serialEvent() 
{
  if(enablerefreshdisplay)
  {
     refresholeddisplay();  
  }
  while(Serial.available())
  {
          inputstring="";
          inputstring=Serial.readString();
          //Serial.println(inputstring);
  }
  currtask=10;
}
void checkresetposition()
{
  //delay(1000);
  displayscreen("stat*noresettpos");
}
void loop() 
{
  delay(10);
  //Serial.print("loop() running on core ");
  //Serial.println(xPortGetCoreID());
  
  //esp_task_wdt_reset();
  if (Serial.available())
  {
        serialEvent();
  }
  //scaninputpinsforhigh();
  //scaninputpinsforlow();
  //vTaskDelay(10);
  
  switch(currtask)
  {
    // awaiting feedback from all carraiges. relevant only for master 
    case -1:
    {
      //displayscreen(jobcompletestatusstring);
      delay(10);
      int whetherallcarraigescompletedjobs=0;
      for(int i=0;i<5;i++)
      {
        if(jobcompletestatus[i]==HIGH)
        {
          whetherallcarraigescompletedjobs=1;
        }
      }
      if(whetherallcarraigescompletedjobs==1)
      {
        //still carraiges are engaged in work. Master has to wait
        //Serial.println("not here");
        if(whethercompletefeedbackreceived==LOW)
        {
          currtask=-1;
        }
        else
        {
          currtask=9;
        }
      }
      if(whetherallcarraigescompletedjobs==0)
      {
        //all carraiges for which command is given have completed work
        currtask=0;
        //Serial.println("here");
        displayscreen("stat*jobcomp");
      }
      break;
    }
    // reset both horiz and vert carraiges
    case -2:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // reset both horiz and vert carraiges
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      displayscreen("Resetting horizontally");
      updateDisplay();
      templimitbypassstatus=whetherlimitsbypassed;
      whetherlimitsbypassed=LOW;
      
      //SendDataAwaitFeedback(int recipientid, int messagetype, int travelvalue, int speedpercentvalue, String sendmsg, boolean awaitdelfeedback, boolean awaitrecfeedback)
      //SendDataAwaitFeedback(driveindexarray[i],-2,recxaxisspeed,recyaxisspeed,"",HIGH, HIGH)
      
      //if message recieved by North or Top carraige no horizontal movement is required
      if(mysenderid!=0 && mysenderid!=2)
      {
        horizdistancetravel(35000,incomingMessage.travelvalue,2);        
      }
      else
      {
        displayscreen("No Horz drv rst");
      }
      
      displayscreen("Resetting vertically");
      
      if(mysenderid==2)
      {
        displayscreen("Resetting top");
        vertdistancetravel(6500,incomingMessage.speedpercentvalue,2);
      }
      else
      {
        vertdistancetravel(2500,incomingMessage.speedpercentvalue,1);
      }
      
      if(!whetherstopsignalreceived)
      {
        displayscreen("Sending comp FB");
        SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
        currtask=0;  
      }
      whetherlimitsbypassed=templimitbypassstatus;
      break;
    }
    // idle
    case 0:
    {
      // idle
      digitalWrite(LED_BUILTIN,LOW);
      delay(10);
      break;
    }
    // moving horizontal forward
    case 1:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // moving horizontal forward
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      updateDisplay();
      //if message recieved by North or Top carraige no horizontal movement is required
      if(mysenderid!=0 && mysenderid!=2)
      {
        horizdistancetravel(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,1);
        if(!whetherstopsignalreceived)
        {
          displayscreen("Sending comp FB");
          SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
          currtask=0;  
        }  
      }
      else
      {
          displayscreen("No Horiz drive");
          SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
          currtask=0;
      }
      break;
    }
    // moving horizontal reverse
    case 2:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // moving horizontal reverse
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      updateDisplay();
      //if message recieved by North or Top carraige no horizontal movement is required
      if(mysenderid!=0 && mysenderid!=2)
      {
        horizdistancetravel(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,2);
        if(!whetherstopsignalreceived)
        {
          displayscreen("Sending comp FB");
          SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
          currtask=0;  
        }  
      }
      else
      {
        displayscreen("No Horiz drive");
        SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
        currtask=0;
      }
      break;
    }
    // moving vertical up
    case 3:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // moving vertical up
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      updateDisplay();
      vertdistancetravel(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,1);
      if(!whetherstopsignalreceived)
      {
        displayscreen("Sending comp FB");
        SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
        currtask=0;  
      }
      break;
    }
    // moving vertical down
    case 4:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // moving vertical down
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      updateDisplay();
      vertdistancetravel(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,2);
      if(!whetherstopsignalreceived)
      {
        displayscreen("Sending comp FB");
        SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
        currtask=0;  
      }
      break;
    }
    // moving vertical and painting
    case 5:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // moving vertical up and painting
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      updateDisplay();
      String breakstring = String(incomingMessage.messagetxt);
      
      if(digitalRead(vertical_limit_extreme_reference_start_pin)==LOW)
      {
        displayscreen("V&P from bottom");
        if(mysenderid==2)
        {
          vertdistancetravelandpaint(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,1, breakstring);
        }
        else
        {
          vertdistancetravelandpaintcurveneg(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,1, breakstring);            
        }
      }
      else if(digitalRead(vertical_limit_extreme_reference_end_pin)==LOW)
      {
        displayscreen("V&P from top");
        if(mysenderid==2)
        {
          vertdistancetravelandpaint(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,2, breakstring);
        }
        else
        {
          vertdistancetravelandpaintcurveneg(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,2, breakstring);            
        }
      }
      else if(digitalRead(vertical_limit_extreme_reference_end_pin)==HIGH && digitalRead(vertical_limit_extreme_reference_start_pin)==HIGH)
      {
        displayscreen("Not in limits");
        displayscreen("Resetting vertically");
        if(mysenderid==2)
        {
          displayscreen("Resetting top");
          vertdistancetravel(6500,incomingMessage.speedpercentvalue,2);
          vertdistancetravelandpaint(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,1, breakstring);
        }
        else
        {
          vertdistancetravel(2500,incomingMessage.speedpercentvalue,2);
          vertdistancetravelandpaintcurveneg(incomingMessage.travelvalue,incomingMessage.speedpercentvalue,1, breakstring);  
        }
      }
      
      if(!whetherstopsignalreceived)
      {
        displayscreen("Sending comp FB");
        SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
        currtask=0;  
      }
      break;
    }
    // plunger engage for inch
    case 6:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // plunger engage for inch
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      updateDisplay();
      spray(incomingMessage.travelvalue);
      if(!whetherstopsignalreceived)
      {
        displayscreen("Sending comp FB");
        SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
        currtask=0;  
      }
      break;
    }
    // stopping all drives
    case 7:
    {
      SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // stopping all drives
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      displayscreen("All drive stop");
      horizontal_drive_speed_select(0);
      vertical_drive_speed_select(0);
      plunger_disengage();
      digitalWrite(drive_engage_relay_pin,LOW);
      displayscreen("Sending comp FB");
      SendDataAwaitFeedback(incomingMessage.senderid,9,mydriveindex,0,"fb",LOW,LOW);
      currtask=0;
      whetherstopsignalreceived=LOW;
      break;
    }
    // toggling limits
    case 8:
    {
      // toggling limits
      //SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      displayscreen("Message incoming");
      if(incomingMessage.travelvalue==0)
      {
        whetherlimitsbypassed=LOW;
        displayscreen("limits engaged");
      }
      else
      {
        whetherlimitsbypassed=HIGH;
        displayscreen("limits bypassed");
      }
      delay(1000);
      currtask=0;
      break;
    }
    // processing incoming feedback. relevant only for master 
    case 9:
    {
      //SendDataAwaitFeedback(incomingMessage.senderid,0,0,0,"fb",LOW,LOW); //send feedback message that instruction recieved
      // incoming feedback for job completion. relevant only for master 
      digitalWrite(LED_BUILTIN,HIGH);
      if(enablerefreshdisplay)
      {
        refresholeddisplay();  
      }
      //displayscreen("FB processing");
      for(int i=0;i<5;i++)
      {
        if(feedbackrecstatus[i]==HIGH)
        {
          jobcompletestatus[i]=LOW;
          jobcompletestatusstring[i]='0';
          feedbackrecstatus[i]=LOW;
        }
      }
      Serial.print("jobpendingstatusupd:");
      Serial.println(jobcompletestatusstring);
      //displayscreen(jobcompletestatusstring);
      whethercompletefeedbackreceived=LOW;
      currtask=-1;
      break;
    }
    // processing incoming serial input from computer. relevant only for master 
    case 10:
    {
      //Serial.flush();
      // processing incoming serial input from computer. relevant only for master 
      //displayscreen("serialevent");
      Serial.println(inputstring); 
      String instructString="";
      int instructval=0;
      int xval=0;
      int yval=0;
      int zval=0;
      int wval=0;
      String Xval="";
      String Yval="";
      String Zval="";
      String Wval="";
      String Vval="";
      int countlimiter=0;
      
      for(int i=0;i<inputstring.length();i++)
      {
          char inChar = inputstring[i];
          if(inChar=='*')
          {
             countlimiter++;
             continue;
          }
          switch(countlimiter)
          {
            case 0:
              instructString+=inChar;
              break;
            case 1:
              Xval+=inChar;
              break;
            case 2:  
              Yval+=inChar;
              break;
            case 3:  
              Zval+=inChar;
              break;
            case 4:  
              Wval+=inChar;
              break;
            case 5:  
              Vval+=inChar;
              break;    
          }
      }
      instructval=instructString.toInt();
      /*
      Serial.print("InstructString: ");
      Serial.println(instructString);
      Serial.print("Xval: ");
      Serial.println(Xval);
      Serial.print("Yval: ");
      Serial.println(Yval);
      */
      //initiate communication with master controller
      if(instructval==1)
      {
          initenablearrays();
          commtomaster();
          currtask=0;
      }
      //reset command recieved
      if(instructval==2)
      {
       //reset command recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       int atleastonetaskdesp=0;
       int eithertopornorthengaged=0;
       int whethersouthengaged=0;
        
       recxaxisspeed=xval;
       recyaxisspeed=yval;
       //displayscreen(Xval);
       //displayscreen(Yval);
       for(int i=0;i<5;i++)
       {
          if(inchenable[i]==HIGH)
          {
            if(SendDataAwaitFeedback(driveindexarray[i],-2,recxaxisspeed,recyaxisspeed,"",HIGH, HIGH))
            {
              jobcompletestatus[i]=HIGH;
              jobcompletestatusstring[i]='1';
              currtask=-1;
              atleastonetaskdesp=1;
            }
            if(driveindexarray[i]==1)
            {
                whethersouthengaged=1;               
            }
            if(driveindexarray[i]==0 || driveindexarray[i]==2)
            {
                eithertopornorthengaged=1;            
            }
          }
       }
       if(eithertopornorthengaged==1 && whethersouthengaged==0)
       {
          if(SendDataAwaitFeedback(1,-2,recxaxisspeed,recyaxisspeed,"",HIGH, HIGH))
          {
              jobcompletestatus[3]=HIGH;
              jobcompletestatusstring[3]='1';
              currtask=-1;
              atleastonetaskdesp=1;    
          }
       }
       if(atleastonetaskdesp==1)
       {
        displayscreen("stat*jobprog");
        currtask=-1;
       }
       else
       {
        displayscreen("stat*tskdspfail");
        currtask=0;
       }
      }
      //inch up (vertical drives) recieved
      if(instructval==3)
      {
       //inch up (vertical drives) recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       int atleastonetaskdesp=0;
        
       recaxisspeed=xval;
       recinchdistance=yval;
       inchdistance=recinchdistance;
       //displayscreen(Xval);
       //displayscreen(Yval);
       for(int i=0;i<5;i++)
       {
          if(inchenable[i]==HIGH)
          {
            if(SendDataAwaitFeedback(driveindexarray[i],3,inchdistance,recaxisspeed,"",HIGH, HIGH))
            {
              jobcompletestatus[i]=HIGH;
              jobcompletestatusstring[i]='1';
              currtask=-1;
              atleastonetaskdesp=1;
            }
          }
       }
       if(atleastonetaskdesp==1)
       {
        displayscreen("stat*jobprog");
        currtask=-1;
       }
       else
       {
        displayscreen("stat*tskdspfail");
        currtask=0;
       }
       //displayscreen("stat*inchcomp");
      }
      //inch down (vertical drives) recieved
      if(instructval==4)
      {
       //inch down (vertical drives) recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       recaxisspeed=xval;
       recinchdistance=yval;
       inchdistance=recinchdistance;

       int atleastonetaskdesp=0;
       
       //displayscreen(Xval);
       //displayscreen(Yval);
       for(int i=0;i<5;i++)
       {
          if(inchenable[i]==HIGH)
          {
            if(SendDataAwaitFeedback(driveindexarray[i],4,inchdistance,recaxisspeed,"",HIGH, HIGH))
            {
              jobcompletestatus[i]=HIGH;
              jobcompletestatusstring[i]='1';
              currtask=-1;
              atleastonetaskdesp=1;    
            }
          }
       }
       if(atleastonetaskdesp==1)
       {
         displayscreen("stat*jobprog");
         currtask=-1;
       }
       else
       {
          displayscreen("stat*tskdspfail");
          currtask=0;
       }
       //displayscreen("stat*inchcomp");
      }
      //inch forward (horizontal drives) recieved
      if(instructval==5)
      {
       //inch forward (horizontal drives) recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       recaxisspeed=xval;
       recinchdistance=yval;
       inchdistance=recinchdistance;

       int atleastonetaskdesp=0;
       int eithertopornorthengaged=0;
       int whethersouthengaged=0;
       
       //displayscreen(Xval);
       //displayscreen(Yval);
       for(int i=0;i<5;i++)
       {
          if(inchenable[i]==HIGH)
          {
            if(driveindexarray[i]!=0 && driveindexarray[i]!=2)
            {
              if(SendDataAwaitFeedback(driveindexarray[i],1,inchdistance,recaxisspeed,"",HIGH, HIGH))
              {
                jobcompletestatus[i]=HIGH;
                jobcompletestatusstring[i]='1';
                currtask=-1;
                atleastonetaskdesp=1;    
              }
              if(driveindexarray[i]==1)
              {
                whethersouthengaged=1;               
              }
            }
            else
            {
              eithertopornorthengaged=1;
            }
          }
       }
       if(eithertopornorthengaged==1 && whethersouthengaged==0)
       {
          if(SendDataAwaitFeedback(1,1,inchdistance,recaxisspeed,"",HIGH, HIGH))
          {
              jobcompletestatus[3]=HIGH;
              jobcompletestatusstring[3]='1';
              currtask=-1;
              atleastonetaskdesp=1;    
          }
       }
       if(atleastonetaskdesp==1)
       {
        displayscreen("stat*jobprog");
        currtask=-1;
       }
       else
       {
        displayscreen("stat*tskdspfail");
        currtask=0;
       }       
       
       //displayscreen("stat*inchcomp");
      }
      //inch reverse (horizontal drives) recieved
      if(instructval==6)
      {
       //inch reverse (horizontal drives) recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       recaxisspeed=xval;
       recinchdistance=yval;
       inchdistance=recinchdistance;

       int atleastonetaskdesp=0;
       int eithertopornorthengaged=0;
       int whethersouthengaged=0;
       
       //displayscreen(Xval);
       //displayscreen(Yval);
       for(int i=0;i<5;i++)
       {
          if(inchenable[i]==HIGH)
          {
            if(driveindexarray[i]!=0 && driveindexarray[i]!=2)
            {
              if(SendDataAwaitFeedback(driveindexarray[i],2,inchdistance,recaxisspeed,"",HIGH, HIGH))
              {
                jobcompletestatus[i]=HIGH;
                jobcompletestatusstring[i]='1';
                currtask=-1;
                atleastonetaskdesp=1;    
              }
              if(driveindexarray[i]==1)
              {
                whethersouthengaged=1;               
              }
            }
            else
            {
              eithertopornorthengaged=1;
            }
          }
       }
       if(eithertopornorthengaged==1 && whethersouthengaged==0)
       {
          if(SendDataAwaitFeedback(1,2,inchdistance,recaxisspeed,"",HIGH, HIGH))
          {
              jobcompletestatus[3]=HIGH;
              jobcompletestatusstring[3]='1';
              currtask=-1;
              atleastonetaskdesp=1;    
          }
       }
       if(atleastonetaskdesp==1)
       {
          displayscreen("stat*jobprog");
          currtask=-1;
       }
       else
       {
          displayscreen("stat*tskdspfail");
          currtask=0;
       }
       //displayscreen("stat*inchcomp");
      }
      //spray paint guns command recieved
      if(instructval==7)
      {
       //spray paint guns command recieved
       xval=Xval.toInt();

       int atleastonetaskdesp=0;

       
       for(int i=0;i<5;i++)
       {
          if(inchenable[i]==HIGH)
          {
            if(SendDataAwaitFeedback(driveindexarray[i],6,xval,0,"",HIGH, HIGH))
            {
              jobcompletestatus[i]=HIGH;
              jobcompletestatusstring[i]='1';
              currtask=-1;
              atleastonetaskdesp=1;    
            }
          }
       }
       if(atleastonetaskdesp==1)
       {
        displayscreen("stat*jobprog");
        currtask=-1;
       }
       else
       {
         displayscreen("stat*tskdspfail");
         currtask=0;
       }
       //displayscreen("stat*inchspraycomp");
      }
      //change enabling of inching
      if(instructval==8)
      {
       //change enabling of inching
       displayscreen("stat*inchchange");
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       //displayscreen(Xval);
       //displayscreen(Yval);
       if(xval==6)
       {
          if(yval==1)
          {
            allotinchenablearray(HIGH);
          }
          else if(yval==0)
          {
            allotinchenablearray(LOW);
          }
       }
       else if(xval==7)
       {
          if(yval==1)
          {
            for(int i=0;i<5;i++)
            {
              if(Zval[i]=='1')
              {
                inchenable[i]=HIGH;
                inchenablestring[i]='1';
              }
              else
              {
                inchenable[i]=LOW;
                inchenablestring[i]='0';             
              }
            }
          }
       }
       else if(xval<=5 && xval>=1)
       {
          if(yval==1)
          {
            inchenable[xval-1]=HIGH;
            inchenablestring[xval-1]='1';
          }
          else if(yval==0)
          {
            inchenable[xval-1]=LOW;
            inchenablestring[xval-1]='0';
          }
       }
       //displayscreen(inchenablestring);
       Serial.print("inchenablestringset:");
       Serial.println(inchenablestring);
       currtask=0;
      }
      //change enabling of pattern drive
      if(instructval==9)
      {
       //change enabling of pattern drive
       displayscreen("stat*engchange");
       xval=Xval.toInt();
       yval=Yval.toInt();
       
       //displayscreen(Xval);
       //displayscreen(Yval);
       if(xval==6)
       {
          if(yval==1)
          {
            allotengageenablearray(HIGH);
          }
          else if(yval==0)
          {
            allotengageenablearray(LOW);
          }
       }
       else if(xval<=5 && xval>=1)
       {
          if(yval==1)
          {
            engageenable[xval-1]=HIGH;
            engageenablestring[xval-1]='1';
          }
          else if(yval==0)
          {
            engageenable[xval-1]=LOW;
            engageenablestring[xval-1]='0';
          }
       }
       displayscreen(engageenablestring);
       currtask=0;
      }
      //toggle limits command received
      if(instructval==10)
      {
       //toggle limits command received
       displayscreen("stat*togglinglimits");
       xval=Xval.toInt();
       for(int i=0;i<5;i++)
       {
            if(inchenable[i]==HIGH)
            {
                SendDataAwaitFeedback(driveindexarray[i],8,xval,0,"",LOW, LOW);
            }
       }
       currtask=0;
      }
      //stop command recieved
      if(instructval==11)
      {
       //stop command recieved
       int atleastonetaskdesp=0;
       for(int i=0;i<5;i++)
       {
            if(inchenable[i]==HIGH || jobcompletestatus[i]==HIGH)
            {
                SendDataAwaitFeedback(driveindexarray[i],7,0,0,"",HIGH, HIGH);
                jobcompletestatus[i]=HIGH;
                jobcompletestatusstring[i]='1';
                atleastonetaskdesp=1;
                currtask=-1;  
            }
       }
       if(atleastonetaskdesp==1)
       {
         displayscreen("stat*jobprog");
         currtask=-1;
       }
       else
       {
          displayscreen("stat*tskdspfail");
          currtask=0;
       }
       //checkresetposition();
      }
      //vertical motion and paint command recieved
      if(instructval==12)
      {
       //vertical motion and paint command recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       zval=Zval.toInt();
       //wval=Wval.toInt();
       
       
       recaxisspeed=xval;
       recinchdistance=yval;
       inchdistance=recinchdistance;

       //int diffinwidthfortop=(zval-wval)/2;
       String painttoggleinstfortop="";
       painttoggleinstfortop=Wval;
       //painttoggleinstfortop+=String(diffinwidthfortop)+"#"+String(zval-diffinwidthfortop);
       //Serial.println(painttoggleinstfortop);
       int atleastonetaskdesp=0;
       for(int i=0;i<5;i++)
       {
            if(inchenable[i]==HIGH)
            {
                if(driveindexarray[i]==2)
                {
                  if(SendDataAwaitFeedback(driveindexarray[i],5,zval,recaxisspeed,painttoggleinstfortop,HIGH, HIGH))
                  {
                    jobcompletestatus[i]=HIGH;
                    jobcompletestatusstring[i]='1';
                    atleastonetaskdesp=1;
                    currtask=-1;                    
                  }
                }
                else
                {
                  if(SendDataAwaitFeedback(driveindexarray[i],5,inchdistance,recaxisspeed,Vval,HIGH, HIGH))
                  {
                    jobcompletestatus[i]=HIGH;
                    jobcompletestatusstring[i]='1';
                    atleastonetaskdesp=1;
                    currtask=-1;                    
                  }
                }
             }
       }
       if(atleastonetaskdesp==1)
       {
         displayscreen("stat*jobprog");
         currtask=-1;
       }
       else
       {
          displayscreen("stat*tskdspfail");
          currtask=0;
       }
       //checkresetposition();
      Serial.print("jobpendingatcmd:");
      Serial.println(jobcompletestatusstring);
      break;
      }
      if(instructval==13)
      {
       //vertical motion and paint command recieved
       xval=Xval.toInt();
       yval=Yval.toInt();
       zval=Zval.toInt();
       wval=Wval.toInt();
       
       recaxisspeed=xval;
       recinchdistance=yval;
       inchdistance=recinchdistance;

       int diffinwidthfortop=(zval-wval)/2;
       String painttoggleinstfortop="";
       painttoggleinstfortop+=String(diffinwidthfortop)+"#"+String(zval-diffinwidthfortop);
       //Serial.println(painttoggleinstfortop);
       int atleastonetaskdesp=0;
       for(int i=0;i<5;i++)
       {
            if(inchenable[i]==HIGH)
            {
                if(driveindexarray[i]==2)
                {
                  if(SendDataAwaitFeedback(driveindexarray[i],11,zval,recaxisspeed,painttoggleinstfortop,HIGH, HIGH))
                  {
                    jobcompletestatus[i]=HIGH;
                    jobcompletestatusstring[i]='1';
                    atleastonetaskdesp=1;
                    currtask=-1;                    
                  }
                }
                else
                {
                  if(SendDataAwaitFeedback(driveindexarray[i],11,inchdistance,recaxisspeed,Vval,HIGH, HIGH))
                  {
                    jobcompletestatus[i]=HIGH;
                    jobcompletestatusstring[i]='1';
                    atleastonetaskdesp=1;
                    currtask=-1;                    
                  }
                }
             }
       }
       if(atleastonetaskdesp==1)
       {
         displayscreen("stat*jobprog");
         currtask=-1;
       }
       else
       {
          displayscreen("stat*tskdspfail");
          currtask=0;
       }
       //checkresetposition();
       Serial.print("jobpendingatcmd:");
       Serial.println(jobcompletestatusstring);
       break;
      }
      Serial.print("jobpendingatcmd:");
      Serial.println(jobcompletestatusstring);
      break;  
    }
    // vertically moving and painting from bottom accomodating curve with 50% speed
//    case 11:
//    {
//      if(digitalRead(vertical_limit_extreme_reference_start_pin)==LOW)
//      {
//        displayscreen("V&P from bottom");
//        vertdistancetravelpaintcurve(150, 50, 1);
//        vertdistancetravelpaintcurve(2050, 100, 1);
//        plunger_disengage();
//      }
//      else if(digitalRead(vertical_limit_extreme_reference_end_pin)==LOW)
//      {
//        displayscreen("V&P from top");
//        vertdistancetravelpaintcurve(2050, 100, 2);
//        vertdistancetravelpaintcurve(150, 50, 2);
//        plunger_disengage();
//      }
//      else if(digitalRead(vertical_limit_extreme_reference_end_pin)==HIGH && digitalRead(vertical_limit_extreme_reference_start_pin)==HIGH)
//      {
//        displayscreen("Not in limits");
//        displayscreen("Resetting vertically");
//        vertdistancetravel(2500,incomingMessage.speedpercentvalue,2);
//        vertdistancetravelpaintcurve(150, 50, 1);
//        vertdistancetravelpaintcurve(2050, 100, 1);
//        plunger_disengage();
//      }
//    }
  }
}
void declarerelayoutputpins()
{
  pinMode(horizontal_relay_direction_pin,OUTPUT);
  pinMode(vertical_relay_direction_pin,OUTPUT);
  pinMode(drive_engage_relay_pin,OUTPUT);
  pinMode(plunger_engage_relay_pin,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
void declaredriveoutputpins()
{
  pinMode(horizontal_drive_speed_control_pin,OUTPUT);
  pinMode(horizontal_drive_speed_reference_pin,OUTPUT);
  pinMode(vertical_drive_speed_control_pin,OUTPUT);
  pinMode(vertical_drive_speed_reference_pin,OUTPUT);

  ledcSetup(pwmChannel2, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);
  
  ledcAttachPin(vertical_drive_speed_control_pin, pwmChannel2);
  ledcAttachPin(horizontal_drive_speed_control_pin, pwmChannel1);

  ledcWrite(pwmChannel1, 0);
  digitalWrite(horizontal_drive_speed_reference_pin,LOW);

  ledcWrite(pwmChannel2, 0);
  digitalWrite(vertical_drive_speed_reference_pin,LOW);

  whetherstopsignalreceived=LOW;
}
void declareinputpins()
{
  pinMode(horizontal_limit_extreme_reference_start_pin,INPUT);
  pinMode(horizontal_limit_extreme_reference_end_pin,INPUT);
  pinMode(horizontal_limit_coach_reference_start_pin,INPUT);
  pinMode(horizontal_limit_coach_reference_end_pin,INPUT);
  pinMode(vertical_limit_extreme_reference_start_pin,INPUT);
  pinMode(vertical_limit_extreme_reference_end_pin,INPUT);
  pinMode(vertical_limit_coach_reference_start_pin,INPUT);
  pinMode(vertical_limit_coach_reference_end_pin,INPUT);
  pinMode(spare_input_pin1,INPUT);
  pinMode(spare_input_pin2,INPUT);
  pinMode(spare_input_pin3,INPUT);
}
void declareinputpinspulldown()
{
  pinMode(horizontal_limit_extreme_reference_start_pin,INPUT_PULLDOWN);
  pinMode(horizontal_limit_extreme_reference_end_pin,INPUT_PULLDOWN);
  pinMode(horizontal_limit_coach_reference_start_pin,INPUT_PULLDOWN);
  pinMode(horizontal_limit_coach_reference_end_pin,INPUT_PULLDOWN);
  pinMode(vertical_limit_extreme_reference_start_pin,INPUT_PULLDOWN);
  pinMode(vertical_limit_extreme_reference_end_pin,INPUT_PULLDOWN);
  pinMode(vertical_limit_coach_reference_start_pin,INPUT_PULLDOWN);
  pinMode(vertical_limit_coach_reference_end_pin,INPUT_PULLDOWN);
  pinMode(spare_input_pin1,INPUT_PULLDOWN);
  pinMode(spare_input_pin2,INPUT_PULLDOWN);
  pinMode(spare_input_pin3,INPUT_PULLDOWN);
}
void declareinputpinspullup()
{
  pinMode(horizontal_limit_extreme_reference_start_pin,INPUT_PULLUP);
  pinMode(horizontal_limit_extreme_reference_end_pin,INPUT_PULLUP);
  pinMode(horizontal_limit_coach_reference_start_pin,INPUT_PULLUP);
  pinMode(horizontal_limit_coach_reference_end_pin,INPUT_PULLUP);
  pinMode(vertical_limit_extreme_reference_start_pin,INPUT_PULLUP);
  pinMode(vertical_limit_extreme_reference_end_pin,INPUT_PULLUP);
  pinMode(vertical_limit_coach_reference_start_pin,INPUT_PULLUP);
  pinMode(vertical_limit_coach_reference_end_pin,INPUT_PULLUP);
  pinMode(spare_input_pin1,INPUT_PULLUP);
  pinMode(spare_input_pin2,INPUT_PULLUP);
  pinMode(spare_input_pin3,INPUT_PULLUP);
}
void scaninputpinsforhigh()
{
  if(digitalRead(horizontal_limit_extreme_reference_start_pin)==HIGH)
  {
    displayscreen("Lim Horiz ext stt HIH");
    //Serial.println("horizontal_limit_extreme_reference_start_pin is HIGH");
  }
  if(digitalRead(horizontal_limit_extreme_reference_end_pin)==HIGH)
  {
    displayscreen("Lim Horiz ext end HIH");
    //Serial.println("horizontal_limit_extreme_reference_end_pin is HIGH");
  }
  /*
  if(digitalRead(horizontal_limit_coach_reference_start_pin)==HIGH)
  {
    displayscreen("Lim Horiz ref stt HIH");
    //Serial.println("horizontal_limit_coach_reference_start_pin is HIGH");
  }
  if(digitalRead(horizontal_limit_coach_reference_end_pin)==HIGH)
  {
    displayscreen("Lim Horiz ref end HIH");
    //Serial.println("horizontal_limit_coach_reference_end_pin is HIGH");
  }
  */
  if(digitalRead(vertical_limit_extreme_reference_start_pin)==HIGH)
  {
    displayscreen("Lim Vert ext stt HIH");
    //Serial.println("vertical_limit_extreme_reference_start_pin is HIGH");
  }
  if(digitalRead(vertical_limit_extreme_reference_end_pin)==HIGH)
  {
    displayscreen("Lim Vert ext end HIH");
    //Serial.println("vertical_limit_extreme_reference_end_pin is HIGH");
  }
  /*
  if(digitalRead(vertical_limit_coach_reference_start_pin)==HIGH)
  {
    displayscreen("Lim Vert ref stt HIH");
    //Serial.println("vertical_limit_coach_reference_start_pin is HIGH");
  }
  if(digitalRead(vertical_limit_coach_reference_end_pin)==HIGH)
  {
    displayscreen("Lim Vert ref end HIH");
    //Serial.println("vertical_limit_coach_reference_end_pin is HIGH");
  }
  if(digitalRead(spare_input_pin1)==HIGH)
  {
    displayscreen("Spare input pin1 HIH");
    //Serial.println("spare_input_pin1 is HIGH");
  }
  if(digitalRead(spare_input_pin2)==HIGH)
  {
    displayscreen("Spare input pin2 HIH");
    //Serial.println("spare_input_pin2 is HIGH");
  }
  if(digitalRead(spare_input_pin3)==HIGH)
  {
    displayscreen("Spare input pin3 HIH");
    //Serial.println("spare_input_pin3 is HIGH");
  }
  */
}
void scaninputpinsforlow()
{
  if(digitalRead(horizontal_limit_extreme_reference_start_pin)==LOW)
  {
    displayscreen("Lim Horiz ext stt LOW");
    //Serial.println("horizontal_limit_extreme_reference_start_pin is LOW");
  }
  if(digitalRead(horizontal_limit_extreme_reference_end_pin)==LOW)
  {
    displayscreen("Lim Horiz ext end LOW");
    //Serial.println("horizontal_limit_extreme_reference_end_pin is LOW");
  }
  /*
  if(digitalRead(horizontal_limit_coach_reference_start_pin)==LOW)
  {
    displayscreen("Lim Horiz ref stt LOW");
    //Serial.println("horizontal_limit_coach_reference_start_pin is LOW");
  }
  if(digitalRead(horizontal_limit_coach_reference_end_pin)==LOW)
  {
    displayscreen("Lim Horiz ref end LOW");
    //Serial.println("horizontal_limit_coach_reference_end_pin is LOW");
  }
  */
  
  if(digitalRead(vertical_limit_extreme_reference_start_pin)==LOW)
  {
    displayscreen("Lim Vert ext stt LOW");
    //Serial.println("vertical_limit_extreme_reference_start_pin is LOW");
  }
  if(digitalRead(vertical_limit_extreme_reference_end_pin)==LOW)
  {
    displayscreen("Lim Vert ext end LOW");
    //Serial.println("vertical_limit_extreme_reference_end_pin is LOW");
  }
  /*
  if(digitalRead(vertical_limit_coach_reference_start_pin)==LOW)
  {
    displayscreen("Lim Vert ref stt LOW");
    //Serial.println("vertical_limit_coach_reference_start_pin is LOW");
  }
  if(digitalRead(vertical_limit_coach_reference_end_pin)==LOW)
  {
    displayscreen("Lim Vert ref end LOW");
    //Serial.println("vertical_limit_coach_reference_end_pin is LOW");
  }
  if(digitalRead(spare_input_pin1)==LOW)
  {
    displayscreen("spare input pin1 LOW");
    //Serial.println("spare_input_pin1 is LOW");
  }
  if(digitalRead(spare_input_pin2)==LOW)
  {
    displayscreen("spare input pin2 LOW");
    //Serial.println("spare_input_pin2 is LOW");
  }
  if(digitalRead(spare_input_pin3)==LOW)
  {
    displayscreen("spare input pin3 LOW");
    //Serial.println("spare_input_pin3 is LOW");
  }
  */
}
void horizontal_motor_forward()
{
  //displayscreen("Horiz Mot Forward");
  digitalWrite(horizontal_relay_direction_pin,HIGH);
}
void horizontal_motor_reverse()
{
  //displayscreen("Horiz Mot Reverse");
  digitalWrite(horizontal_relay_direction_pin,LOW);
}
void vertical_motor_forward()
{
  //displayscreen("Vert Mot Forward");
  digitalWrite(vertical_relay_direction_pin,HIGH);
}
void vertical_motor_reverse()
{
  //displayscreen("Vert Mot Reverse");
  digitalWrite(vertical_relay_direction_pin,LOW);
}
void drive_engage()
{
  displayscreen("Enaging drive");
  digitalWrite(drive_engage_relay_pin,HIGH);
}
void drive_disengage()
{
  displayscreen("Disengaging drive");
  digitalWrite(drive_engage_relay_pin,LOW);
}
void plunger_engage()
{
  displayscreen("Enaging plunger");
  digitalWrite(plunger_engage_relay_pin,HIGH);
}
void plunger_disengage()
{
  displayscreen("Disengaging plunger");
  digitalWrite(plunger_engage_relay_pin,LOW);
}
void horizontal_drive_speed_select(int speedinpercentage)
{
  int speedinanalog=(speedinpercentage*255/100);
  String displaystring;
  displaystring="speed horiz:";
  displaystring=displaystring+speedinpercentage;
  displaystring=displaystring+"% ";
  displaystring=displaystring+speedinanalog;
  //displayscreen(displaystring);
 
  if(speedinpercentage==0)
  {
    ledcWrite(pwmChannel1, 0);
    displayscreen("horiz drive stop");
  }
  else if(speedinpercentage==50)
  {
    ledcWrite(pwmChannel1, 138);
  }
  else if(speedinpercentage==75)
  {
    ledcWrite(pwmChannel1, 199);
  }  
  else
  {
    ledcWrite(pwmChannel1, speedinanalog);
  }
  digitalWrite(horizontal_drive_speed_reference_pin,LOW);
}
void vertical_drive_speed_select(int speedinpercentage)
{
  int speedinanalog=(speedinpercentage*255/100);
  /*
  String displaystring;
  displaystring="speed vert:";
  displaystring=displaystring+speedinpercentage;
  displaystring=displaystring+"% ";
  displaystring=displaystring+speedinanalog;
  displayscreen(displaystring);
  */
  
  if(speedinpercentage==0)
  {
    ledcWrite(pwmChannel2, 0);
    displayscreen("vert drive stop");
  }
  else if(speedinpercentage==50)
  {
    ledcWrite(pwmChannel2, 138);
  }
  else if(speedinpercentage==75)
  {
    ledcWrite(pwmChannel2, 199);
  }
  else
  {
    ledcWrite(pwmChannel2, speedinanalog);
  }
  digitalWrite(vertical_drive_speed_reference_pin,LOW);
}
void testrelayboard()
{
  displayscreen("Starting relay test");
  delay(testtimeinterval);
  horizontal_motor_forward();
  delay(testtimeinterval);
  horizontal_motor_reverse();
  delay(testtimeinterval);
  vertical_motor_forward();
  delay(testtimeinterval);
  vertical_motor_reverse();
  delay(testtimeinterval);
  drive_engage();
  delay(testtimeinterval);
  drive_disengage();
  delay(testtimeinterval);
  plunger_engage();
  delay(testtimeinterval);
  plunger_disengage();
  delay(testtimeinterval);
}
void testdrivespeedcontrol()
{
  displayscreen("Starting speed test");
  delay(1000);
  horizontal_drive_speed_select(100);
  delay(5000);
  vertical_drive_speed_select(100);
  delay(5000);
  horizontal_drive_speed_select(50);
  delay(5000);
  vertical_drive_speed_select(50);
}
void testhorizdrive()
{
  displayscreen("horiz drive test");
  delay(1000);
  horizontal_drive_speed_select(100);
  delay(5000);
  horizontal_drive_speed_select(50);
  delay(5000);
  horizontal_drive_speed_select(100);
  delay(5000);
  horizontal_motor_forward();
  delay(5000);
  horizontal_motor_reverse();
  delay(5000);
  horizontal_drive_speed_select(0);
}
void horizdistancetravel(int distanceinmm, int speedinpercentage, int traveldirection)
{
  float timetoengagedrive=(distanceinmm/(xfullspeed*speedinpercentage/100))*1000.000;
  String displaystring;
  int limit_pin_considered;
  
  reqtime=round(timetoengagedrive);
  displaystring="Horz ";
  displaystring=displaystring+speedinpercentage;
  displaystring=displaystring+"% ";
  displaystring=displaystring+reqtime;
  displaystring=displaystring+"ms ";
  
  if(traveldirection==1) //move forward
  {
    horizontal_motor_forward();
    displaystring=displaystring+" F";
    limit_pin_considered=horizontal_limit_extreme_reference_end_pin;
  }
  else //move reverse
  {
    horizontal_motor_reverse();
    displaystring=displaystring+" R";
    limit_pin_considered=horizontal_limit_extreme_reference_start_pin;
  }
  displayscreen(displaystring);
  horizontal_drive_speed_select(speedinpercentage);
  starttime=millis();
  currtime=millis();
  if(digitalRead(limit_pin_considered)==LOW)
  {
    displayscreen("limit ald engaged");
  }
  for(;(currtime-starttime)<reqtime && whetherstopsignalreceived==LOW;)
  {
    currtime=millis();
    if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
    {
      if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
      {
        if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
        {
          displayscreen("limit engaged");
          break;
        }
      }
    }
  }
  horizontal_drive_speed_select(0);
 
}
void vertdistancetravel(int distanceinmm, int speedinpercentage, int traveldirection)
{
  float timetoengagedrive=(distanceinmm/(yfullspeed*speedinpercentage/100))*1000.000;
  String displaystring;
  int limit_pin_considered;
  
  reqtime=round(timetoengagedrive);
  displaystring="Vert ";
  displaystring=displaystring+speedinpercentage;
  displaystring=displaystring+"% ";
  displaystring=displaystring+reqtime;
  displaystring=displaystring+"ms ";
  if(traveldirection==1) //move forward
  {
    vertical_motor_forward();
    displaystring=displaystring+" F";
    limit_pin_considered=vertical_limit_extreme_reference_end_pin;
  }
  else //move reverse
  {
    vertical_motor_reverse();
    displaystring=displaystring+" R";
    limit_pin_considered=vertical_limit_extreme_reference_start_pin;
  }
  displayscreen(displaystring);
  starttime=millis();
  currtime=millis();
  vertical_drive_speed_select(speedinpercentage);
  if(digitalRead(limit_pin_considered)==LOW)
  {
      displayscreen("limit ald engaged");
  }
  for(;(currtime-starttime)<reqtime && whetherstopsignalreceived==LOW ;)
  {
    currtime=millis();
    if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
    {
      if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
      {
        if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
        {
          displayscreen("limit engaged");
          break;
        }
      }
    }
  }
  vertical_drive_speed_select(0);
}
void vertdistancetravelandpaintcurveneg(int distanceinmm, int speedinpercentage, int traveldirection, String toggledistancetext)
{
  float timetoengagedriveatfullspeed=(distanceinmm-150)/(yfullspeed*speedinpercentage/100)*1000.000;
  float timetoengagedriveathalfspeed=(150/(yfullspeed*30/100))*1000.000; // changed to 30 from 50
  float timetoengagedrive=timetoengagedriveatfullspeed+timetoengagedriveathalfspeed;
  float timetotogglespeed;
  boolean whetherpaintgunengaged=LOW;
  unsigned long toggletime[10];
  unsigned long toggletimetemp[10];
  int breakcount=0;
  String partstring="";
  int limit_pin_considered;

  unsigned long spraystarttime;
  boolean whetherloadreduced=LOW;

  for(int i=0;i<toggledistancetext.length();i++)
  {
    if(toggledistancetext[i]=='#')
    {
        if(partstring.toInt()<=150)
        {
          toggletimetemp[breakcount]=(partstring.toInt()/(yfullspeed*30/100))*1000.000; // changed to 30 from 50
        }
        else if(partstring.toInt()>150)
        {
          toggletimetemp[breakcount]=(partstring.toInt()-150)/(yfullspeed*speedinpercentage/100)*1000.000 + 150/(yfullspeed*30/100)*1000.000; // changed to 30 from 50
        }
        //toggletime[breakcount]=(partstring.toInt()/(yfullspeed*speedinpercentage/100))*1000.000;        
      breakcount++;
      partstring="";
    }
    else
    {
      partstring+=toggledistancetext[i];
    }
  }

    if(partstring.toInt()<=150)
    {
          toggletimetemp[breakcount]=(partstring.toInt()/(yfullspeed*30/100))*1000.000; // changed to 30 from 50
    }
    else if(partstring.toInt()>150)
    {
          toggletimetemp[breakcount]=(partstring.toInt()-150)/(yfullspeed*speedinpercentage/100)*1000.000 + 150/(yfullspeed*30/100)*1000.000; // changed to 30 from 50
    }
    //toggletime[breakcount]=(partstring.toInt()/(yfullspeed*speedinpercentage/100))*1000.000;
    breakcount++;  

 
  
  if(traveldirection!=1)
  {
    for(int i=breakcount-1;i>=0;i--)
    {
      toggletime[breakcount-1-i]=timetoengagedrive-toggletimetemp[i];
    }
    timetotogglespeed=(distanceinmm-150)/(yfullspeed*speedinpercentage/100)*1000.000;   
  }
  else
  {
    for(int i=0;i<breakcount;i++)
    {
      toggletime[i]=toggletimetemp[i];
    }
    timetotogglespeed=(150)/(yfullspeed*30/100)*1000.000;     // changed to 30 from 50
  }
  /*
  Serial.print("breakcount:");
  Serial.println(breakcount);

  Serial.println("toggle timecounts");
  for(int i=0;i<breakcount;i++)
  {
    Serial.println(toggletime[i]);
  }
  */
  
  String displaystring;
  reqtime=round(timetoengagedrive);
  displaystring="Vert ";
  displaystring=displaystring+speedinpercentage;
  displaystring=displaystring+"% ";
  displaystring=displaystring+reqtime;
  displaystring=displaystring+"ms ";
  if(traveldirection==1) //move forward
  {
    vertical_motor_forward();
    displaystring=displaystring+" F";
    limit_pin_considered=vertical_limit_extreme_reference_end_pin;
  }
  else //move reverse
  {
    vertical_motor_reverse();
    displaystring=displaystring+" R";
    limit_pin_considered=vertical_limit_extreme_reference_start_pin;  
  }
  displayscreen(displaystring);
  if(traveldirection==1)
  {
    vertical_drive_speed_select(30); // changed to 30 from 50
  }
  else
  {
    vertical_drive_speed_select(speedinpercentage);
  }
  starttime=millis();
  currtime=millis();
  if(digitalRead(limit_pin_considered)==LOW)
  {
      displayscreen("limit ald engaged");
  }
  for(int i=0;(currtime-starttime)<reqtime && whetherstopsignalreceived==LOW ;)
  {
    currtime=millis();
    if((currtime-starttime)>=timetotogglespeed)
    {
      if(traveldirection!=1)
      {
        vertical_drive_speed_select(30); // changed to 30 from 50
      }
      else
      {
        vertical_drive_speed_select(speedinpercentage);
      }
    }
    
    if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
    {
      if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
      {
        if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
        {
          displayscreen("limit engaged");
          break;
        }
      }
    }
    if(whetherloadreduced==LOW && whetherpaintgunengaged==HIGH && (currtime-spraystarttime)>1000)
    {
       //Serial.println("here");
       digitalWrite(drive_engage_relay_pin,HIGH);
       whetherloadreduced=HIGH;
    }
    if((currtime-starttime)>=toggletime[i] && i<breakcount)
    {
      if(!whetherpaintgunengaged)
      {
        plunger_engage();
        spraystarttime=millis();
        //Serial.println(toggletime[i]);
        //Serial.println(i);
        //digitalWrite(LED_BUILTIN, LOW);  
        whetherpaintgunengaged=HIGH;
        whetherloadreduced=LOW;
      }
      else
      {
        plunger_disengage();
        digitalWrite(drive_engage_relay_pin,LOW);
        //Serial.println(toggletime[i]);
        //digitalWrite(LED_BUILTIN, HIGH);  
        whetherpaintgunengaged=LOW;
        whetherloadreduced=LOW;
        //Serial.println(i);
      }
      i++;
    }
  }
  vertical_drive_speed_select(0);
  plunger_disengage();
  digitalWrite(drive_engage_relay_pin,LOW);
}
void vertdistancetravelandpaint(int distanceinmm, int speedinpercentage, int traveldirection, String toggledistancetext)
{
  float timetoengagedrive=(distanceinmm/(yfullspeed*speedinpercentage/100))*1000.000;
  boolean whetherpaintgunengaged=LOW;
  unsigned long toggletime[10];
  unsigned long toggletimetemp[10];
  int breakcount=0;
  String partstring="";
  int limit_pin_considered;

  unsigned long spraystarttime;
  boolean whetherloadreduced=LOW;

  for(int i=0;i<toggledistancetext.length();i++)
  {
    if(toggledistancetext[i]=='#')
    {
      if(traveldirection==1) //move up from bottom
      {
        toggletime[breakcount]=(partstring.toInt()/(yfullspeed*speedinpercentage/100))*1000.000;        
      }
      else // moving down from top
      {
        toggletimetemp[breakcount]=((distanceinmm-partstring.toInt())/(yfullspeed*speedinpercentage/100))*1000.000;
      }
      breakcount++;
      partstring="";
    }
    else
    {
      partstring+=toggledistancetext[i];
    }
  }
  if(traveldirection==1) //move up from bottom
  {
    toggletime[breakcount]=(partstring.toInt()/(yfullspeed*speedinpercentage/100))*1000.000;
    breakcount++;  
  }
  else // moving down from top
  {
    toggletimetemp[breakcount]=((distanceinmm-partstring.toInt())/(yfullspeed*speedinpercentage/100))*1000.000;
    breakcount++;
  }
  
  if(traveldirection!=1)
  {
    for(int i=breakcount-1;i>=0;i--)
    {
      toggletime[breakcount-1-i]=toggletimetemp[i];
    }   
  }

  /*
  Serial.print("breakcount:");
  Serial.println(breakcount);

  Serial.println("toggle timecounts");
  for(int i=0;i<breakcount;i++)
  {
    Serial.println(toggletime[i]);
  }
  */
  
  String displaystring;
  reqtime=round(timetoengagedrive);
  displaystring="Vert ";
  displaystring=displaystring+speedinpercentage;
  displaystring=displaystring+"% ";
  displaystring=displaystring+reqtime;
  displaystring=displaystring+"ms ";
  if(traveldirection==1) //move forward
  {
    vertical_motor_forward();
    displaystring=displaystring+" F";
    limit_pin_considered=vertical_limit_extreme_reference_end_pin;
  }
  else //move reverse
  {
    vertical_motor_reverse();
    displaystring=displaystring+" R";
    limit_pin_considered=vertical_limit_extreme_reference_start_pin;  
  }
  displayscreen(displaystring);
  vertical_drive_speed_select(speedinpercentage);
  starttime=millis();
  currtime=millis();
  if(digitalRead(limit_pin_considered)==LOW)
  {
      displayscreen("limit ald engaged");
  }
  for(int i=0;(currtime-starttime)<reqtime && whetherstopsignalreceived==LOW ;)
  {
    currtime=millis();
    if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
    {
      if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
      {
        if(digitalRead(limit_pin_considered)==LOW  && whetherlimitsbypassed==LOW)
        {
          displayscreen("limit engaged");
          break;
        }
      }
    }
    if(whetherloadreduced==LOW && whetherpaintgunengaged==HIGH && (currtime-spraystarttime)>1000)
    {
       //Serial.println("here");
       digitalWrite(drive_engage_relay_pin,HIGH);
       whetherloadreduced=HIGH;
    }
    if((currtime-starttime)>=toggletime[i] && i<breakcount)
    {
      if(!whetherpaintgunengaged)
      {
        plunger_engage();
        spraystarttime=millis();
        //Serial.println(toggletime[i]);
        //Serial.println(i);
        //digitalWrite(LED_BUILTIN, LOW);  
        whetherpaintgunengaged=HIGH;
        whetherloadreduced=LOW;
      }
      else
      {
        plunger_disengage();
        digitalWrite(drive_engage_relay_pin,LOW);
        //Serial.println(toggletime[i]);
        //digitalWrite(LED_BUILTIN, HIGH);  
        whetherpaintgunengaged=LOW;
        whetherloadreduced=LOW;
        //Serial.println(i);
      }
      i++;
    }
  }
  vertical_drive_speed_select(0);
  plunger_disengage();
  digitalWrite(drive_engage_relay_pin,LOW);
}
void puttextonscreen(String newtext)
{
  display.clearDisplay();
  int i=0;
  for(i=0;i<linesloaded;i++)
  {
    if(line[i+1]!="")
    {
      line[i]=line[i+1];
    }
  }
  line[i]=newtext;
  for(i=0;i<=linesloaded;i++)
  {
    display.setCursor(0,i*10);
    display.println(line[i]);
    display.display();
  }
  if(linesloaded<5)
  {
    linesloaded++;
  }
}
void refresholeddisplay()
{
  linesloaded=0;
  for(int i=0;i<5;i++)
  {
    line[i]="";
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
}
void setupoleddisplay()
{
  linesloaded=0;
  for(int i=0;i<5;i++)
  {
    line[i]="";
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
}
void displayscreen(String text)
{
  if(enableoled)
  {
    puttextonscreen(text);
  }
  if(enableserialmonitor)
  {
    Serial.println(text);
  }
}
void setupespnow()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    
    displayscreen("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  
  awaiting_feedback=LOW;
  feedback_recieved=LOW;

  findmysenderid();
}
void registerpeer(int recipientid)
{
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress[recipientid], 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    //displayscreen("Failed to add peer");
    return;
  }
}
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
    if(await_deliverystatus)
    {
      deliverystatus=HIGH;
    }
  }
  else
  {
    if(await_deliverystatus)
    {
      deliverystatus=LOW;
    }
  }
  //displayscreen(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if(enablerefreshdisplay)
  {
     refresholeddisplay();  
  }
  //displayscreen("msg rec");
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  //displayscreen("after memcpy");
  //Serial.println(awaiting_feedback);
  if(incomingMessage.messagetype==0)
  {
    if(awaiting_feedback) //feedback message
    {
      //displayscreen("0 feed recd");
      feedback_recieved=HIGH;
      awaiting_feedback=LOW;
    }
  }
  else
  {
      currtask=incomingMessage.messagetype;
      if(currtask==7)
      {
        //Serial.println("stop cmd rec");
        whetherstopsignalreceived=HIGH;
      }
      if(currtask==9)
      {
        /*
        Serial.println("comp feed rec");
        Serial.print("from:");
        Serial.println(incomingMessage.senderid);
        Serial.print("rec tsk type:");
        Serial.println(currtask);
        */
        whethercompletefeedbackreceived=HIGH;
        feedbackrecstatus[incomingMessage.travelvalue]=HIGH;
      }
 
  }

}
boolean SendDataAwaitFeedback(int recipientid, int messagetype, int travelvalue, int speedpercentvalue, String sendmsg, boolean awaitdelfeedback, boolean awaitrecfeedback)
{

  registerpeer(recipientid);
  // Set values to send
  
  // Length (with one extra character for the null terminator)
  int str_len = sendmsg.length() + 1; 
  // Prepare the character array (the buffer) 
  char char_array[str_len];
  // Copy it over 
  sendmsg.toCharArray(char_array, str_len);
  strcpy(sentMessage.messagetxt,char_array);
  
  sentMessage.senderid=mysenderid;
  sentMessage.messagetype=messagetype;
  sentMessage.travelvalue=travelvalue;
  sentMessage.speedpercentvalue=speedpercentvalue;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress[recipientid], (uint8_t *) &sentMessage, sizeof(sentMessage));
  
  deliverystatus=LOW;
  await_deliverystatus=awaitdelfeedback;
  awaiting_feedback=awaitrecfeedback;
  
  if (result == ESP_OK) 
  {
    //Serial.println("Dispatch Success");
    if(await_deliverystatus)
    {
      //Serial.println("awaiting delivery status");
      
      begintime=millis();
      for(;deliverystatus==LOW;)
      {
        endtime=millis();
        if((endtime-begintime)>deltimeout)
        {
          displayscreen("Send delivery timeout");
          return LOW;
        }
      }
      //Serial.print("delivered status recieved; feedback time:");
      //Serial.println(begintime);
      //Serial.println(endtime);
      

      if(awaitrecfeedback==HIGH)
      {
        begintime=millis();
        for(;feedback_recieved==LOW;)
        {
          endtime=millis();
          
          if((endtime-begintime)>rectimeout)
          {
            displayscreen("Rec feedback timeout");
            return LOW;
          }
          
        }
        //Serial.print("feedback time: ");
        //Serial.println(endtime-begintime);
        return HIGH;
      }
      else
      {
        //Serial.println("Dispatch Success");
        return HIGH;
      }
    }
    else
    {
      //Serial.println("Dispatch Success");
      return HIGH;
    }
  }
  else 
  {
    displayscreen("Dispatch Failure");
    return LOW;
  }
}
void updateDisplay()
{
  //display.clearDisplay();
  // Display Readings in Serial Monitor

  /*
  String displaystring;
  displaystring="IM FROM:";
  displaystring=displaystring+incomingMessage.senderid;
  displaystring=displaystring+" IC:";
  displaystring=displaystring+incomingMessage.messagetype;
  displayscreen(displaystring);

  displaystring="TV:";
  displaystring=displaystring+incomingMessage.travelvalue;
  displayscreen(displaystring);

  displaystring="SP%:";
  displaystring=displaystring+incomingMessage.speedpercentvalue;
  displayscreen(displaystring);

  displaystring="SS:";
  displaystring=displaystring+incomingMessage.messagetxt;
  displayscreen(displaystring);
  */
}
void sendtestdata()
{
  
  displayscreen("Starting transmission");
  //delay(5000);
  //vertdistancetravelandpaint(500,50,1,"250*400");  
  if(SendDataAwaitFeedback(0,1,2200,50,"",HIGH, HIGH))
  {
    displayscreen("Send 1 Success");

    if(SendDataAwaitFeedback(1,1,2200,50,"",HIGH, HIGH))
    {
      displayscreen("Send 2 Success");
    }
    else
    {
      displayscreen("Send 2 Failed");
      SendDataAwaitFeedback(0,1,0,0,"stop",HIGH, HIGH);
    }
  }
  else
  {
    displayscreen("Send 1 Failed");
  }
}
void findmysenderid()
{
  String currmacaddress=WiFi.macAddress();
  displayscreen(currmacaddress);
  
  int breakcount=0;
  int macaddressint[6];
  String partstring="";
  for(int i=0;i<currmacaddress.length();i++)
  {
    if(currmacaddress[i]==':')
    {
      int str_len = partstring.length() + 1; 
      char char_array[str_len];
      partstring.toCharArray(char_array, str_len);
      long ans=strtoul(char_array,NULL,16);
      macaddressint[breakcount]=ans;
      breakcount++;
      partstring="";
    }
    else
    {
      partstring+=currmacaddress[i];  
    }
  }
  int str_len = partstring.length() + 1; 
  char char_array[str_len];
  partstring.toCharArray(char_array, str_len);
  long ans=strtoul(char_array,NULL,16);
  macaddressint[5]=ans;

  int i;
  for(i=0;i<6;i++)
  {
    int check=0;
    for(int j=0;j<6;j++)
    {
      if(broadcastAddress[i][j]!=macaddressint[j])
      {
        check=1;
      }
    }
    if(check==0)
    {
      break;
    }
  }
  mysenderid=i;
  for(i=0;i<6;i++)
  {
    if(driveindexarray[i]==mysenderid)
    {
      mydriveindex=i;
      break;
    }
  }
  if(mysenderid==5 && mydriveindex==5)
  {
    whethermaster=1;
    enableoled=LOW;
    enableserialmonitor=HIGH;
    enablerefreshdisplay=HIGH;
  }
  else
  {
    whethermaster=0;
    enableoled=HIGH;
    enableserialmonitor=HIGH;
    enablerefreshdisplay=HIGH;
  }
}
void spray(int spraytimeinmillis)
{
      unsigned long spraystarttime=millis();
      unsigned long spraycurrtime=millis();
      plunger_engage();
      for(;(spraycurrtime-spraystarttime)<spraytimeinmillis && whetherstopsignalreceived==LOW;)
      {
          spraycurrtime=millis();
          if((spraycurrtime-spraystarttime)>1000)
          {
            digitalWrite(drive_engage_relay_pin,HIGH);           
          }
      }
      
      plunger_disengage();
      digitalWrite(drive_engage_relay_pin,LOW);
}
