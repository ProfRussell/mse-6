/*
 Mark Russell
 August 12, 2016
 This is a modified version of Barrett Anderies R/C sketch, which is based on the PS3BT.ino example sketch by Kristian Lauszus
 For more information visit his blog: http://blog.tkjelectronics.dk/ or 
 send him an e-mail:  kristianl@tkjelectronics.com
 */

/* Wiring Guide */
//Serial MP3 Player A     |  Arduino UNO R3//
//              RX        |   7
//              TX        |   8
//              VCC       |   5V
//              GND       |   GND

#include <SoftwareSerial.h>
#include <PS3BT.h>                                                    //Include the necessary libraries.
#include <Servo.h>
#include <SPI.h>

#define ARDUINO_RX 8//should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 7//connect to RX of the module
SoftwareSerial myMP3(ARDUINO_RX, ARDUINO_TX);

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd); 

Servo servo1;                                                         //Create instances of type Servo. servo1 is the steering servo and servo2 is the ESC.
Servo servo2;

static int8_t Send_buf[6] = {0} ; // Serial MP3 initialization
/************Command byte**************************/
/*basic commands*/
#define CMD_PLAY  0X01
#define CMD_PAUSE 0X02
#define CMD_NEXT_SONG 0X03
#define CMD_PREV_SONG 0X04
#define CMD_VOLUME_UP   0X05
#define CMD_VOLUME_DOWN 0X06
#define CMD_FORWARD 0X0A // >>
#define CMD_REWIND  0X0B // <<
#define CMD_STOP 0X0E
#define CMD_STOP_INJECT 0X0F//stop interruptting with a song, just stop the interlude

/*5 bytes commands*/
#define CMD_SEL_DEV 0X35
#define DEV_TF 0X01
#define CMD_IC_MODE 0X35
#define CMD_SLEEP   0X03
#define CMD_WAKE_UP 0X02
#define CMD_RESET   0X05

/*6 bytes commands*/  
#define CMD_PLAY_W_INDEX   0X41
#define CMD_PLAY_FILE_NAME 0X42
#define CMD_INJECT_W_INDEX 0X43

/*Special commands*/
#define CMD_SET_VOLUME 0X31
#define CMD_PLAY_W_VOL 0X31

#define CMD_SET_PLAY_MODE 0X33
#define ALL_CYCLE 0X00
#define SINGLE_CYCLE 0X01

#define CMD_PLAY_COMBINE 0X45//can play combination up to 15 songs

void sendCommand(int8_t command, int16_t dat ); // last of the serial MP3 initialization stuff2`

int playSounds = 0;                                                   //loop counter for sounds

void setup() {
  Serial.begin(115200);                                              
  if (Usb.Init() == -1) {                                            
    Serial.print(F("\r\nOSC did not start"));
    while(1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));              
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  servo1.attach(4);                                                  //Steering servo on digital pin 4
  servo2.attach(3);                                                  //ESC on sigital pin 3

  myMP3.begin(9600);                                                 //Set up the MP3 card
  delay(500);//Wait chip initialization is complete
  sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card  
  delay(200);//wait for 200ms

}
void loop() 
{
  Usb.Task();

  if(PS3.PS3Connected || PS3.PS3NavigationConnected) {
    
    servo1.write(map(PS3.getAnalogHat(LeftHatX), 0, 255, 0, 180));  //Steering (joystick min, joystick max, left min, right max)
    servo2.write(map(PS3.getAnalogHat(LeftHatY), 0, 255, 120, 60));   //Speed (joystick min, joystick max, speed max [can goto 180], speed min [can goto 0])
  }
  else 
   {
    servo1.write(90);
    servo2.write(90);
   }
    
    if(PS3.getButtonClick(PS)) {								// Stop & disco
	  servo1.write(90);
	  servo2.write(90);		
      PS3.disconnect();
   }
   
    if(PS3.getButtonClick(TRIANGLE)) {						// Kill switch
    servo1.write(90);
    servo2.write(90);		
    sendCommand(CMD_STOP, DEV_TF);						
   }   
   
    if(PS3.getButtonClick(CROSS)) {							// Music
     setVolume (0x21);
     playSounds = playSounds + 1;  // up track
     if (playSounds == 1) { cyclePlay(01); }
     if (playSounds == 2) { cyclePlay(02); }
     if (playSounds == 3) { cyclePlay(03); }
        { 
        cyclePlay(04); 
        playSounds = 0;
        }   
	
   }     
   
}

/* The Serial MP3 methods*/
void setVolume(int8_t vol)
{
  mp3_5bytes(CMD_SET_VOLUME, vol);
}
void playWithVolume(int16_t dat)
{
  mp3_6bytes(CMD_PLAY_W_VOL, dat);
}

/*cycle play with an index*/
void cyclePlay(int16_t index)
{
  mp3_6bytes(CMD_SET_PLAY_MODE,index);
}

void sendCommand(int8_t command, int16_t dat = 0)
{
  delay(20);
  if((command == CMD_PLAY_W_VOL)||(command == CMD_SET_PLAY_MODE)||(command == CMD_PLAY_COMBINE))
    return;
  else if(command < 0x10) 
  {
  mp3Basic(command);
  }
  else if(command < 0x40)
  { 
  mp3_5bytes(command, dat);
  }
  else if(command < 0x50)
  { 
  mp3_6bytes(command, dat);
  }
  else return;
 
}


void mp3Basic(int8_t command)
{
  Send_buf[0] = 0x7e; //starting byte
  Send_buf[1] = 0x02; //the number of bytes of the command without starting byte and ending byte
  Send_buf[2] = command; 
  Send_buf[3] = 0xef; //
  sendBytes(4);
}

void mp3_5bytes(int8_t command, uint8_t dat)
{
  Send_buf[0] = 0x7e; //starting byte
  Send_buf[1] = 0x03; //the number of bytes of the command without starting byte and ending byte
  Send_buf[2] = command; 
  Send_buf[3] = dat; //
  Send_buf[4] = 0xef; //
  sendBytes(5);
}
void mp3_6bytes(int8_t command, int16_t dat)
{
  Send_buf[0] = 0x7e; //starting byte
  Send_buf[1] = 0x04; //the number of bytes of the command without starting byte and ending byte
  Send_buf[2] = command; 
  Send_buf[3] = (int8_t)(dat >> 8);//datah
  Send_buf[4] = (int8_t)(dat); //datal
  Send_buf[5] = 0xef; //
  sendBytes(6);
}
void sendBytes(uint8_t nbytes)
{
  for(uint8_t i=0; i < nbytes; i++)//
  {
    myMP3.write(Send_buf[i]) ;
  }
}
