//#define DEBUG_COMMANDER

//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix Commander version - Try to emulate most of PS2, but PS2 has 16 buttons and Commander 
// has 10. so some things may not be there, others may be doubled up.
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
// Quick and Dirty description of controls... WIP
// In most cases I try to mention what button on the PS2 things coorespond to..
// On/OFF - Turning the commander 2 on and off (PS2 start button)
// R1 - options (Change walk gait, Change Leg in Single Leg, Change GP sequence) (Select on PS2)
// R2 - Toggle walk method...  Run Sequence in GP mode
// R3 - Walk method (Not done yet) - (PS2 R3)
// L4 - Ballance mode on and off
// L5 - Stand/Sit (Triangle on PS2)
// L6+Right Joy UP/DOWN - Body up/down - (PS2 Dpad Up/Down)
// L6+Right Joy Left/Right - Speed higher/lower - (PS2 DPad left/right)
// Right Top(S7) - Cycle through options of Normal walk/Double Height/Double Travel) - (PS2 R1, R2)
// Left Top(S8) - Cycle through modes (Walk, Translate, Rotate, Single Leg) (PS2: Circle, X, L1, L2)

// Note: Left some descriptions of PS2 stuff, especially parts still left to Map/Implement.
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls] - Need to check...
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls] - How to do sequences???
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
#include <Arduino.h>
#include <Wire.h>

#if defined(ESP8266)
int HexapodPort = 123;
WiFiServer hexapodServer(HexapodPort);
WiFiClient hexapodClient;
#endif

#define PhantomSE_commander_addr 0x30 // 
#define cTravelDeadZone             6 // The deadzone for the analog input from the remote
#define ARBOTIX_TO               2000 // if we don't get a valid message in this number of mills turn off

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//[CONSTANTS]
enum { WALKMODE=0, TRANSLATEMODE, ROTATEMODE,
#ifdef OPT_SINGLELEG
  SINGLELEGMODE, 
#endif
#ifdef OPT_GPPLAYER
  GPPLAYERMODE, 
#endif
  MODECNT};
enum { NORM_NORM=0, NORM_LONG, HIGH_NORM, HIGH_LONG};

// BYTE 9 (external for TYPE)
#define TYPE_RC       1
#define TYPE_XBOX     2
#define TYPE_PS3      3
#define TYPE_PS4      4
#define TYPE_TELNET   5
/* bitmasks for buttons array */
// BYTE 8 (buttons_L)
#define BIT_TYPE      0x0001 // 0
#define BIT_R1        0x0002 // 1
#define BIT_RT        0x0004 // 2
#define BIT_TRIANGLE  0x0008 // 3
#define BIT_CIRCLE    0x0010 // 4
#define BIT_CROSS     0x0020 // 5
#define BIT_SQUARE    0x0040 // 6
#define BIT_START     0x0080 // 7
// BYTE 7 (buttons_H)
#define BIT_PS        0x0100 // 8
#define BIT_L1        0x0200 // 9
#define BIT_LT        0x0400 // 10
#define BIT_UP        0x0800 // 11
#define BIT_RIGHT     0x1000 // 12
#define BIT_DOWN      0x2000 // 13
#define BIT_LEFT      0x4000 // 14
#define BIT_SELECT    0x8000 // 15

// Control mapping
#define BTN_MODE        BIT_SELECT   // Cycle through modes
#define BTN_BALLANCE    BIT_SQUARE   // Switch Balance mode on/off
#define BTN_STANDSIT    BIT_TRIANGLE // Stand up, sit down
#define BTN_ALT         BIT_R1       // To control body offset as well as Speed with rightHat
#define BTN_DEBUG       BIT_CIRCLE   // Debug on/off
#define BTN_GAIT        BIT_L1       // [WALKMODE] Switch gates 
#define BTN_LEGLIFT     BIT_CROSS    // [WALKMODE] Double leg lift height
#define BTN_WALKM       BIT_PS       // [WALKMODE] Switch between Walk method
#define BTN_SWITCHSEQ   BIT_L1       // [GPPLAYERMODE] Switch between sequences
#define BTN_STARTSEQ    BIT_PS       // [GPPLAYERMODE] Start Sequence
#define BTN_SWITCHLEG   BIT_L1       // [SINGLELEGMODE] Switch leg
#define BTN_HOLDLEG     BIT_PS       // [SINGLELEGMODE] Hold single leg in place

/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */
class Commander {    
public:
  Commander(); 
  void begin(unsigned long baud);
  void SendMsgs();
  int ReadMsgs();
  int ReadMsgsWire();
  int ReadMsgsTelnet();

  void setLed(byte num);

  // joystick values are -125 to 125
  signed char   rightHatX;    // vertical stick movement = forward speed
  signed char   rightHatY;    // horizontal stick movement = sideways or angular speed
  signed char   leftHatX;     // vertical stick movement = tilt    
  signed char   leftHatY;     // horizontal stick movement = pan (when we run out of pan, turn body?)
  unsigned char analogL2;     // 
  unsigned char analogR2;     // 
  unsigned char external;     // Extended function set
  unsigned int  buttons;      // buttons are 0 or 1 (PRESSED), and bitmapped
  
  bool PhoenixCommanderConnected = false;
  bool PS3connected = false;
  bool RCconnected = false;
  bool TELNETconnected = false;

  unsigned int Timeout = 5000;
  unsigned long PrevTimeout = 0;
  byte TimeoutCount = 5;
  byte TimeoutCounter = 5;

private:
  int index;              // -1 = waiting for new packet
  int checksum;
};


//=============================================================================
// Global - Local to this file only...
//=============================================================================
Commander command = Commander();
unsigned long g_ulLastMsgTime;
short  g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)

#ifdef USEMULTI
//==============================================================================
//
// Lets define our Sub-class of the InputControllerClass
//
//==============================================================================
class CommanderInputController : 
public InputController {
public:
  CommanderInputController();        // A reall simple constructor...
  virtual bool     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);
};
CommanderInputController g_CommanderController;
//==============================================================================
// Constructor. See if there is a simple way to have one or more Input
//     controllers. Maybe register at construction time
//==============================================================================
CommanderInputController::CommanderInputController() { RegisterInputController(this); }
#else
#define CommanderInputController InputController
InputController  g_InputController;
#endif

static short   g_BodyYOffset; 
static short   g_BodyYShift;
static byte    ControlMode;
static byte    HeightSpeedMode;
//static bool  DoubleHeightOn;
static bool    DoubleTravelOn;
static byte    bJoystickWalkMode;
byte           GPSeq;             //Number of the sequence

static unsigned int buttonsPrev;
static byte         externalPrev;

// some external or forward function references.
extern void CommanderTurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
bool CommanderInputController::Init(void) {
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  command.begin(XBEE_BAUD);
  GPSeq = 0;  // init to something...
  ControlMode = WALKMODE;
  HeightSpeedMode = NORM_NORM;
  //    DoubleHeightOn = false;
  DoubleTravelOn = false;
  bJoystickWalkMode = 0;
  return true;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
 // We don't need to do anything...
void CommanderInputController::AllowControllerInterrupts(boolean fAllow __attribute__((unused))) {}

void printMode(byte mode, bool new_line=false);
void printMode(byte mode, bool new_line) {
  switch(mode) {
    case WALKMODE : printDebug("WALKMODE"); break;
    case TRANSLATEMODE : printDebug("TRANSLATEMODE"); break;
    case ROTATEMODE : printDebug("ROTATEMODE"); break;
    #ifdef OPT_SINGLELEG 
    case SINGLELEGMODE : printDebug("SINGLELEGMODE"); break;
    #endif
    #ifdef OPT_GPPLAYER
    case GPPLAYERMODE : printDebug("GPPLAYERMODE"); break;
    #endif
    default : printDebug("unknown"); break;
  }
  if(new_line) printlnDebug();
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void CommanderInputController::ControlInput(void) {
  
  byte hasInput = false;
  hasInput = command.ReadMsgs();
  if(hasInput>5) hasInput = 0;

  if(hasInput) {
    bool isChanged = false;
         if(command.rightHatX != 0) isChanged = true;
    else if(command.rightHatY != 0) isChanged = true;
    else if(command.leftHatX  != 0) isChanged = true;
    else if(command.leftHatY  != 0) isChanged = true;
    else if(command.buttons   != 0) isChanged = true;
    if(isChanged) {
      command.PrevTimeout = millis();
      command.TimeoutCounter = 0;
    } else {
      if(command.TimeoutCounter<command.TimeoutCount) {
        if(millis()-command.PrevTimeout>=command.Timeout) {
          command.TimeoutCounter++;
          MSound( 1, 50, 2000);
          command.PrevTimeout = millis();
        }
      } else hasInput = 0;
    }
  }
  
  if(hasInput) {

    if(hasInput==TYPE_PS3) {
      if(!command.PS3connected) {
        command.PS3connected = true;
        printlnDebug("PS3 connected");
      }
    } else if(hasInput==TYPE_RC) {
      if(!command.RCconnected) {
        command.RCconnected = true;
        printlnDebug("RC connected");
      }
    } else if(hasInput==TYPE_TELNET) {
      if(!command.TELNETconnected) {
        command.TELNETconnected = true;
        printlnDebug("TELNET connected");
      }
    }
    
    // If we receive a valid message than turn robot on...
    boolean fAdjustLegPositions = false;
    short   sLegInitXZAdjust    = 0;
    short   sLegInitAngleAdjust = 0;

    if (!g_InControlState.fRobotOn ) {
      g_InControlState.fRobotOn = true;
      fAdjustLegPositions = true;
      printlnDebug("Turn Robot ON");
      printDebug("ControlMode: ");
      printMode(ControlMode, true);
    }

    // [SWITCH MODES]
    // Cycle through modes...
    if((command.buttons & BTN_MODE) && !(buttonsPrev & BTN_MODE)) {
      printDebug("In Command: Switch mode ");
      printMode(ControlMode);
      if (++ControlMode >= MODECNT) {
        ControlMode = WALKMODE;    // cycled back around...
        MSound( 2, 50, 2000, 50, 3000); 
      } else MSound( 1, 50, 2000);  
      printDebug(" --> ");
      printMode(ControlMode, true);
      #ifdef OPT_SINGLELEG      
      if(ControlMode != SINGLELEGMODE) g_InControlState.SelectedLeg = 255;
      else {
        g_InControlState.SelectedLeg = 0;   // Select leg 0 when we go into this mode.
        g_InControlState.PrevSelectedLeg = 255;
      }
      #endif
      //if(ControlMode<4) command.setLed(ControlMode+1);
    }

    // [Common functions]
    // Switch Balance mode on/off 
    if ((command.buttons & BTN_BALLANCE) && !(buttonsPrev & BTN_BALLANCE)) {
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      printDebug("In Command: Balance mode ");
      if (g_InControlState.BalanceMode) {
        printlnDebug("ON");
        MSound( 1, 250, 1500); 
      } else {
        printlnDebug("OFF");
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    // Stand up, sit down  
    if ((command.buttons & BTN_STANDSIT) && !(buttonsPrev & BTN_STANDSIT)) {
      printDebug("In Command: ");
      if (g_BodyYOffset>0) {
        g_BodyYOffset = 0;
        printlnDebug("Sit down");
        MSound( 1, 250, 1500);
      } else {
        g_BodyYOffset = 35;
        printlnDebug("Stand up");
        MSound( 2, 100, 2000, 50, 4000);
      }
      fAdjustLegPositions = true;
      g_fDynamicLegXZLength = false;
    }

    // We will use L6 with the Right joystick to control both body offset as well as Speed...
    // We move each pass through this by a percentage of how far we are from center in each direction
    // We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
    // which give an idea, but only for those whoes robot has a speaker
    int lx = command.leftHatX;
    int ly = command.leftHatY;

    if (command.buttons & BTN_ALT ) {
      // raise or lower the robot on the joystick up /down
      // Maybe should have Min/Max
      int delta = command.rightHatY/25;
      if (delta) {
        g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
        fAdjustLegPositions = true;
        printDebug("In command: rightHatY[");
        printDebug(command.rightHatY);
        printDebug("] Change BODY_Y = ");
        printlnDebug(g_BodyYOffset);
      }

      // Also use right Horizontal to manually adjust the initial leg positions.
      sLegInitXZAdjust = lx/10;        // play with this.
      sLegInitAngleAdjust = ly/8;
      lx = 0;
      ly = 0;

      // Likewise for Speed control
      delta = command.rightHatX / 16;
      if ((delta < 0) && g_InControlState.SpeedControl) {
        if ((word)(-delta) <  g_InControlState.SpeedControl) g_InControlState.SpeedControl += delta;
        else g_InControlState.SpeedControl = 0;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl);
      }
      if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
        g_InControlState.SpeedControl += delta;
        if (g_InControlState.SpeedControl > 2000) g_InControlState.SpeedControl = 2000;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl);
      }
      if(delta) {
        printDebug("In command: rightHatX[");
        printDebug(command.rightHatX);
        printDebug("] Change SPEED = ");
        printlnDebug(g_InControlState.SpeedControl);
      }
      command.rightHatX = 0; // don't walk when adjusting the speed here...
    }

    if ((command.buttons & BTN_DEBUG) && !(buttonsPrev & BTN_DEBUG)) {
      MSound(1, 50, 2000);
      g_fDebugOutput = !g_fDebugOutput;
      printDebug("In Command: DEBUG ");
      if(g_fDebugOutput) printlnDebug("ON");
      else printlnDebug("OFF");
      setDebug(g_fDebugOutput);
    }

    //========================================================================//
    yield();
    //========================================================================//
    
    // [Walk functions]
    if (ControlMode == WALKMODE) {
      // Switch gates
      if ( ((command.buttons & BTN_GAIT) && !(buttonsPrev & BTN_GAIT)) &&
           abs(g_InControlState.TravelLength.x)<cTravelDeadZone && //No movement
           abs(g_InControlState.TravelLength.z)<cTravelDeadZone &&
           abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        printDebug("In Command: switch gates ");
        printDebug(g_InControlState.GaitType);
        g_InControlState.GaitType++; // = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) MSound( 1, 50, 2000);                 // Make sure we did not exceed number of gaits...
        else {
          MSound (2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        printDebug(" --> ");
        printlnDebug(g_InControlState.GaitType);
        GaitSelect();
      }

      // Double leg lift height
      if ((command.buttons & BTN_LEGLIFT) && !(buttonsPrev & BTN_LEGLIFT)) {
        printlnDebug("In Command: double leg lift height");
        MSound( 1, 50, 2000);  
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2) g_InControlState.LegLiftHeight = 80;
        else g_InControlState.LegLiftHeight = 50;
      }

      // Switch between Walk method 1 && Walk method 2
      if ((command.buttons & BTN_WALKM) && !(buttonsPrev & BTN_WALKM)) {
        printDebug("In Command: switch Walk method ");
        printDebug(bJoystickWalkMode);
        #ifdef cTurretRotPin
        if ((++bJoystickWalkMode) > 2) bJoystickWalkMode = 0;
        #else
        if ((++bJoystickWalkMode) > 1) bJoystickWalkMode = 0;
        #endif
        printDebug(" --> ");
        printlnDebug(bJoystickWalkMode);
        MSound (1, 50, 2000 + bJoystickWalkMode*250);
      }

      // Walking
      switch (bJoystickWalkMode) {
        case 0: g_InControlState.TravelLength.x = -lx;
                g_InControlState.TravelLength.z = -ly;
                g_InControlState.TravelLength.y = -(command.rightHatX)/4; //Right Stick Left/Right 
                break;
        case 1: g_InControlState.TravelLength.z = (command.rightHatY); //Right Stick Up/Down  
                g_InControlState.TravelLength.y = -(command.rightHatX)/4; //Right Stick Left/Right 
                break;
        #ifdef cTurretRotPin
        case 2: g_InControlState.TravelLength.x = -lx;
                g_InControlState.TravelLength.z = -ly;
                // Will use Right now stick to control turret.
                g_InControlState.TurretRotAngle1 =  max(min(g_InControlState.TurretRotAngle1+command.rightHatX/5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
                g_InControlState.TurretTiltAngle1 =  max(min(g_InControlState.TurretTiltAngle1+command.rightHatY/5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
        #endif
        default : break;
      }

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

    } // end [Walk functions]

    // [Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x =  SmoothControl(((lx)*2/3), g_InControlState.BodyPos.x, SmDiv);
      g_InControlState.BodyPos.z =  SmoothControl(((ly)*2/3), g_InControlState.BodyPos.z, SmDiv);
      g_InControlState.BodyRot1.y = SmoothControl(((command.rightHatX)*2), g_InControlState.BodyRot1.y, SmDiv);
      //      g_InControlState.BodyPos.x = (lx)/2;
      //      g_InControlState.BodyPos.z = -(ly)/3;
      //      g_InControlState.BodyRot1.y = (command.rightHatX)*2;
      g_BodyYShift = (-(command.rightHatY)/2);
    } // end [Translate functions]

    // [Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (ly);
      g_InControlState.BodyRot1.y = (command.rightHatX)*2;
      g_InControlState.BodyRot1.z = (lx);
      g_BodyYShift = (-(command.rightHatY)/2);
    } // end [Rotate functions]

    // [GPPlayer functions]
    #ifdef OPT_GPPLAYER
    if (ControlMode == GPPLAYERMODE) {
      // Lets try some speed control... Map all values if we have mapped some before
      // or start mapping if we exceed some minimum delta from center
      // Have to keep reminding myself that commander library already subtracted 128...
      if (g_ServoDriver.FIsGPSeqActive() ) {
        if ( (g_sGPSMController != 32767) || (command.rightHatY > 16) || (command.rightHatY < -16) ) {
          // We are in speed modify mode...
          if (command.rightHatY >= 0) g_sGPSMController = map(command.rightHatY, 0, 127, 0, 200);
          else g_sGPSMController = map(command.rightHatY, -127, 0, -200, 0);
          g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
        }
      }

      // Switch between sequences
      if ((command.buttons & BTN_SWITCHSEQ) && !(buttonsPrev & BTN_SWITCHSEQ)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          printDebug("In Command: switch sequences ");
          printDebug(GPSeq);
          if (GPSeq < 5) {  //Max sequence
            MSound (1, 50, 1500);  
            GPSeq = GPSeq+1;
          } else {
            MSound (2, 50, 2000, 50, 2250);
            GPSeq = 0;
          }
          printDebug(" --> ");
          printlnDebug(GPSeq);
        }
      }
      
      // Start Sequence
      if ((command.buttons & BTN_STARTSEQ) && !(buttonsPrev & BTN_STARTSEQ)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          printlnDebug("In Command: start GP sequences");
          MSound( 2, 100, 2000, 50, 4000);
          g_ServoDriver.GPStartSeq(GPSeq);
          g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
        } else {
          g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
          printlnDebug("In Command: abort GP sequences");
          MSound (2, 50, 2000, 50, 2000);
        }
      }

    }      // end [GPPlayer functions]
    #endif // OPT_GPPLAYER

    // [Single leg functions]
    #ifdef OPT_SINGLELEG      
    if (ControlMode == SINGLELEGMODE) {
      // Switch leg for single leg control
      if ((command.buttons & BTN_SWITCHLEG) && !(buttonsPrev & BTN_SWITCHLEG)) {
        printDebug("In Command: switch single leg ");
        printDebug(g_InControlState.SelectedLeg);
        MSound (1, 50, 2000);  
        if (g_InControlState.SelectedLeg<(CNT_LEGS-1)) g_InControlState.SelectedLeg++;
        else g_InControlState.SelectedLeg = 0;
        printDebug(" --> ");
        printlnDebug(g_InControlState.SelectedLeg);
      }

      #if 0
      g_InControlState.SLLeg.x= (signed char)((int)((int)lx+128)/2); //Left Stick Right/Left
      g_InControlState.SLLeg.y= (signed char)((int)((int)command.rightHatY+128)/10); //Right Stick Up/Down
      g_InControlState.SLLeg.z = (signed char)((int)((int)ly+128)/2); //Left Stick Up/Down
      #else
      // BUGBUG:: Need to figure out a decent range for these values... 
      g_InControlState.SLLeg.x = lx; //Left Stick Right/Left
      g_InControlState.SLLeg.y = command.rightHatY / 3 - 20; //Right Stick Up/Down
      g_InControlState.SLLeg.z = ly; //Left Stick Up/Down
      #endif
      
      #ifdef DEBUG_SINGLELEG
      printDebug(g_InControlState.SLLeg.x, DEC);
      printDebug(",");
      printDebug(g_InControlState.SLLeg.y, DEC);
      printDebug(",");
      printlnDebug(g_InControlState.SLLeg.z, DEC);
      #endif
      
      // Hold single leg in place
      if ((command.buttons & BTN_HOLDLEG) && !(buttonsPrev & BTN_HOLDLEG)) {
        printDebug("In Command: single leg ");
        MSound (1, 50, 2000);  
        g_InControlState.fSLHold = !g_InControlState.fSLHold;
        if(g_InControlState.fSLHold) printlnDebug("holded");
        else printlnDebug("released");
      }
      
    }      // [Single leg functions]
    #endif // OPT_SINGLELEG

    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(command.rightHatX));

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift,  0);

    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when we have finished any previous adjustment
      if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust) g_fDynamicLegXZLength = true;
        sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
        // Handle maybe change angles...
        if (sLegInitAngleAdjust) RotateLegInitAngles(sLegInitAngleAdjust);
        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }    

    // Put main workings into main program file
    if (fAdjustLegPositions && !g_fDynamicLegXZLength) AdjustLegPositionsToBodyHeight();

    // Save away the buttons state as to not process the same press twice.
    buttonsPrev  = command.buttons;
    externalPrev = command.external;
    g_ulLastMsgTime = millis();
  } // else hasInput == false
  else {
    // We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
    if (g_InControlState.fRobotOn) {
      if ((millis() - g_ulLastMsgTime) > ARBOTIX_TO) CommanderTurnRobotOff();
    }
  }
  
}

//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderTurnRobotOff(void) {
  if(command.PS3connected) {
    command.PS3connected = false;
    printlnDebug("PS3 disconnected");
  }
  if(command.RCconnected) {
    command.RCconnected = false;
    printlnDebug("RC disconnected");
  }
  if(command.TELNETconnected) {
    command.TELNETconnected = false;
    printlnDebug("TELNET disconnected");
  }
  printlnDebug("Turn Robot OFF"); // Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  #ifdef OPT_SINGLELEG      
  g_InControlState.SelectedLeg = 255;
  #endif
  g_InControlState.fRobotOn = 0;
  #ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
  #endif
  g_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}

//==============================================================================
// Commander::Commander - Constructor
//==============================================================================
Commander::Commander() { index = -1; }

//==============================================================================
// Commander::begin 
//==============================================================================
void Commander::begin(unsigned long baud) {

}

//==============================================================================
// Commander::setLed 
//==============================================================================
byte sendMsgBuf[3];
void Commander::setLed(byte num) {
   sendMsgBuf[0] = num;
   sendMsgBuf[1] = 1;
   sendMsgBuf[2] = 2;
   SendMsgs();
}

//==============================================================================
// Commander::SendMsgs 
//==============================================================================
void Commander::SendMsgs() {
  Wire.beginTransmission(PhantomSE_commander_addr);
  Wire.write(sendMsgBuf, sizeof(sendMsgBuf));
  Wire.endTransmission();
}

//==============================================================================
// ReadMsgs
// *  format = 0xFF RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT CHECKSUM */
//==============================================================================
int Commander::ReadMsgs() {
  return ReadMsgsWire();
  //byte state = ReadMsgsWire();
  //if(state) return state;
  //return ReadMsgsTelnet();
}

//==============================================================================
// ReadMsgsWire
// * process messages coming from I2C 
// ***** Data Format ***** //
// BYTE 0  - start byte (0xFF)
// BYTE 1  - RightHatX  (signed)
// BYTE 2  - RightHatY  (signed)
// BYTE 3  - LeftHatX   (signed)
// BYTE 4  - LeftHatY   (signed)
// BYTE 5  - AnalogL2   (signed)
// BYTE 6  - AnalogR2   (signed)
// BYTE 7  - buttons_H  (button PS & Left side button mask)
// BYTE 8  - buttons_L  (Right side button mask)
// BYTE 9  - external   (controller type)
// BYTE 10 - checksum
//*************************//
//==============================================================================
unsigned long prevI2Ctest = 0;
unsigned long prevI2Creq = 0;
int device_addr = 0;
int Commander::ReadMsgsWire() {
  external = 0;
  if(millis() - prevI2Ctest >= 1000) {
    Wire.beginTransmission(PhantomSE_commander_addr);
    if(Wire.endTransmission()==0) {
      if(!PhoenixCommanderConnected) {
        PhoenixCommanderConnected = true;
        device_addr = PhantomSE_commander_addr;
        printlnDebug("PhoenixCommander connected");
      }
    } else if(PhoenixCommanderConnected) {
      PhoenixCommanderConnected = false;
      device_addr = 0;
      printlnDebug("PhoenixCommander disconnected");
    }
    prevI2Ctest = millis();
    prevI2Creq = millis();
  }
  if(device_addr) {
    if(millis() - prevI2Creq >= 20) {
      Wire.requestFrom(device_addr, 11);
      byte vals[11];
      int checksum = 0;
      byte index = 0;
      while(Wire.available()) {
        byte d = Wire.read();
        if(index<11) {
          vals[index] = d;
          index++;
        }
      }
      if(index==11) {
        if(vals[9]) {
          if(!vals[1]) vals[1] = 1;
          if(!vals[2]) vals[2] = 1;
          if(!vals[3]) vals[3] = 1;
          if(!vals[4]) vals[4] = 1;
          rightHatX =  (signed char) (vals[1]-128);
          rightHatY = -(signed char) (vals[2]-128);
          leftHatX  =  (signed char) (vals[3]-128);
          leftHatY  = -(signed char) (vals[4]-128);
          analogL2  = vals[5];
          analogR2  = vals[6];
          buttons   = vals[7]; buttons = buttons << 8;
          buttons  |= vals[8];
          external  = vals[9];
        }
      }
      prevI2Creq = millis();
    }
  }
  return external;
}

//==============================================================================
//* process messages coming from Telnet
// ***** Data Format ***** //
// BYTE 0  - start byte (0xFF)
// BYTE 1  - RightHatX  (signed)
// BYTE 2  - RightHatY  (signed)
// BYTE 3  - LeftHatX   (signed)
// BYTE 4  - LeftHatY   (signed)
// BYTE 5  - AnalogL2   (signed)
// BYTE 6  - AnalogR2   (signed)
// BYTE 7  - buttons_H  (button PS & Left side button mask)
// BYTE 8  - buttons_L  (Right side button mask)
// BYTE 9  - external   (controller type)
// BYTE 10 - checksum
//*************************//
//==============================================================================
bool HexapodTelnetInited = false;
unsigned long SecMillis  = 0;
unsigned long BPS        = 0;
byte vals[11];
int Commander::ReadMsgsTelnet() {
  external = 0;
  if(millis() - SecMillis >= 1000) {
    printDebug("BPS: ");
    printlnDebug(BPS);
    BPS = 0;
    SecMillis = millis();
  }
  #if defined(ESP8266)
  if(WiFi.status() != WL_CONNECTED) return 0;
  if(!HexapodTelnetInited) {
    hexapodServer.begin();
    hexapodServer.setNoDelay(true);
    HexapodTelnetInited = true;
  }
  //check if there are any new clients
  if(hexapodServer.hasClient()) {
    //find free/disconnected spot
    for(byte i=0; i < 1; i++) {
      if(!hexapodClient || !hexapodClient.connected()) {
        if(hexapodClient) hexapodClient.stop();
        hexapodClient = hexapodServer.available();
        hexapodClient.setNoDelay(true);
        continue;
      }
    }
    // no free/disconnected spot so reject
    WiFiClient hexapodClient = hexapodServer.available();
    hexapodClient.stop();
  }
  //--------------------------------------------------------
  if(!hexapodClient || !hexapodClient.connected()) return 0;
  while(hexapodClient.available()) {
    byte in = hexapodClient.read();
    BPS++;
    if(index == -1) {         // looking for new packet
      if(in == 0xff) {
        vals[0] = in;
        index = 1;
        checksum = 0;
      }
    }
    else if(index == 1) {
      vals[index] = in;
      if(vals[index] != 0xff){            
        checksum += (int) vals[index];
        index++;
      }
    } else if(index > 1) {
      vals[index] = in;
      checksum += (int) vals[index];
      index++;
      if(index == 11) { // packet complete
        if(checksum%256 != 255) {
          // packet error!
          index = -1;
          printlnDebug(" !!! checksum error !!!");
          return 0;
        } else {
          if(!vals[1]) vals[1] = 1;
          if(!vals[2]) vals[2] = 1;
          if(!vals[3]) vals[3] = 1;
          if(!vals[4]) vals[4] = 1;
          rightHatX =  (signed char) (vals[1]-128);
          rightHatY = -(signed char) (vals[2]-128);
          leftHatX  =  (signed char) (vals[3]-128);
          leftHatY  = -(signed char) (vals[4]-128);
          analogL2  = vals[5];
          analogR2  = vals[6];
          buttons   = vals[7]; buttons = buttons << 8;
          buttons  |= vals[8];
          external  = 5;
        }
        index = -1;
        while(hexapodClient.read() != -1) BPS++;
        return external;
      }
    }
  }
  #endif
  return 0;
}
//==============================================================================

