
//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//
//Programmer: Jeroen Janssen [aka Xan]
//            Kurt Eckhardt(KurtE)        - converted to C and Arduino
//            Kare Halvorsen aka Zenta    - Makes everything work correctly!  
//            Denis Silivanov (denis_n73) - edit for own PhantomSE version
//
// This version of the Phoenix code was correct for ESP8266, simple servos and external recievers
//
// Phoenix.h - This is the first header file that is needed to build Phoenix program
//          a Phoenix program for a specific Hex Robot.
//=============================================================================
#define DEFINE_HEX_GLOBALS
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#define HEXMODE   // default to hex mode
#include "PhoenixHEXA.h"
#include "ax12.h"
#include <stdarg.h>
//==============================================================================

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP   1

#define c1DEC       10
#define c2DEC       100
#define c4DEC       10000
#define c6DEC       1000000

#ifdef QUADMODE
enum { cRR=0, cRF, cLR, cLF, CNT_LEGS};
#elif defined(OCTOMODE)
enum { cRR=0, cRMR, cRMF, cRF, cLR, cLMR, cLMF, cLF, CNT_LEGS};
#else
enum { cRR=0, cRM, cRF, cLR, cLM, cLF, CNT_LEGS};
#endif

#define WTIMERTICSPERMSMUL      64  // BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192
#define WTIMERTICSPERMSDIV      125 // 
#define USEINT_TIMERAV

// BUGBUG: to make Dynamic first pass simpl make it a variable.
extern  byte    NUM_GAITS;
#define SmDiv        4     // "Smooth division" factor for the smooth control function (3 to 5 is most suitable)
extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider);

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern boolean          g_fDebugOutput;
extern boolean          g_fEnableServos;      // Hack to allow me to turn servo processing off...
extern boolean          g_fRobotUpsideDown;    // Is the robot upside down?

extern void MSound(byte cNotes, ...);
extern boolean CheckVoltage(void);

extern word GetLegsXZLength(void);
extern void AdjustLegPositions(word XZLength1);
extern void AdjustLegPositionsToBodyHeight();
extern void ResetLegInitAngles(void);
extern void RotateLegInitAngles (int iDeltaAngle);
extern long GetCmdLineNum(byte **ppszCmdLine);

extern boolean g_fDBGHandleError; // debug handler...

#ifdef c4DOF
extern const byte cTarsLength[] PROGMEM;
#endif

#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   g_ServoDriver.BackgroundProcess()
#else
#define DoBackgroundProcess()   
#endif

#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif

#ifdef __AVR__
#if not defined(UBRR1H)
#if cSSC_IN != 0
extern SoftwareSerial SSCSerial;
#endif
#endif
#endif
#if defined(__PIC32MX__)
#if defined F
#undef F
#endif
#define F(X) (X)
#endif

//=============================================================================
//=============================================================================
// Define the class(s) for our Input controllers.  
//=============================================================================
//=============================================================================
class InputController {
public:
  virtual bool     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);
#ifdef OPT_TERMINAL_MONITOR_IC  // Allow Input controller to define stuff as well
  void            ShowTerminalCommandList(void);
  boolean         ProcessTerminalCommand(byte *psz, byte bLen);
#endif
private:
};
// Define a function that allows us to define which controllers are to be used.
extern void  RegisterInputController(InputController *pic);
typedef struct _Coord3D {
  long      x;
  long      y;
  long      z;
} COORD3D;

//==============================================================================
// Define Gait structure/class - Hopefully allow specific robots to define their
// own gaits and/or define which of the standard ones they want.
//==============================================================================
typedef struct _PhoenixGait {
  short           NomGaitSpeed;       //Nominal speed of the gait
  byte            StepsInGait;         //Number of steps in gait
  byte            NrLiftedPos;         //Number of positions that a single leg is lifted [1-3]
  byte            FrontDownPos;        //Where the leg should be put down to ground
  byte            LiftDivFactor;       //Normaly: 2, when NrLiftedPos=5: 4
  byte            TLDivFactor;         //Number of steps that a leg is on the floor while walking
  byte            HalfLiftHeight;      // How high to lift at halfway up.
#ifdef QUADMODE
  // Extra information used in the Quad balance mode
  word            COGAngleStart1;      // COG shifting starting angle
  word            COGAngleStep1;       // COG Angle Steps in degrees
  byte            COGRadius;           // COG Radius; the amount the body shifts
  boolean         COGCCW;              // COG Gait sequence runs counter clock wise
#endif    
  byte            GaitLegNr[CNT_LEGS]; //Init position of the leg
#ifdef DISPLAY_GAIT_NAMES
  PGM_P           pszName;             // The gait name
#endif
} PHOENIXGAIT;

#ifdef DISPLAY_GAIT_NAMES
#define GATENAME(name)  ,name
#else
#define GATENAME(name)
#endif

//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _InControlState {
  boolean       fRobotOn;            //Switch to turn on Phoenix
  boolean       fPrev_RobotOn;       //Previous loop state 
  //Body position
  COORD3D       BodyPos;
  COORD3D       BodyRotOffset;      // Body rotation offset;
  //Body Inverse Kinematics
  COORD3D       BodyRot1;            // X -Pitch, Y-Rotation, Z-Roll
  //[gait]
  byte          GaitType;            //Gait type
  byte          GaitStep;            //Actual current step in gait
  PHOENIXGAIT   gaitCur;             // Definition of the current gait
  short       LegLiftHeight;       //Current Travel height
  COORD3D       TravelLength;        // X-Z or Length, Y is rotation.
#ifdef cTurretRotPin
  // Turret information
  int           TurretRotAngle1;      // Rotation of turrent in 10ths of degree
  int           TurretTiltAngle1;    // the tile for the turret
#endif
  //[Single Leg Control]
#ifdef OPT_SINGLELEG
  byte          SelectedLeg;
  byte          PrevSelectedLeg;
  COORD3D       SLLeg;               // 
  boolean       fSLHold;             //Single leg control mode
#endif
  //[Balance]
  boolean       BalanceMode;
  //[TIMING]
  byte          InputTimeDelay; //Delay that depends on the input to get the "sneaking" effect
  word          SpeedControl;   //Adjustible Delay
  byte          ForceGaitStepCnt;          // new to allow us to force a step even when not moving
#ifdef OPT_DYNAMIC_ADJUST_LEGS
  short         aCoxaInitAngle1[CNT_LEGS]; 
#endif
} INCONTROLSTATE;

//==============================================================================
//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================
//==============================================================================
class ServoDriver {
public:
  bool Init(void);
  word GetBatteryVoltage(void);
#ifdef OPT_GPPLAYER    
  inline boolean  FIsGPEnabled(void) { return _fGPEnabled; };
  boolean         FIsGPSeqDefined(uint8_t iSeq);
  inline boolean  FIsGPSeqActive(void) { return _fGPActive; };
  void            GPStartSeq(uint8_t iSeq);  // 0xff - says to abort...
  void            GPPlayer(void);
  uint8_t         GPNumSteps(void);          // How many steps does the current sequence have
  uint8_t         GPCurStep(void);           // Return which step currently on... 
  void            GPSetSpeedMultiplyer(short sm) ;      // Set the Speed multiplier (100 is default)
#endif
  void            BeginServoUpdate(void);    // Start the update 
#ifdef c4DOF
  void            OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
#else
  void            OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
#endif    
#ifdef cTurretRotPin
  void            OutputServoInfoForTurret(short sRotateAngle1, short sTiltAngle1);
#endif
  void            CommitServoDriver(word wMoveTime);
  void            FreeServos(void);
  void            IdleTime(void);        // called when the main loop when the robot is not on
  // Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
  void            BackgroundProcess(void);
#endif    
#ifdef OPT_TERMINAL_MONITOR  
  void            ShowTerminalCommandList(void);
  boolean         ProcessTerminalCommand(byte *psz, byte bLen);
#endif
private:
#ifdef OPT_GPPLAYER    
  boolean _fGPEnabled;     // IS GP defined for this servo driver?
  boolean _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
  uint8_t    _iSeq;        // current sequence we are running
  short    _sGPSM;        // Speed multiplier +-200 
#endif
};   

//==============================================================================
//==============================================================================
// Define global class objects
//==============================================================================
//==============================================================================
extern ServoDriver      g_ServoDriver;           // our global servo driver class
extern InputController  g_InputController;       // Our Input controller 
extern INCONTROLSTATE   g_InControlState;        // State information that controller changes
//==============================================================================
#ifdef QUADMODE
#define ADD_GAITS
#define PYPOSE_GAIT_SPEED 1 //98
//  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground, 
//     Quad extra: COGAngleStart, COGAngleStep, CogRadius, COGCCW
//                      { RR, RF, LR, LF}
PHOENIXGAIT APG_EXTRA[] = { 
  {PYPOSE_GAIT_SPEED, 8, 2, 1, 2, 6, 1, 0, 0,0, true, {7, 1, 3, 5}},   // ripple
  {PYPOSE_GAIT_SPEED, 12, 2, 1, 2, 10, 1, 0, 0,0, true, {7, 1, 4, 10}},   // ripple
  {PYPOSE_GAIT_SPEED, 4, 2, 1, 2, 2, 1, 0, 0, 0, true,{3, 1, 1, 3}},  // Amble
  {PYPOSE_GAIT_SPEED, 6, 3, 2, 2, 3, 2, 0, 0,0, true, {1, 4, 4, 1}} }; // Smooth Amble 
#endif

//==============================================================================
#include "PhoenixCommander.h"
#include "PhoenixDriver.h"
#include "PhoenixCode.h"
//==============================================================================

//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================

//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void Phoenix_setup() {

  #ifdef OPT_SKETCHSETUP
  SketchSetup();
  #endif
  g_fShowDebugPrompt = true;
  g_fDebugOutput = false;

  // Init our ServoDriver
  printDebug("[PhantomSE] Init servoDriver...");
  if(g_ServoDriver.Init()) printlnDebug("OK!"); 
  else printlnDebug("Fail!");
  
  // Setup Init Positions
  for (LegIndex= 0; LegIndex < CNT_LEGS; LegIndex++ ) {
    LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);    //Set start positions for each leg
    LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
    LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);  
  }

  ResetLegInitAngles();

  //Single leg control. Make sure no leg is selected
  #ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg = 255; // No Leg selected
  g_InControlState.PrevSelectedLeg = 255;
  #endif
  
  //Body Positions
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;

  //Body Rotations
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.BodyRotOffset.x = 0;
  g_InControlState.BodyRotOffset.y = 0;        //Input Y offset value to adjust centerpoint of rotation
  g_InControlState.BodyRotOffset.z = 0;

  //Gait
  g_InControlState.GaitType = 0; 
  g_InControlState.BalanceMode = 0;
  g_InControlState.LegLiftHeight = 50;
  g_InControlState.ForceGaitStepCnt = 0;    // added to try to adjust starting positions depending on height...
  g_InControlState.GaitStep = 1;
  GaitSelect();

  #ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
  #endif

  printDebug("[PhantomSE] Init inputController...");
  if(g_InputController.Init()) printlnDebug("OK!");
  else printlnDebug("Fail!");

  // Servo Driver
  ServoMoveTime = 150;
  g_InControlState.fRobotOn = 0;
  g_fLowVoltageShutdown = false;
  
  #ifdef OPT_WALK_UPSIDE_DOWN
  g_fRobotUpsideDown = false; // Assume off... 
  #ifdef DBGSerial
  printDebug("OPT_WALK_UPSIDE_DOWN ");
  printlnDebug(IsRobotUpsideDown, DEC);
  #endif  
  #endif

}

//=============================================================================
// Loop: the main arduino main Loop function
//=============================================================================
unsigned long lifePrevMillis = 0;
unsigned long testPrevMillis = 0;
void Phoenix_loop(void) {
  
  // Start time
  unsigned long lTimeWaitEnd;
  lTimerStart = millis();

  DoBackgroundProcess();
  
  CheckVoltage();        // check our voltages...
  
  if(!g_fLowVoltageShutdown) g_InputController.ControlInput();
  
  #ifdef IsRobotUpsideDown
  if (!fWalking){// dont do this while walking
    g_fRobotUpsideDown = IsRobotUpsideDown;    // Grab the current state of the robot... 
    if (g_fRobotUpsideDown != fRobotUpsideDownPrev) {
      // Double check to make sure that it was not a one shot error
      g_fRobotUpsideDown = IsRobotUpsideDown;    // Grab the current state of the robot... 
      if (g_fRobotUpsideDown != fRobotUpsideDownPrev) {
        fRobotUpsideDownPrev = g_fRobotUpsideDown;
        #ifdef DGBSerial        
        DBGSerial.println(fRobotUpsideDownPrev, DEC);
        #endif        
      }
    }
  }
  //  DBGSerial.println(analogRead(0), DEC);
  #endif

  #ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){
    g_InControlState.TravelLength.x = -g_InControlState.TravelLength.x;
    g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
    g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
    g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
  }
  #endif

  #ifdef OPT_GPPLAYER // GP Player
  g_ServoDriver.GPPlayer();
  if (g_ServoDriver.FIsGPSeqActive()) return;  // go back to process the next message
  #endif

  //Single leg control
  SingleLegControl();
  
  DoBackgroundProcess();

  //Gait
  GaitSeq();

  DoBackgroundProcess();

  //Balance calculations
  TotalTransX = 0;     //reset values used for calculation of balancecgfqlth
  
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal1 = 0;
  TotalYBal1 = 0;
  TotalZBal1 = 0;
  
  if (g_InControlState.BalanceMode) {
    #ifdef DEBUG
    if (g_fDebugOutput) {
      TravelRequest = (abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) 
        || (abs(g_InControlState.TravelLength.y)>cTravelDeadZone) || (g_InControlState.ForceGaitStepCnt != 0) || fWalking;
      DBGSerial.print("T("); 
      DBGSerial.print(fWalking, DEC);
      DBGSerial.print(" ");
      DBGSerial.print(g_InControlState.TravelLength.x,DEC); 
      DBGSerial.print(","); 
      DBGSerial.print(g_InControlState.TravelLength.y,DEC); 
      DBGSerial.print(","); 
      DBGSerial.print(g_InControlState.TravelLength.z,DEC); 
      DBGSerial.print(")"); 
    }
    #endif
    for (LegIndex = 0; LegIndex < (CNT_LEGS/2); LegIndex++) {    // balance calculations for all Right legs
      DoBackgroundProcess();
      BalCalcOneLeg (-LegPosX[LegIndex]+GaitPosX[LegIndex], LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
          (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }
    for (LegIndex = (CNT_LEGS/2); LegIndex < CNT_LEGS; LegIndex++) {    // balance calculations for all Right legs
      DoBackgroundProcess();
      BalCalcOneLeg(LegPosX[LegIndex]+GaitPosX[LegIndex], LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
          (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
  }

  //Reset IKsolution indicators 
  IKSolution = 0 ;
  IKSolutionWarning = 0; 
  IKSolutionError = 0 ;

  //Do IK for all Right legs
  for (LegIndex = 0; LegIndex < (CNT_LEGS/2); LegIndex++) {    
    DoBackgroundProcess();
    BodyFK(-LegPosX[LegIndex]+g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);
    LegIK (LegPosX[LegIndex]-g_InControlState.BodyPos.x+BodyFKPosX-(GaitPosX[LegIndex] - TotalTransX), 
    LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Do IK for all Left legs  
  for (LegIndex = (CNT_LEGS/2); LegIndex < CNT_LEGS; LegIndex++) {
    DoBackgroundProcess();
    BodyFK(LegPosX[LegIndex]-g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);
    LegIK (LegPosX[LegIndex]+g_InControlState.BodyPos.x-BodyFKPosX+GaitPosX[LegIndex] - TotalTransX,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }
  
  #ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){ //Need to set them back for not messing with the SmoothControl
    g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
    g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
    g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
  }
  #endif

  //Check mechanical limits
  CheckAngles();

  //Write IK errors to leds
  LedC = IKSolutionWarning;
  LedA = IKSolutionError;

  //Drive Servos
  if (g_InControlState.fRobotOn) {
    if (g_InControlState.fRobotOn && !g_InControlState.fPrev_RobotOn) {
      MSound(3, 60, 2000, 80, 2250, 100, 2500);
      Eyes = 1;
    }
    
    //Calculate Servo Move time
    if ((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) ||
      (abs(g_InControlState.TravelLength.y*2)>cTravelDeadZone)) {
      ServoMoveTime = g_InControlState.gaitCur.NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;
      //Add aditional delay when Balance mode is on
      if (g_InControlState.BalanceMode) ServoMoveTime = ServoMoveTime + BALANCE_DELAY;
    } else {
      ServoMoveTime = 200 + g_InControlState.SpeedControl; // Movement speed excl. Walking
    }

    // note we broke up the servo driver into start/commit that way we can output all of the servo information
    // before we wait and only have the termination information to output after the wait.  That way we hopefully
    // be more accurate with our timings...
    DoBackgroundProcess();
        
    StartUpdateServos();
    
    // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed 
    // before sending the next one

    // Finding any incident of GaitPos/Rot <>0:
    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
      if ( (GaitPosX[LegIndex] > cGPlimit) || (GaitPosX[LegIndex] < -cGPlimit)
        || (GaitPosZ[LegIndex] > cGPlimit) || (GaitPosZ[LegIndex] < -cGPlimit) 
        || (GaitRotY[LegIndex] > cGPlimit) || (GaitRotY[LegIndex] < -cGPlimit)) {
        // For making sure that we are using timed move until all legs are down
        bExtraCycle = g_InControlState.gaitCur.NrLiftedPos + 1;
        break;
      }
    }
    
    if (bExtraCycle > 0) {
      bExtraCycle--;
      fWalking = !(bExtraCycle==0);
      //Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;
      // Wait the appropriate time, call any background process while waiting...
      do { DoBackgroundProcess(); }
      while(millis() < lTimeWaitEnd);
      DebugWrite(A1, LOW);
      #ifdef DEBUG_X
      if(g_fDebugOutput) {
        DBGSerial.print("BRX:");
        DBGSerial.print(g_InControlState.BodyRot1.x,DEC); 
        DBGSerial.print("W?:");
        DBGSerial.print(fWalking,DEC);  
        DBGSerial.print(" GS:");
        DBGSerial.print(g_InControlState.GaitStep,DEC);  
        //Debug LF leg
        DBGSerial.print(" GPZ:");
        DBGSerial.print(GaitPosZ[cLF],DEC);
        DBGSerial.print(" GPY:");
        DBGSerial.println(GaitPosY[cLF],DEC);
      }
      #endif
    }
    #ifdef DEBUG_X
    if (g_fDebugOutput) {
      DBGSerial.print("TY:");
      DBGSerial.print(TotalYBal1,DEC); 
      DBGSerial.print(" LFZ:");
      DBGSerial.println(LegPosZ[cLF],DEC);
      DBGSerial.flush();  // see if forcing it to output helps...
    }
    #endif
    // Only do commit if we are actually doing something...
    g_ServoDriver.CommitServoDriver(ServoMoveTime);
  }
  else {
    //Turn the bot off - May need to add ajust here...
    if (g_InControlState.fPrev_RobotOn || (AllDown= 0)) {
      ServoMoveTime = 600;
      StartUpdateServos();
      g_ServoDriver.CommitServoDriver(ServoMoveTime);
      MSound(3, 100, 2500, 80, 2250, 60, 2000);  
      lTimeWaitEnd = millis() + 600;    // setup to process background stuff while we wait...
      do {
        // Wait the appropriate time, call any background process while waiting...
        DoBackgroundProcess();
      } 
      while (millis() < lTimeWaitEnd);
      //delay(600);
    } 
    else {
      g_ServoDriver.FreeServos();
      Eyes = 0;
    }

    // Allow the Servo driver to do stuff durint our idle time
    g_ServoDriver.IdleTime();

    delay(20);  // give a pause between times we call if nothing is happening
  }

  PrevServoMoveTime = ServoMoveTime;

  // Store previous g_InControlState.fRobotOn State
  if (g_InControlState.fRobotOn) g_InControlState.fPrev_RobotOn = 1;
  else g_InControlState.fPrev_RobotOn = 0;

  if(millis() - lifePrevMillis >= 10000) {
    printlnDebug("=== It works!!! ===");
    lifePrevMillis = millis();
  }
  
}

