// Copyright (2023)
// Control Freaks
// Vicksburg High School
// Michigan
// this is a time based program
//

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
//import e//du.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Robot extends TimedRobot {

   //private Joystick m_leftStick;
   //private Joystick m_rightStick;
   private final XboxController m_driverController = new XboxController(0);
   private final XboxController m_armController = new XboxController(1);

   private final DigitalInput m_armPivotLimitSwitch = new DigitalInput(1);
   private final DigitalInput m_armWinchLimitSwitch = new DigitalInput(2);
   private final DigitalOutput m_pincher = new DigitalOutput(0);

   CANCoder frontRightEncoder = new CANCoder(1);
   CANCoder frontLeftEncoder = new CANCoder(0);
   CANCoder backRightEncoder = new CANCoder(2);
   CANCoder backLeftEncoder = new CANCoder(3);

   //private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  // private final Compressor m_compressor = new Compressor(12, PneumaticsModuleType.REVPH);

   private boolean timeInit;
   private int counter;
   private int pitchCounter;
   private double TIME_STEP;
   private double distance;
   private double distanceInit;
   private double flcmd = 0;
   private double frcmd = 0;
   private double blcmd = 0;
   private double brcmd = 0;
   private double prevDesiredAngle;
   private double prevGain;
   private double prevDrCmd;

   private double backLeftSpinAngle;
   private double backRightSpinAngle;
   private double frontLeftSpinAngle;
   private double frontRightSpinAngle;

   private double frontRightSpinErrorSum;
   private double backRightSpinErrorSum;
   private double frontLeftSpinErrorSum;
   private double backLeftSpinErrorSum;

   private double frontRightSpinError; 
   private double backRightSpinError;
   private double frontLeftSpinError;
   private double backLeftSpinError;

   private double armPivotPower;
   private double armWinchPower;  

   private double backLeftDriveGain;
   private double backRightDriveGain;
   private double frontLeftDriveGain;
   private double frontRightDriveGain;

   public double yawFlipError;
   public double yawFlipErrorSum;
   public double prevYawFlipError;

   // Arm variables
   public double prevArmTarget; 
   public double prevWinchTarget; 
   public double armPivotError;
   public double prevArmPivotError;
   private double currentWinchPosition; 
   private double currentWinchSpeed;
   //private double filteredWinchTarget; 
   private double  armPivotLimitPosition;   
   private boolean armPivotLimitSwitch;
   private double  armWinchLimitPosition;
   private boolean armWinchLimitSwitch;

   private double currentArmPosition;
   private double desiredPivotTarget; 
   private double desiredWinchTarget; 

   private int state; 
   float pitch;
   float yaw;
   int debug_timer;
   public int stateCounter;

   BooleanLogEntry myBooleanLog;
   DoubleLogEntry myDoubleLog;
   StringLogEntry myStringLog;

   AHRS gyro = new AHRS(SPI.Port.kMXP);

   private final CANSparkMax m_frontLeftDrive  = new CANSparkMax(1, MotorType.kBrushless);
   private final CANSparkMax m_frontRightDrive = new CANSparkMax(3, MotorType.kBrushless);
   private final CANSparkMax m_backLeftDrive   = new CANSparkMax(5, MotorType.kBrushless);
   private final CANSparkMax m_backRightDrive  = new CANSparkMax(7, MotorType.kBrushless);

   private final CANSparkMax m_frontLeftSteer  = new CANSparkMax(2,MotorType.kBrushless);
   private final CANSparkMax m_frontRightSteer = new CANSparkMax(4, MotorType.kBrushless);
   private final CANSparkMax m_backLeftSteer   = new CANSparkMax(6,MotorType.kBrushless);
   private final CANSparkMax m_backRightSteer  = new CANSparkMax(8, MotorType.kBrushless);

   private final CANSparkMax m_armPivot = new CANSparkMax(9,MotorType.kBrushless);
   private final CANSparkMax m_armWinch = new CANSparkMax(10,MotorType.kBrushless); 

   private RelativeEncoder e_frontLeftDrive;
   private RelativeEncoder e_frontRightDrive;
   private RelativeEncoder e_backLeftDrive;
   private RelativeEncoder e_backRightDrive;

   private RelativeEncoder e_armPivot;
   private RelativeEncoder e_armWinch;

   @Override
   public void robotInit() {
      
      TIME_STEP = 0.02;  // 50 hertz
    //  m_compressor.enableDigital();
      //m_solenoid.set(true);
      
      m_frontLeftDrive.setInverted(false);
      m_frontRightDrive.setInverted(false);
      m_backLeftDrive.setInverted(false);  
      m_backRightDrive.setInverted(false);  
      //e_armPivot.setPosition(newPosition: 0);
      //e_armWinch.setPosition(0);
      frontLeftEncoder.setPosition(0);
      frontRightEncoder.setPosition(0);
      backLeftEncoder.setPosition(0);
      backRightEncoder.setPosition(0);
      m_armPivot.setInverted(false); 
      m_armWinch.setInverted(false);   
      
      m_pincher.set(false);

      frontRightSpinErrorSum = 0;
      backRightSpinErrorSum = 0;
      frontLeftSpinErrorSum = 0;
      backLeftSpinErrorSum = 0;

      frontRightSpinError = 0;
      backRightSpinError = 0;
      frontLeftSpinError = 0;
      backLeftSpinError = 0;

      yawFlipError = 0.0;
      yawFlipErrorSum = 0.0;
      prevYawFlipError = 0.0;
      armPivotLimitPosition = 0; 
      armWinchLimitPosition = 0;
      prevWinchTarget = 0;
      timeInit = false;
      counter = 0;
      pitchCounter = 0;
      distance = 0;
      pitch = 0;
      yaw = 0;
      prevDesiredAngle = 0;
      prevGain = 0;
      stateCounter = 0;

      prevDrCmd = 0;
      prevArmTarget = 0; 
      prevArmPivotError = 0; 
      gyro.calibrate();

      debug_timer = 0;
	  
	   e_frontLeftDrive  = m_frontLeftDrive.getEncoder();
      e_frontRightDrive = m_frontRightDrive.getEncoder();
      e_backLeftDrive   = m_backLeftDrive.getEncoder();
      e_backRightDrive  = m_backRightDrive.getEncoder();
      e_armPivot        = m_armPivot.getEncoder();
      e_armWinch        = m_armWinch.getEncoder();

      // Starts recording to data log
      DataLogManager.start();
      DataLog log = DataLogManager.getLog();
      myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
      myDoubleLog = new DoubleLogEntry(log, "/my/double");
      myStringLog = new StringLogEntry(log, "/my/string");
   }

   @Override
   public void autonomousInit() {
      state = 0;
      timeInit = false;
   }

   @Override
   public void autonomousPeriodic() {
      

      yaw = gyro.getYaw();
      currentArmPosition = e_armPivot.getPosition();
      pitch =   gyro.getPitch();
      armPivotLimitSwitch = m_armPivotLimitSwitch.get();
      armWinchLimitSwitch = m_armWinchLimitSwitch.get();
      currentArmPosition = e_armPivot.getPosition();
      currentWinchPosition = e_armWinch.getPosition();

      switch(state) {
         case 0: // initalize
            prevArmTarget = currentArmPosition;
            prevWinchTarget = currentWinchPosition;
            m_pincher.set(false); 
            armWinchLimitPosition = 0.0;
            stateCounter = 0;
            debug_timer = 0;
            //state = 33; //debug only
            state++;
         break;
         case 1: //crank in winch to calibrate position
            desiredWinchTarget = determineWinchTarget(false, false, false, false, prevWinchTarget, currentWinchPosition, -1,armWinchLimitSwitch, armWinchLimitPosition);
            prevWinchTarget = desiredWinchTarget; 
            armWinchPower = determineWinchPower(desiredWinchTarget,currentWinchPosition,armWinchLimitSwitch); 
            m_armWinch.set(armWinchPower);
            stateCounter++;
            if (armWinchLimitSwitch || stateCounter > 300) {
               //m_armWinch.set(0.0);
               armWinchLimitPosition = currentWinchPosition;
               state++;
               stateCounter = 0;
            }
         break;
         case 2:  //go down just a little
            desiredWinchTarget = determineWinchTarget(false, false, false, false, prevWinchTarget, currentWinchPosition, 1,armWinchLimitSwitch, armWinchLimitPosition);
            armWinchPower = determineWinchPower(desiredWinchTarget,currentWinchPosition,armWinchLimitSwitch); 
            m_armWinch.set(armWinchPower);
            stateCounter++;
            if (stateCounter > 25) {
               m_armWinch.set(0.0);
               state++;
               stateCounter = 0;
            }
         break;
         case 3: 


               stateCounter++;
               if (stateCounter > 100) {
                  state++;
                  stateCounter = 0;
               }

         break;
         case 4: //move arm pivot to the limit switch to calibrate
            if (armPivotLimitSwitch) {
               armPivotLimitPosition = currentArmPosition;
               m_pincher.set(true); 
               stateCounter++;
               if (stateCounter > 10) {
                  state++;
                  stateCounter = 0;
               }
            }
            desiredPivotTarget = determinePivotTarget(false, false, false, false, prevArmTarget, currentArmPosition, -0.523, armPivotLimitSwitch, armPivotLimitPosition);
            prevArmTarget = desiredPivotTarget; 
            armPivotError = desiredPivotTarget-currentArmPosition;
            armPivotPower = determinePivotPower(armPivotError,prevArmPivotError);
            m_armPivot.set(armPivotPower);
            prevArmPivotError = armPivotError;
         break;
         case 5: //get arm to position
            desiredPivotTarget = determinePivotTarget(false, false, true, false, prevArmTarget, currentArmPosition, 0.0, armPivotLimitSwitch, armPivotLimitPosition);
            prevArmTarget = desiredPivotTarget; 
            armPivotError = desiredPivotTarget-currentArmPosition;
            armPivotPower = determinePivotPower(armPivotError,prevArmPivotError);
            m_armPivot.set(armPivotPower);
            prevArmPivotError = armPivotError;
            if (Math.abs(armPivotError) < 1.0) {
               stateCounter++;
               if (stateCounter > 10) {
                  state++;
                  stateCounter = 0;
               }
            }
         break;
         case 6:
            desiredWinchTarget = determineWinchTarget(false, false, false, true, prevWinchTarget, currentWinchPosition, 0,armWinchLimitSwitch, armWinchLimitPosition);
            prevWinchTarget = desiredWinchTarget; 
            armWinchPower = determineWinchPower(desiredWinchTarget,currentWinchPosition,armWinchLimitSwitch); 
            m_armWinch.set(armWinchPower);
            if (Math.abs(desiredWinchTarget - currentWinchPosition) < 2)
            {
               m_armWinch.set(0);
               state++;
            }
         break;
         case 7:
            m_pincher.set(false);
            stateCounter++;
            if (stateCounter > 100) {
               state++;
               stateCounter = 0;
            }
         break;
         case 8: 
            if(driveForwardInches(140, 0.35)) {
               state++;
               m_armPivot.set(0);
               
               //state++; //jump to state 2 
            }
         break;
         case 11:
            if(balancePitch(0.08)) {
               state++;
               pitchCounter = 0;
            }
         break;
         case 24:
         pitchCounter++;
            if(pitchCounter > 250) {
               //state++;
            }
         break; 
         case 33: 
         if(driveForwardInches(60, 0.35)) {
            state++;
            prevDrCmd = 0;
            //state++; //jump to state 2 
         }
      break;
      }
      if (debug_timer > 9)
      {
         //myBooleanLog.append(armPivotLimitSwitch);
         myDoubleLog.append(currentWinchPosition);
         myDoubleLog.append(Double.valueOf(state));
         //myDoubleLog.append(armWinchPower);
         //myDoubleLog.append(armWinchLimitPosition);
         //myDoubleLog.append(e_frontLeftDrive.getPosition());
         //myDoubleLog.append(e_frontRightDrive.getPosition());
         //myDoubleLog.append(e_backLeftDrive.getPosition());
         //myDoubleLog.append(e_backRightDrive.getPosition());
         //myDoubleLog.append(frontRightEncoder.getPosition());
         //myDoubleLog.append(brcmd);
         debug_timer = 0;
      } else {
         debug_timer = debug_timer +1;
      }
   }

   @Override
   public void teleopPeriodic() {
      double orientation = 1;
      double gain;
      double drCmd;
      
      double desiredAngle;
      double desiredAngleFlipped;
      double filteredAngle;
      double joyXPos;
      double joyYPos;
      double joyWinch;
      double joyPivot;
      boolean leftBumper;
      boolean rightBumper;
      double d_yaw;
      double other_yaw;
      double gainTgt;
      double togglePad;
      boolean yawCompensationEnable = false;
      boolean buttonA;
      boolean buttonB;
      boolean buttonX;
      boolean buttonY; 
      double turnGain;
      double desiredYaw;
      double currentAngle;
      int debug;
      debug = 0;

      boolean button2A; 
      boolean button2B;
      boolean button2Y;
      boolean button2X; 
      boolean left2Bumper;
      boolean right2Bumper;
      double leftTrigger;
      double rightTrigger;
      
      double kp_yaw;
      double ki_yaw;
      double kd_yaw;

      armPivotLimitSwitch = m_armPivotLimitSwitch.get();
      armWinchLimitSwitch = m_armWinchLimitSwitch.get();

      //Game Pad 1 - Driver
      joyXPos = m_driverController.getRightX();//m_leftStick.getX();
      joyYPos = m_driverController.getRightY();//m_leftStick.getY();
      leftBumper = m_driverController.getLeftBumper();
      rightBumper = m_driverController.getRightBumper();
      togglePad = m_driverController.getLeftX();
      buttonA = m_driverController.getAButton();
      buttonB = m_driverController.getBButton();
      buttonX = m_driverController.getXButton();
      buttonY = m_driverController.getYButton();
      leftTrigger = m_driverController.getLeftTriggerAxis();
      rightTrigger = m_driverController.getRightTriggerAxis();

	  //Game Pad 2 - Arm control
      joyWinch = -m_armController.getLeftY();
      joyPivot = -m_armController.getRightY();
      button2A = m_armController.getAButton();
      button2B = m_armController.getBButton();
      button2Y = m_armController.getYButton();
      button2X = m_armController.getXButton();
      left2Bumper = m_armController.getLeftBumper();
      right2Bumper = m_armController.getRightBumper();

      desiredYaw = 0;
      turnGain = 0.0;
      blcmd = 0.0;
      flcmd = 0.0;
      frcmd = 0.0;
      brcmd = 0.0;
      drCmd = 0.0;
      yaw = gyro.getYaw();
      desiredAngle = prevDesiredAngle;

      //this gain is the drive wheel speed gain
      gainTgt = 0.25; 
      if (leftBumper) {
         gainTgt = 0.8;
      } else if (rightBumper) {
         gainTgt = 0.1;
      }
      gain = prevGain + (0.025*(gainTgt - prevGain));
      prevGain = gain;
      gain = gain * orientation;

      // if the driver joystick is not near zero, drive
      if ((Math.abs(joyXPos) > 0.09)||(Math.abs(joyYPos) > 0.09)){
         drCmd = (joyXPos*joyXPos)+(joyYPos*joyYPos);
         drCmd = gain * Math.sqrt(drCmd);
         // this is a large function that determines the desired
         //angle of the wheel
         desiredAngle = determineDesiredAngle(joyXPos,joyYPos);    
      }
  
      // these are offsets
      backLeftSpinAngle = 0;
      backRightSpinAngle =0;
      frontLeftSpinAngle =0;
      frontRightSpinAngle =0;

      // this robot tends to favor the right side motors
      //slow them down a little
      backLeftDriveGain =1;
      backRightDriveGain = 0.8;
      frontLeftDriveGain= 1;
      frontRightDriveGain = 0.8;
      
      // rotate the robot
      if ((togglePad) > 0.75){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = 0.5 * orientation * Math.abs(gain);
      }
      if ((togglePad) < -0.75){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = -0.5 * orientation * Math.abs(gain);
      }
      if (buttonA ) {
         desiredAngle = 0;
         backLeftSpinAngle = 1;
         backRightSpinAngle = 1;
         frontLeftSpinAngle =1;
         frontRightSpinAngle =1;
         backLeftDriveGain =-1;
         backRightDriveGain =-1;
         frontLeftDriveGain= -1;
         frontRightDriveGain =-1;
         if (leftBumper){
            if (prevDrCmd<.8){
               drCmd = prevDrCmd +.01;
            } else {
               drCmd=.8;
            }
         } else {
            if (prevDrCmd>.1){
               drCmd = prevDrCmd -.01;
            } else {
               drCmd=.1;
            }
         drCmd = .1;
         }
      }
      if (buttonB) {
         desiredAngle = 0;
         backLeftSpinAngle = 90;
         backRightSpinAngle = 90;
         frontLeftSpinAngle =90;
         frontRightSpinAngle =90;
         backLeftDriveGain =-1;
         backRightDriveGain =-1;
         frontLeftDriveGain= -1;
         frontRightDriveGain = -1;

         if (leftBumper){
            if (prevDrCmd<.8){
               drCmd = prevDrCmd +.01;
            } else {
               drCmd=.8;
            }
         } else {
            if (prevDrCmd>.1){
               drCmd = prevDrCmd -.01;
            } else {
               drCmd=.1;
            }
         drCmd = .1;
         }
     }
     if (buttonX) {
        desiredAngle = 0;
        backLeftSpinAngle = 90;
        backRightSpinAngle = 90;
        frontLeftSpinAngle =90;
        frontRightSpinAngle =90;
        backLeftDriveGain =1;
        backRightDriveGain =1;
        frontLeftDriveGain= 1;
        frontRightDriveGain = 1;
        if (leftBumper) {
           if (prevDrCmd < 0.8){
              drCmd = prevDrCmd +.01;
           } else {
              drCmd=.8;
           }
        } else {
           if (prevDrCmd > 0.1) {
              drCmd = prevDrCmd -.01;
           } else {
             drCmd=.1;
           }
           drCmd = .1;
        }
     }
     if (buttonY) {
        desiredAngle = 0;
        backLeftSpinAngle = 1;
        backRightSpinAngle = 1;
        frontLeftSpinAngle =1;
        frontRightSpinAngle =1;
        backLeftDriveGain =1;
        backRightDriveGain =1;
        frontLeftDriveGain= 1;
        frontRightDriveGain = 1;
        if (leftBumper){
           if (prevDrCmd<.8){
              drCmd = prevDrCmd +.01;
           } else {
              drCmd=.8;
           }
        } else {
           if (prevDrCmd>.1){
              drCmd = prevDrCmd -.01;
           } else {
              drCmd=.1;
           }
           drCmd = .1;
        }
     }
      if (m_driverController.getPOV()>45&&m_driverController.getPOV()<135){
         desiredAngle = 0;
         backLeftSpinAngle = -45;
         backRightSpinAngle = 45;
         frontLeftSpinAngle =45;
         frontRightSpinAngle =-45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = 0 * orientation;
      }

      //next two sections deal with rotating the robot 180 degrees
      // turning it around - yaw
      other_yaw = Double.valueOf(yaw);
      kp_yaw = 0.009;
      ki_yaw = 0.0001;
      kd_yaw = 0.0001;

      if (rightTrigger > 0.5){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;
         desiredYaw = 180.0;
         if (other_yaw < 0) {
            other_yaw = other_yaw + 360;
         }
         yawFlipError = desiredYaw-other_yaw;
         yawFlipErrorSum = yawFlipErrorSum + (yawFlipError * TIME_STEP);
         drCmd = (kp_yaw*yawFlipError)+(ki_yaw*yawFlipErrorSum)+(kd_yaw*(yawFlipError-prevYawFlipError));
         if (drCmd > 0.3) {drCmd = 0.3;}
         if (drCmd < -0.3) {drCmd = -0.3;}
         prevYawFlipError = yawFlipError;
       }
       else if (leftTrigger > 0.5) {
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;
         desiredYaw = 0.0;
         yawFlipError = desiredYaw-other_yaw;
         yawFlipErrorSum = yawFlipErrorSum + (yawFlipError * TIME_STEP);

         drCmd = (kp_yaw*yawFlipError)+(ki_yaw*yawFlipErrorSum)+(kd_yaw*(yawFlipError-prevYawFlipError));
         if (drCmd > 0.3) {drCmd = 0.3;}
         if (drCmd < -0.3) {drCmd = -0.3;}
         prevYawFlipError = yawFlipError;
      } else {
         yawFlipErrorSum = 0.0;
         prevYawFlipError = 0.0;
      }

      d_yaw = Double.valueOf(yaw);
      //ZERO YAW
      /*if (buttonB && (drCmd < 0.03)) {
         desiredAngle = 0;
         backLeftSpinAngle   =  45;
         backRightSpinAngle  = -45;
         frontLeftSpinAngle  = -45;
         frontRightSpinAngle =  45;
         backLeftDriveGain   =  1;
         backRightDriveGain  = -1;
         frontLeftDriveGain  =  1;
         frontRightDriveGain = -1;

         drCmd = (desiredYaw-d_yaw)*kp_yaw * orientation;
         if (drCmd > 0.4) {drCmd = 0.4;}
         if (drCmd < -0.4) {drCmd = -0.4;}
      }*/

      // yaw compensation logic, if driving straight, try to zero yaw
      // can be disabled if yawCompensationEnable is set to false
      //
      if ((rightTrigger < 0.4) && (leftTrigger < 0.4)) {
         if (d_yaw > 20) {d_yaw = 20;}
         if (d_yaw < -20) {d_yaw = -20;}
	      if ((-10 < desiredAngle) && (desiredAngle < 10) && (drCmd > 0.1)&&yawCompensationEnable) {
            if (yaw > 1) {
		         backRightDriveGain  = backRightDriveGain * (1 + (d_yaw*0.1));
		         frontRightDriveGain = frontRightDriveGain * (1 + (d_yaw*0.1));
		      }
         }
	      if (((-170 > desiredAngle) || (desiredAngle > 170)) && (drCmd > 0.1)&&yawCompensationEnable) {
	         if (yaw < -1) {
	            backRightDriveGain  = backRightDriveGain * (1 - (d_yaw*0.1));
   	         frontRightDriveGain = frontRightDriveGain * (1 - (d_yaw*0.1));
		      }
         }
      }
	  
      currentAngle = frontLeftEncoder.getPosition();
      desiredAngle  = angleWrap(currentAngle, desiredAngle);

      // Here is the logic that allows the wheels to decide whether 
      // to just spin backward or completely flip around
      if (desiredAngle >= -0.01) {
         desiredAngleFlipped = desiredAngle - 180;
         if (desiredAngleFlipped < -180) {
            desiredAngleFlipped = -180;
         } 
      } else {
         desiredAngleFlipped = desiredAngle + 180; 
      }
      if (Math.abs(frontLeftEncoder.getPosition() - desiredAngle)
        > Math.abs(frontLeftEncoder.getPosition() - desiredAngleFlipped)) 
      {
         desiredAngle = desiredAngleFlipped;
         backLeftDriveGain   = -backLeftDriveGain;
         backRightDriveGain  = -backRightDriveGain;
         frontLeftDriveGain  = -frontLeftDriveGain;
         frontRightDriveGain = -frontRightDriveGain;
         debug = 2;
      } else  {
         debug = 1;
      }

      // simple filter to smooth out angle change, this may have a minor bug
      filteredAngle = desiredAngle;//prevDesiredAngle + (0.01*(desiredAngle - prevDesiredAngle));

      //
      // wheel spin
      // error = desired - current
      //
      frontRightSpinError = filteredAngle+frontRightSpinAngle-frontRightEncoder.getPosition();
      backRightSpinError = filteredAngle+backRightSpinAngle-backRightEncoder.getPosition();
      frontLeftSpinError = filteredAngle+frontLeftSpinAngle-frontLeftEncoder.getPosition();
      backLeftSpinError = filteredAngle+backLeftSpinAngle-backLeftEncoder.getPosition();
     
      //
      // needed for I term which isn't essential for wheel spin control
      //
      frontRightSpinErrorSum = frontRightSpinErrorSum+frontRightSpinError*TIME_STEP;
      backRightSpinErrorSum = backRightSpinErrorSum+backRightSpinError*TIME_STEP;
      frontLeftSpinErrorSum = frontLeftSpinErrorSum+frontLeftSpinError*TIME_STEP;
      backLeftSpinErrorSum = backLeftSpinErrorSum+backLeftSpinError*TIME_STEP;

      // calculate the required motor power command to get to desired wheel angle
      flcmd = -wheelAngle(frontLeftSpinError,  frontLeftSpinErrorSum);
      frcmd =  wheelAngle(frontRightSpinError, frontRightSpinErrorSum);       
      blcmd = -wheelAngle(backLeftSpinError,   backLeftSpinErrorSum);
      brcmd = -wheelAngle(backRightSpinError,  backRightSpinErrorSum);
      
      prevDesiredAngle = filteredAngle;
      
      //
      //setting the arm pivot angle and winch position
      //
      currentArmPosition = e_armPivot.getPosition();
      currentWinchPosition = e_armWinch.getPosition();
      currentWinchSpeed = e_armWinch.getVelocity(); 

      if (armPivotLimitSwitch) {
         armPivotLimitPosition = currentArmPosition; 
      }
      desiredPivotTarget = determinePivotTarget(button2A, button2B, button2X, button2Y, prevArmTarget, currentWinchPosition, joyPivot, armPivotLimitSwitch, armPivotLimitPosition);
      prevArmTarget = desiredPivotTarget; 
      
      if (armWinchLimitSwitch) {
         armWinchLimitPosition = currentWinchPosition;
      }
      
      desiredWinchTarget = determineWinchTarget(button2A, button2B, button2X, button2Y, prevWinchTarget, currentWinchPosition, joyWinch,armWinchLimitSwitch, armWinchLimitPosition);
      prevWinchTarget = desiredWinchTarget; 
      
      //arm pivot logic uses a PD controller
      armPivotError = desiredPivotTarget-currentArmPosition;
      armPivotPower = determinePivotPower(armPivotError,prevArmPivotError);
      m_armPivot.set(armPivotPower);
      prevArmPivotError = armPivotError;

      //arm winch logic uses a P controller
      armWinchPower = determineWinchPower(desiredWinchTarget,currentWinchPosition,armWinchLimitSwitch); 
      m_armWinch.set(armWinchPower);

      //open or close the pneumatic grabber
      m_pincher.set(!right2Bumper);

      // sending the power commands to the driving and steering motors
      prevDrCmd = drCmd;
      m_frontLeftDrive.set(drCmd*frontLeftDriveGain);
      m_frontRightDrive.set(drCmd*frontRightDriveGain);
      m_backLeftDrive.set(drCmd*backLeftDriveGain);
      m_backRightDrive.set(drCmd*backRightDriveGain);
	  
      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);
      //e_frontLeftDrive.getPosition();
      //e_frontRightDrive.getPosition();
      //e_backLeftDrive.getPosition();
      //e_backRightDrive.getPosition();
      // telemtry information, feel free to change as needed
      if (debug_timer > 9)
      {
         myBooleanLog.append(armPivotLimitSwitch);
         //myDoubleLog.append(currentWinchPosition);
         //myDoubleLog.append(desiredWinchTarget);
         //myDoubleLog.append(armWinchPower);
         //myDoubleLog.append(armWinchLimitPosition);
         myDoubleLog.append(currentArmPosition);
         myDoubleLog.append(desiredPivotTarget);
         myDoubleLog.append(armPivotPower);
         //myDoubleLog.append(e_backRightDrive.getPosition());
         //myDoubleLog.append(frontRightEncoder.getPosition());
         //myDoubleLog.append(brcmd);
         debug_timer = 0;
      } else {
         debug_timer = debug_timer +1;
      }   
   }
//
// Drive Forward Inches
// used in autonomous mode
//
   public boolean driveForwardInches(double inches, double powerInput){
      boolean complete = false; 
      double driveYaw;
      double leftPwr;
      double rightPwr;
      double power = 0;
      
      rightPwr = 0.0;
      leftPwr = 0.0;
      driveYaw = Double.valueOf(yaw);

      distance = -e_frontLeftDrive.getPosition();
      if (timeInit == false)  {
         distanceInit = distance;
         timeInit = true;
      }
      //
      // wheel spin
      // error = desired - current
      //
      // target is 0
      frontRightSpinError = 0 - frontRightEncoder.getPosition();
      backRightSpinError  = 0 - backRightEncoder.getPosition();
      frontLeftSpinError  = 0 - frontLeftEncoder.getPosition();
      backLeftSpinError   = 0 - backLeftEncoder.getPosition();
     
      //
      // needed for I term which isn't essential for wheel spin control
      //
      frontRightSpinErrorSum = 0.0;//frontRightSpinErrorSum+frontRightSpinError*TIME_STEP;
      backRightSpinErrorSum = 0.0;//backRightSpinErrorSum+backRightSpinError*TIME_STEP;
      frontLeftSpinErrorSum = 0.0;//frontLeftSpinErrorSum+frontLeftSpinError*TIME_STEP;
      backLeftSpinErrorSum = 0.0;//backLeftSpinErrorSum+backLeftSpinError*TIME_STEP;

      // calculate the required motor power command to get to desired wheel angle
      flcmd = -wheelAngle(frontLeftSpinError,  frontLeftSpinErrorSum);
      frcmd =  wheelAngle(frontRightSpinError, frontRightSpinErrorSum);       
      blcmd = -wheelAngle(backLeftSpinError,   backLeftSpinErrorSum);
      brcmd = -wheelAngle(backRightSpinError,  backRightSpinErrorSum);
      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);


      if ((distance - distanceInit) < (inches*0.56)) {
         power = prevDrCmd + (0.1*(powerInput-prevDrCmd));
         if (driveYaw > 2) {
            rightPwr = 0.025*(driveYaw);
         } else if (driveYaw<-2){
            leftPwr = -0.025*(driveYaw);
         }
         // this robot tends to favor the right side motors
         //slow them down a little
         backLeftDriveGain =1;
         backRightDriveGain = 1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = 1;

         flcmd = -frontLeftDriveGain*power+leftPwr;
         frcmd = -frontRightDriveGain*power+rightPwr;
         blcmd = -backLeftDriveGain*power+leftPwr;
         brcmd = -backRightDriveGain*power+rightPwr;
         prevDrCmd = power;

         m_frontLeftDrive.set(flcmd);
         m_frontRightDrive.set(frcmd);
         m_backLeftDrive.set(blcmd);
         m_backRightDrive.set(brcmd);
      } else {
         complete = true; 
         timeInit = false; 
         m_frontLeftDrive.set(0.0);
         m_frontRightDrive.set(0.0);
         m_backLeftDrive.set(0.0);
         m_backRightDrive.set(0.0);
      }
     // telemtry
     if (counter > 9)
     {
        myDoubleLog.append(rightPwr);
        myDoubleLog.append(leftPwr);
        myDoubleLog.append(driveYaw);
        counter = 0;
     } else {
        counter = counter +1;
     }
     return(complete);
   }

   //
   // Balance Pitch
   // used in autonomous mode to balance on the charging station
   // specific to the 2023 FRC game
   //
   public boolean balancePitch(double power){
      boolean complete = false; 
      double blcmd;
      double brcmd;
      double flcmd;
      double frcmd;
    
      if (counter > 9)
      {
         myBooleanLog.append(true);
         myDoubleLog.append(pitch);
         myStringLog.append("pitch");
         counter = 0;
      } else {
         counter = counter +1;
      }
      
      if (timeInit == false)  {
         timeInit = true;
         pitchCounter = 0;
      }
    
      if ((pitch) > (5.0)) {
         flcmd = -power;
         frcmd = -power;
         blcmd = -power;
         brcmd = -power;
         m_frontLeftDrive.set(flcmd);
         m_frontRightDrive.set(frcmd);
         m_backLeftDrive.set(blcmd);
         m_backRightDrive.set(brcmd);
      } else if(pitch < (-5.0)) {
         flcmd = power*0.75;
         frcmd = power*0.75;
         blcmd = power*0.75;
         brcmd = power*0.75;
         m_frontLeftDrive.set(flcmd);
         m_frontRightDrive.set(frcmd);
         m_backLeftDrive.set(blcmd);
         m_backRightDrive.set(brcmd);
      } else {
         m_frontLeftDrive.set(0.0);
         m_frontRightDrive.set(0.0);
         m_backLeftDrive.set(0.0);
         m_backRightDrive.set(0.0);  
         pitchCounter++;
         if (pitchCounter > 100) {
            complete = true; 
            timeInit = false; 
         }
      }
      return(complete);
   }
   //
   // Wheel Angle - PI control of wheel angle
   // used in teleop mode
   // outputs the power command for the steer wheel
   //
   public double wheelAngle(double error, double errorSum)
   {
      double powerCmd;
      double kp = 0.008;
      double ki = 0.00;
      double maxPwr = 0.5;

      powerCmd = (kp * error) + (ki * errorSum);

      if (powerCmd > maxPwr){powerCmd = maxPwr;}
      if (powerCmd < -maxPwr){powerCmd = -maxPwr;}

      return(powerCmd);
   }
//
// Angle Angle - attempts to set the desired gear in the same
// 0-360 or 360-720 etc range
// used in teleop mode
// NOTE:  THIS COULD BE IMPROVED
//
   public double angleWrap(double current, double desired)
   {
      if ((current < -450)) {
         if (desired > 90) {
            return(desired - 720);
         }
      }
      if ((current > 450)) {
         if (desired < -90) {
            return(desired + 720);
         }
      }
      if ((current > 90)) {
         if (desired < -90) {
            return(desired + 360);
         }
      }
      if ((current < -90)) {
         if (desired > 90) {
            return(desired - 360);
         }
      }
      return(desired);
   }

   // determine the desired position of the arm pivot
   public static double determinePivotTarget(boolean a,boolean b,boolean x, boolean y, double prevTgt, double winchPosition, double joyX, boolean limitSwitch, double zeroPosition)
   {
      double target; 
      double gain = 1;
      target = prevTgt; 
      if (a) {
         if (winchPosition < 5) {
            target = zeroPosition; 
         } else {
            target = zeroPosition+10;
         }
      }

      if (b) {
         if (winchPosition < 5) {
            target = zeroPosition+4; 
         } else {
            target = zeroPosition+10;
         }
      }

      if (y) {
         target = zeroPosition+21;
      }

      if (x) {
         target = zeroPosition+22; 
      }
      if (joyX > 0.2) {
         target = target + (joyX*gain);
      }
      if (joyX < -0.2) {
         if (!limitSwitch) {
            target = target + (joyX*gain);
         }
         //if (target < 0) {target = 0;}
      }
      
      return (prevTgt+ (0.25*(target - prevTgt)));
   }
   
   public static double determineWinchTarget(boolean a,boolean b,boolean x, boolean y, double prevTgt, double winchPosition, double joyY, boolean limitSwitch, double zeroPosition)
   {
      double target; 
      double gain = 3;
      target = prevTgt; 
      if (a) {
         target = zeroPosition; 
      }

      if (b) {
         target = zeroPosition+40;
      }

      if (y) {
         target = zeroPosition+45;
      }

      if (x) {
         target = zeroPosition+90; 
      }
      if (joyY > 0.2) {
         target = target + (joyY*gain);
      }
      if (joyY < -0.2) {
         if (!limitSwitch) {
            target = target + (joyY*gain);
         } else {
            target = winchPosition;
         }
         //if (target < 0) {target = 0;}
      }
      return(prevTgt+ (0.25*(target - prevTgt)));
      //   return 
   }

   public static double determineWinchPower(double desiredPos, double currentPos, boolean limitSwitch) {
      double Kp_winch = 0.05;
      double winchPower;
      double maxDesiredPosition = 140;

      if (desiredPos > maxDesiredPosition) {desiredPos = maxDesiredPosition;}

      winchPower = Kp_winch * (desiredPos-currentPos);

      if (limitSwitch && (winchPower < 0.0)) {
         winchPower = 0.01;
      } else {

      }
      if (winchPower > 0.5)  {winchPower = 0.5;} //positive is in
      if (winchPower < -0.5) {winchPower = -0.5;}  //negative is out
      return(winchPower); 
   }

   public static double determinePivotPower(double error, double prevError) {
      double Kp_arm = 0.030;//0.035
      double Kd_arm = 0.085;//0.065

      return(Kp_arm*(error) + Kd_arm*(error-prevError));
   }
   
   public static double determineDesiredAngle(double joyX, double joyY) {   
      if ((joyX > 0.1)&&(joyY < -0.1)) {
         return(-57.3*Math.atan(Math.abs(joyY/joyX)));
      }
      if ((joyX > 0.1)&&(Math.abs(joyY) < 0.1)) {
         return(-90);
      }
      if ((joyX < -0.1)&&(Math.abs(joyY) < 0.1)) {
         return(90);
      }
      if ((Math.abs(joyX) < 0.1)&&(joyY < -0.1)) {
         return(0.1);
      }
      if ((Math.abs(joyX) < 0.1)&&(joyY > 0.1)) {
         return(180);
      }
      if ((joyX > 0.1)&&(joyY > 0.1)) {
         return(-180+57.3*Math.atan(Math.abs(joyY/joyX)));
      }
      if ((joyX < -0.1)&&(joyY < -0.1)) {
         return(57.3*Math.atan(Math.abs(joyY/joyX)));
      }
      if ((joyX < -0.1)&&(joyY > 0.1)) {
         return(180-57.3*Math.atan(Math.abs(joyY/joyX)));
      }
      return(0.1);
   }
} //END




  /* public static double closestAngle(double a, double b)
   {
        // get direction
        double dir = (b * 360.0) - (a * 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
   }
   
   
   public void setDirection(double setpoint)
{
    directionController.reset();

    // use the fastest way
    double currentAngle = directionSensor.get();
    directionController.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));

    directionController.enable();
}
*/