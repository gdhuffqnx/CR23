// Copyright (2023)
// Control Freaks
// Vicksburg High School
// Michigan

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
import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Robot extends TimedRobot {

   //private Joystick m_leftStick;
   //private Joystick m_rightStick;
   private final XboxController m_driverController = new XboxController(0);
   private final XboxController m_armController = new XboxController(1);


   private final DigitalOutput m_digital = new DigitalOutput(0);

   CANCoder frontRightEncoder = new CANCoder(1);
   CANCoder frontLeftEncoder = new CANCoder(0);
   CANCoder backRightEncoder = new CANCoder(2);
   CANCoder backLeftEncoder = new CANCoder(3);

   private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
   private final Compressor m_compressor = new Compressor(12, PneumaticsModuleType.REVPH);

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
   private double prevArmWinch; 
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
   private double Kp_arm;
   private double Kd_arm; 

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
   
   private int state; 
   float pitch;
   float yaw;
   private int angleCounter;
   int debug_timer;

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
      m_compressor.enableDigital();
      //m_solenoid.set(true);
      
      m_frontLeftDrive.setInverted(false);
      m_frontRightDrive.setInverted(false);
      m_backLeftDrive.setInverted(false);  
      m_backRightDrive.setInverted(false);  
      frontLeftEncoder.setPosition(0);
      frontRightEncoder.setPosition(0);
      backLeftEncoder.setPosition(0);
      backRightEncoder.setPosition(0);
      m_armPivot.setInverted(false); 
      m_armWinch.setInverted(false);   

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

      timeInit = false;
      counter = 0;
      pitchCounter = 0;
      distance = 0;
      pitch = 0;
      yaw = 0;
      prevDesiredAngle = 0;
      prevGain = 0;
      angleCounter = 0;
      prevDrCmd = 0;
      prevArmTarget = 0; 
      prevArmWinch = 0;
      prevArmPivotError = 0; 
      gyro.calibrate();

      debug_timer = 0;

      Kp_arm = 0.035; 
      Kd_arm = 0.065; 
	  
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
      pitch =   gyro.getPitch();
      switch(state) {
         case 0: 
            if(driveForwardInches(15, 0.1)) {
               state++;
               
               //state++; //jump to state 2 
            }
         break;
         case 1:
            if(balancePitch(0.08)) {
               state++;
               pitchCounter = 0;
            }
         break;
         case 2:
         pitchCounter++;
            if(pitchCounter > 250) {
               //state++;
            }
         break; 
         case 33: 
         if(driveForwardInches(48, -0.25)) {
            state++;
            //state++; //jump to state 2 
         }
      break;
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
      boolean leftBumper;
      boolean rightBumper;
      double d_yaw;
      double other_yaw;
      double gainTgt;
      double togglePad;
      boolean yawCompensationEnable = true;
      boolean buttonA;
      boolean buttonB;
      boolean buttonY; 
      double turnGain;
      double desiredYaw;
      double desiredArmTarget; 
      double desiredWinchTarget; 
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
      
      

      //Game Pad 1 - Driver
      joyXPos = m_driverController.getRightX();//m_leftStick.getX();
      joyYPos = m_driverController.getRightY();//m_leftStick.getY();
      leftBumper = m_driverController.getLeftBumper();
      rightBumper = m_driverController.getRightBumper();
      togglePad = m_driverController.getLeftX();
      buttonA = m_driverController.getAButton();
      buttonB = m_driverController.getBButton();
      buttonY = m_driverController.getYButton();
      leftTrigger = m_driverController.getLeftTriggerAxis();
      rightTrigger = m_driverController.getRightTriggerAxis();

	  //Game Pad 2 - Arm control
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

      

      //setting the arm angle and stuff
      currentWinchPosition = e_armWinch.getPosition(); 

      desiredArmTarget = armTarget(button2A, button2B, button2X, button2Y, prevArmTarget, currentWinchPosition);
      prevArmTarget = desiredArmTarget; 

      desiredWinchTarget = winchTarget(button2A, button2B, button2X, button2Y, prevArmTarget, currentWinchPosition);
      prevWinchTarget = desiredWinchTarget; 

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

      if ((Math.abs(joyXPos) > 0.09)||(Math.abs(joyYPos) > 0.09)){
         drCmd = (joyXPos*joyXPos)+(joyYPos*joyYPos);
         drCmd = gain * Math.sqrt(drCmd);
         desiredAngle = determineDesiredAngle(joyXPos,joyYPos);    
      }
  
      backLeftSpinAngle = 0;
      backRightSpinAngle =0;
      frontLeftSpinAngle =0;
      frontRightSpinAngle =0;
      backLeftDriveGain =1;
      backRightDriveGain =0.9;
      frontLeftDriveGain= 1;
      frontRightDriveGain = 0.9;
      
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

      if (buttonA && (drCmd < 0.03)) {
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

      //next two sections deal with rotating the robot
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
         if (drCmd > 0.4) {drCmd = 0.4;}
         if (drCmd < -0.4) {drCmd = -0.4;}
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
         if (drCmd > 0.4) {drCmd = 0.4;}
         if (drCmd < -0.4) {drCmd = -0.4;}
         prevYawFlipError = yawFlipError;
      } else {
         yawFlipErrorSum = 0.0;
         prevYawFlipError = 0.0;
      }

      d_yaw = Double.valueOf(yaw);
      //ZERO YAW
      if (buttonB && (drCmd < 0.03)) {
         desiredAngle = 0;
         backLeftSpinAngle   =  45;
         backRightSpinAngle  = -45;
         frontLeftSpinAngle  = -45;
         frontRightSpinAngle =  45;
         backLeftDriveGain   =  1;
         backRightDriveGain  = -1;
         frontLeftDriveGain  =  1;
         frontRightDriveGain = -1;

         drCmd = (desiredYaw-d_yaw)*kp_yaw* orientation;
         if (drCmd > 0.4) {drCmd = 0.4;}
         if (drCmd < -0.4) {drCmd = -0.4;}
      }

      // yaw compensation logic, if driving straight, try to zero yaw
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

      filteredAngle = prevDesiredAngle + (0.1*(desiredAngle - prevDesiredAngle));

      //error =desired - current
      frontRightSpinError = filteredAngle+frontRightSpinAngle-frontRightEncoder.getPosition();
      backRightSpinError = filteredAngle+backRightSpinAngle-backRightEncoder.getPosition();
      frontLeftSpinError = filteredAngle+frontLeftSpinAngle-frontLeftEncoder.getPosition();
      backLeftSpinError = filteredAngle+backLeftSpinAngle-backLeftEncoder.getPosition();
      
      frontRightSpinErrorSum = frontRightSpinErrorSum+frontRightSpinError*TIME_STEP;
      backRightSpinErrorSum = backRightSpinErrorSum+backRightSpinError*TIME_STEP;
      frontLeftSpinErrorSum = frontLeftSpinErrorSum+frontLeftSpinError*TIME_STEP;
      backLeftSpinErrorSum = backLeftSpinErrorSum+backLeftSpinError*TIME_STEP;

      flcmd = -wheelAngle(frontLeftSpinError,  frontLeftSpinErrorSum);
      frcmd =  wheelAngle(frontRightSpinError, frontRightSpinErrorSum);       
      blcmd = -wheelAngle(backLeftSpinError,   backLeftSpinErrorSum);
      brcmd = -wheelAngle(backRightSpinError,  backRightSpinErrorSum);
      
      prevDesiredAngle = filteredAngle;

      if (debug_timer > 4)
      {
         myBooleanLog.append(right2Bumper);
         //myDoubleLog.append(rightTrigger);
         //myDoubleLog.append(frontRightSpinAngle);
         //myDoubleLog.append(frontRightEncoder.getPosition());
         //myDoubleLog.append(frcmd);
         //myDoubleLog.append(backRightSpinAngle);
         //myDoubleLog.append(backRightEncoder.getPosition());
         //myDoubleLog.append(brcmd);
         debug_timer = 0;
      } else {
         debug_timer = debug_timer +1;
      }   
      
      //arm logic
      armPivotError = desiredArmTarget-e_armPivot.getPosition();
      armPivotPower = Kp_arm*(armPivotError) + Kd_arm*(armPivotError-prevArmPivotError);
      m_armPivot.set(armPivotPower);
      prevArmPivotError = armPivotError;

if (right2Bumper) {
      m_digital.set(true);
   } else {
   m_digital.set(false);
}
      //arm winch logic
      double Kp_winch = 0.1;

      armWinchPower = Kp_winch * (desiredWinchTarget-currentWinchPosition);
      m_armWinch.set(armWinchPower);

      prevDrCmd = drCmd;
      m_frontLeftDrive.set(drCmd*frontLeftDriveGain);
      m_frontRightDrive.set(drCmd*frontRightDriveGain);
      m_backLeftDrive.set(drCmd*backLeftDriveGain);
      m_backRightDrive.set(drCmd*backRightDriveGain);
	  
      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);
   }
//
// Drive Forward Inches
// used in autonomous mode
//
   public boolean driveForwardInches(double inches, double power){
      boolean complete = false; 
      double driveYaw;
      double leftPwr;
      double rightPwr;
      rightPwr = 0.0;
      leftPwr = 0.0;
      driveYaw = Double.valueOf(yaw);


      distance = -e_frontLeftDrive.getPosition();
      if (timeInit == false)  {
         distanceInit = distance;
         timeInit = true;
      }
    
      if ((distance - distanceInit) < (inches*0.56)) {
         if (driveYaw > 0) {
            rightPwr = 0.0009*(driveYaw);
         } else {
            leftPwr = -0.0009*(driveYaw);
         }

         flcmd = -power+leftPwr;
         frcmd = -power+rightPwr;
         blcmd = -power+leftPwr;
         brcmd = -power+rightPwr;

         m_frontLeftDrive.set(flcmd);
         m_frontRightDrive.set(frcmd);
         m_backLeftDrive.set(blcmd);
         m_backRightDrive.set(brcmd);
         
     } else {
         m_frontLeftDrive.set(0.0);
         m_frontRightDrive.set(0.0);
         m_backLeftDrive.set(0.0);
         m_backRightDrive.set(0.0);  
         complete = true; 
         timeInit = false; 
     }
	 
     m_frontLeftSteer.set(0.0);
     m_frontRightSteer.set(0.0);
     m_backLeftSteer.set(0.0);
     m_backRightSteer.set(0.0);
	  
     if (counter > 9)
     {
        myDoubleLog.append(rightPwr);
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
   // Wheel Angle - Proporational control of wheel angle
   // used in teleop mode
   // outputs the power command for the steer wheel
   //
   public double wheelAngle(double error, double errorSum)
   {
      double powerCmd;
      double kp = 0.005;
      double ki = 0.01;
      double maxPwr = 0.4;

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

   public static double armTarget(boolean a,boolean b,boolean x, boolean y, double prevTgt, double winchPosition)
   {
      double target; 
      target = prevTgt; 
      if (a) {
         if (winchPosition < 3){
            target = 0; 
         } else {
            target = 5;
         }
      }

      if (b) {
         if (winchPosition < 3){
            target = 4; 
         } else {
            target = 5;
         }
      }

      if (y) {
         target = 9;
      }

      if (x) {
         target = 15; 
      }
      return target;
   }
   
   public static double winchTarget(boolean a,boolean b,boolean x, boolean y, double prevTgt, double winchPosition)
   {
      double target; 
      target = prevTgt; 
      if (a) {
         target = 0; 
      }

      if (b) {
         target = 0;
      }

      if (y) {
         target = -25;
      }

      if (x) {
         target = -50; 
      }
      return target;
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