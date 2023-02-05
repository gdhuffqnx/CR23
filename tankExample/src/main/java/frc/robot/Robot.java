// Copyright (2023)
// Control Freaks
// Vicksburg High School
// Michigan

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

   private Joystick m_leftStick;
   //private Joystick m_rightStick;

   CANCoder frontRightEncoder = new CANCoder(1);
   CANCoder frontLeftEncoder = new CANCoder(0);
   CANCoder backRightEncoder = new CANCoder(2);
   CANCoder backLeftEncoder = new CANCoder(3);

   private boolean timeInit;
   private int counter;
   private int pitchCounter;
   private int buttonEdge6;
   private double distance;
   private double distanceInit;
   private float yawInit;
   private double flcmd = 0;
   private double frcmd = 0;
   private double blcmd = 0;
   private double brcmd = 0;
   private double flErrPrev = 0;
   private double frErrPrev = 0;
   private double blErrPrev = 0;
   private double brErrPrev = 0;
   private double flErrSum = 0;
   private double frErrSum = 0;
   private double blErrSum = 0;
   private double brErrSum = 0;
   private double prevDesiredAngle;
   private double prevGain;

   private double backLeftSpinAngle;
   private double backRightSpinAngle;
   private double frontLeftSpinAngle;
   private double frontRightSpinAngle;

   private double prevBackLeftSpinAngle;
   private double prevBackRightSpinAngle;
   private double prevFrontLeftSpinAngle;
   private double prevFrontRightSpinAngle;

   private double backLeftDriveGain;
   private double backRightDriveGain;
   private double frontLeftDriveGain;
   private double frontRightDriveGain;

   //private double armCmd;
   private double Pgain;
   private double Igain;
   private double errSum;
   //private double errSumWheelAngle[4];
   private int state; 
   float pitch;
   float yaw;
   private int angleCounter;

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

   private RelativeEncoder e_frontLeftDrive;
   private RelativeEncoder e_frontRightDrive;
   private RelativeEncoder e_backLeftDrive;
   private RelativeEncoder e_backRightDrive;



   @Override
   public void robotInit() {
      m_leftStick  = new Joystick(0);
      //m_rightStick = new Joystick(1);

      timeInit = false;
      counter = 0;
      pitchCounter = 0;
      distance = 0;
      pitch = 0;
      errSum = 0;
      yaw = 0;
      prevDesiredAngle = 0;
      prevGain = 0;
      angleCounter = 0;
      buttonEdge6 = 0;

      gyro.calibrate();
	  
	   e_frontLeftDrive  = m_frontLeftDrive.getEncoder();
      e_frontRightDrive = m_frontRightDrive.getEncoder();
      e_backLeftDrive   = m_backLeftDrive.getEncoder();
      e_backRightDrive  = m_backRightDrive.getEncoder();
	  
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
      Pgain = 0.002;
      Igain = 0.000001;
      myBooleanLog.append(true);
      myDoubleLog.append(distance);
      myStringLog.append("init");
   }

   @Override
   public void autonomousPeriodic() {
      yaw = gyro.getYaw();
      pitch =   gyro.getPitch();
      switch(state) {
         case 0: 
            if(driveForwardInches(36, 0.25)) {
               state++;
               //state++; //jump to state 2 
            }
         break;
         case 1:
            if(balancePitch(0.1)) {
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
      double filteredAngle;
      double joyXPos;
      double joyYPos;
      double kp;
	   double d_yaw;
      double gainTgt;
      boolean yawCompensationEnable = true;

      kp = 0.005;
      
      

      joyXPos = m_leftStick.getX();
      joyYPos = m_leftStick.getY();
      blcmd = 0.0;
      flcmd = 0.0;
      frcmd = 0.0;
      brcmd = 0.0;
      drCmd = 0.0;
      yaw = gyro.getYaw();
      desiredAngle = prevDesiredAngle;

      
      if (m_leftStick.getRawButton(2)) {
         gainTgt = 0.99;
      } else {
         gainTgt = 0.15;
      }
      
      gain = prevGain + (0.025*(gainTgt - prevGain));
      prevGain = gain;
      gain = gain * orientation;

      if ((Math.abs(joyXPos) > 0.09)||(Math.abs(joyYPos) > 0.09)){
         drCmd = (joyXPos*joyXPos)+(joyYPos*joyYPos);
         drCmd = gain * Math.sqrt(drCmd);

         if ((joyXPos > 0.1)&&(joyYPos < -0.1)) {
            desiredAngle = -57.3*Math.atan(Math.abs(joyYPos/joyXPos));
          }
          if ((joyXPos > 0.1)&&(Math.abs(joyYPos) < 0.1)) {
            desiredAngle = -90;
          }
          if ((joyXPos < -0.1)&&(Math.abs(joyYPos) < 0.1)) {
            desiredAngle = 90;
          }
          if ((Math.abs(joyXPos) < 0.1)&&(joyYPos < -0.1)) {
            desiredAngle = 0;
          }
          if ((Math.abs(joyXPos) < 0.1)&&(joyYPos > 0.1)) {
            desiredAngle = 180;
          }
          if ((joyXPos > 0.1)&&(joyYPos > 0.1)) {
             desiredAngle = -180+57.3*Math.atan(Math.abs(joyYPos/joyXPos));
          }
 
          if ((joyXPos < -0.1)&&(joyYPos < -0.1)) {
            desiredAngle = 57.3*Math.atan(Math.abs(joyYPos/joyXPos));
          }
 
          if ((joyXPos < -0.1)&&(joyYPos > 0.1)) {
            desiredAngle = 180-57.3*Math.atan(Math.abs(joyYPos/joyXPos));
          }
      }

      if ((frontLeftEncoder.getPosition() > 90)) {
         if (desiredAngle < -90) {
            desiredAngle = desiredAngle + 360;
         }
      }
      if ((frontLeftEncoder.getPosition() > 450)) {
         if (desiredAngle < -90) {
            desiredAngle = desiredAngle + 720;
         }
      }
      if ((frontLeftEncoder.getPosition() < -90)) {
         if (desiredAngle > 90) {
            desiredAngle = desiredAngle - 360;
         }
      }
     if ((frontLeftEncoder.getPosition() > 450)) {
         if (desiredAngle > 90) {
            desiredAngle = desiredAngle - 720;
         }
      }

   
      backLeftSpinAngle = 0;
      backRightSpinAngle =0;
      frontLeftSpinAngle =0;
      frontRightSpinAngle =0;
      backLeftDriveGain =1;
      backRightDriveGain =0.8;
      frontLeftDriveGain= 1;
      frontRightDriveGain = 0.8;
      
	  d_yaw = Double.valueOf(yaw);
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

      if ((m_leftStick.getZ()) > 0.75){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = 0.08 * orientation;
      }
      if ((m_leftStick.getZ()) < -0.75){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = -0.08 * orientation;
      }
      if (m_leftStick.getRawButton(6)) {
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

      filteredAngle = prevDesiredAngle + (0.05*(desiredAngle - prevDesiredAngle));

      flcmd = -wheelAngle(filteredAngle+frontLeftSpinAngle,frontLeftEncoder.getPosition(),0,kp,0.0,0);
      frcmd = wheelAngle(filteredAngle+frontRightSpinAngle,frontRightEncoder.getPosition(),1,kp,0.0,0);
      blcmd = -wheelAngle(filteredAngle+backLeftSpinAngle,backLeftEncoder.getPosition(),3,kp,0.0,0);
      brcmd = -wheelAngle(filteredAngle+backRightSpinAngle,backRightEncoder.getPosition(),2,kp,0.0,0);
         
      prevDesiredAngle = filteredAngle;

      

    
      //
      //

      if (counter > 9)
      {
         myBooleanLog.append(true);
         myDoubleLog.append(yaw);
         myStringLog.append("yaw");
         //myBooleanLog.append(true);
         /*myDoubleLog.append(frontRightEncoder.getVelocity());
         myStringLog.append("x");
         myBooleanLog.append(true);
         myDoubleLog.append(backLeftEncoder.getVelocity());
         myStringLog.append("y");*/
         counter = 0;
      } else {
         counter = counter +1;
      }

      m_frontLeftDrive.set(drCmd*frontLeftDriveGain);
      m_frontRightDrive.set(drCmd*frontRightDriveGain);
      m_backLeftDrive.set(-drCmd*backLeftDriveGain);
      m_backRightDrive.set(drCmd*backRightDriveGain);
	  
      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);
   }

   public boolean driveForwardInches(double inches, double power){
      boolean complete = false; 
    
      if (counter > 9)
      {
         myBooleanLog.append(true);
         myDoubleLog.append(pitch);
         myStringLog.append("distance");
         counter = 0;
      } else {
         counter = counter +1;
      }
      distance = e_frontLeftDrive.getPosition();
      if (timeInit == false)  {
         //time1 = Timer.getFPGATimestamp();
         distanceInit = distance;
         timeInit = true;
      }
    
      if ((distance - distanceInit) < (inches*0.56)) {
         flcmd = power;
         frcmd = power;
         blcmd = -power;
         brcmd = power;
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
        myBooleanLog.append(true);
        myDoubleLog.append(distance);
        myStringLog.append("d");

        counter = 0;
     } else {
        counter = counter +1;
     }
     return(complete);
   }

   public boolean driveRightInches (double inches, double power){
      boolean complete = false; 
      double blcmd;
      double error;
    
      distance = Double.valueOf(yaw);

      if (timeInit == false)  {
         distanceInit = distance;
         timeInit = true;
         yawInit = yaw - 45;
         errSum = 0;
      }
    
      error = Double.valueOf(yaw - yawInit);
      errSum = errSum + error;

      blcmd = (-Pgain * error) + (-Igain * errSum);
      if (blcmd > 1.0) {
         blcmd = 1.0;
      }
      if (blcmd < -1.0) {
         blcmd = -1.0;
      }
      m_frontLeftDrive.set(blcmd);
      m_frontRightDrive.set(blcmd);
      m_backLeftDrive.set(blcmd);
      m_backRightDrive.set(blcmd);
  
      if (counter > 9)
      {
         myBooleanLog.append(true);
         myDoubleLog.append(error);
         myStringLog.append("error");
         counter = 0;
      } else {
         counter = counter +1;
      }
      return(complete);
   }

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
         //time1 = Timer.getFPGATimestamp();
         timeInit = true;
         pitchCounter = 0;
      }
      //time2 = Timer.getFPGATimestamp();
    
      if ((pitch) < (-5.0)) {
         flcmd = power;
         frcmd = power;
         blcmd = -power;
         brcmd = power;
         m_frontLeftDrive.set(flcmd);
         m_frontRightDrive.set(frcmd);
         m_backLeftDrive.set(blcmd);
         m_backRightDrive.set(brcmd);
      } else if(pitch > 5.0) {
         flcmd = -power*0.5;
         frcmd = -power*0.5;
         blcmd = power*0.5;
         brcmd = -power*0.5;
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
   
   public double wheelAngle(double desiredAngle, double currentAngle, int id, double kp, double ki, double kd)
   {
      double errorAngle;
      double powerCmd;
      double errSum;
      double deltaError;

      errorAngle = 0;
      errSum = 0;
      deltaError = 0;

      switch(id) {
      case 0:
      errorAngle = desiredAngle - currentAngle; 
         deltaError = (errorAngle - flErrPrev)*50;
         flErrPrev = errorAngle;
         flErrSum = flErrSum + (errorAngle*0.02);
         errSum = flErrSum;
         break;
      case 1:
         errorAngle = desiredAngle - currentAngle; 
         deltaError = (errorAngle - frErrPrev)*50;
         frErrPrev = errorAngle;
         frErrSum = frErrSum + (errorAngle*0.02);
         errSum = flErrSum;
         break;
      case 2:
         errorAngle = desiredAngle - currentAngle; 
         deltaError = (errorAngle - blErrPrev)*50;
         blErrPrev = errorAngle;
         blErrSum = blErrSum + (errorAngle*0.02);
         errSum = flErrSum;
         break;
      case 3:
         errorAngle = desiredAngle - currentAngle; 
         deltaError = (errorAngle - brErrPrev)*50;
         brErrPrev = errorAngle;
         brErrSum = brErrSum + (errorAngle*0.02);
         errSum = brErrSum;
         break;
      }

      powerCmd = (kp * errorAngle) + (ki * errSum) + (kd * deltaError);
      if (powerCmd > 0.5){powerCmd = 0.5;}
      if (powerCmd < -0.5){powerCmd = -0.5;}
      return(powerCmd);
   }
}

