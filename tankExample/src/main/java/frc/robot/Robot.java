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
   //private double armCmd;
   private double Pgain;
   private double Igain;
   private double errSum;
   //private double errSumWheelAngle[4];
   private int state; 
   float pitch;
   float yaw;

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
      distance = 0;
      pitch = 0;
      errSum = 0;
      yaw = 0;
      prevDesiredAngle = 0;
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
      state = 3;
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
            if(driveInches(24, 0.15)) {
               state++; 
            }
         break;
         case 1:
            if(balancePitch(0.05)) {
               state++;
            }
         break;
         case 2:
            if(turnDegrees(90, 0.05)) {
               state++;
            }
         break; 
      }
   }

   @Override
   public void teleopPeriodic() {

      double gain = 0.3;
      double drCmd;
      double desiredAngle;
      double filteredAngle;
      double joyXPos;
      double joyYPos;

      joyXPos = m_leftStick.getX();
      joyYPos = m_leftStick.getY();
      filteredAngle = 0.0;
      blcmd = 0.0;
      flcmd = 0.0;
      frcmd = 0.0;
      brcmd = 0.0;
      drCmd = 0.0;

      desiredAngle = prevDesiredAngle;

      if ((Math.abs(joyXPos) > 0.09)||(Math.abs(joyYPos) > 0.09)){
         drCmd = (joyXPos*m_leftStick.getX())+(m_leftStick.getY()*m_leftStick.getY());
         drCmd = gain * Math.sqrt(drCmd);

         if ((m_leftStick.getX() > 0.1)&&(m_leftStick.getY() < -0.1)) {
            desiredAngle = 57.3*Math.atan(Math.abs(m_leftStick.getY()/m_leftStick.getX()));
          }
          if ((m_leftStick.getX() > 0.1)&&(Math.abs(m_leftStick.getY()) < 0.1)) {
            desiredAngle = 90;
          }
          if ((m_leftStick.getX() < -0.1)&&(Math.abs(m_leftStick.getY()) < 0.1)) {
            desiredAngle = -90;
          }
          if ((Math.abs(m_leftStick.getX()) < 0.1)&&(m_leftStick.getY() < -0.1)) {
            desiredAngle = 179.9;
          }
          if ((Math.abs(m_leftStick.getX()) < 0.1)&&(m_leftStick.getY() > 0.1)) {
            desiredAngle = 0.0;
          }
          if ((m_leftStick.getX() > 0.1)&&(m_leftStick.getY() > 0.1)) {
             desiredAngle = 180-57.3*Math.atan(Math.abs(m_leftStick.getY()/m_leftStick.getX()));
          }
 
          if ((m_leftStick.getX() < -0.1)&&(m_leftStick.getY() < -0.1)) {
            desiredAngle = -57.3*Math.atan(Math.abs(m_leftStick.getY()/m_leftStick.getX()));
          }
 
          if ((m_leftStick.getX() < -0.1)&&(m_leftStick.getY() > 0.1)) {
            desiredAngle = -180+57.3*Math.atan(Math.abs(m_leftStick.getY()/m_leftStick.getX()));
          }

         filteredAngle = prevDesiredAngle + (0.1*(desiredAngle - prevDesiredAngle));

         flcmd = -wheelAngle(filteredAngle,frontLeftEncoder.getPosition(),0,0.011,0.0,0);
         frcmd = wheelAngle(filteredAngle,frontRightEncoder.getPosition(),1,.011,0.0,0);
         blcmd = -wheelAngle(filteredAngle,backLeftEncoder.getPosition(),3,0.011,0.0,0);
         brcmd = -wheelAngle(filteredAngle,backRightEncoder.getPosition(),2,0.011,0.0,0);
         
         prevDesiredAngle = filteredAngle;

      }
      if (m_leftStick.getRawButton(6)) {}
      //
      //

      if (counter > 9)
      {
         myBooleanLog.append(true);
         myDoubleLog.append(desiredAngle);
         myStringLog.append("desired");
         myBooleanLog.append(true);
         myDoubleLog.append(frontLeftEncoder.getPosition());
         myStringLog.append("current");
         myBooleanLog.append(true);
         myDoubleLog.append(backRightEncoder.getPosition());
         myStringLog.append("cmd");
         counter = 0;
      } else {
         counter = counter +1;
      }
      m_frontLeftDrive.set(drCmd);
      m_frontRightDrive.set(drCmd);
      m_backLeftDrive.set(-drCmd);
      m_backRightDrive.set(drCmd);
	  
      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);
   }

   public boolean driveInches (double inches, double power){
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
      
      if (timeInit == false)  {
         //time1 = Timer.getFPGATimestamp();
         distanceInit = distance;
         timeInit = true;
      }
    
      if ((distance - distanceInit) < (inches*0.0645)) {
         flcmd = -power;
         frcmd = -power;
         blcmd = power;
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
     return(complete);
   }

   public boolean turnDegrees (double degrees, double power){
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
         myDoubleLog.append(distance);
         myStringLog.append("distance");
         counter = 0;
      } else {
         counter = counter +1;
      }
      
   if (timeInit == false)  {
      //time1 = Timer.getFPGATimestamp();
      timeInit = true;
   }
   //time2 = Timer.getFPGATimestamp();
    
   if ((pitch) < (-5.0)) {
      flcmd = -power;
      frcmd = -power;
      blcmd = power;
      brcmd = power;
      m_frontLeftDrive.set(flcmd);
      m_frontRightDrive.set(frcmd);
      m_backLeftDrive.set(blcmd);
      m_backRightDrive.set(brcmd);
   } else if(pitch > 5.0) {
      flcmd = power;
      frcmd = power;
      blcmd = -power;
      brcmd = -power;
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
   return(complete);
   }

   public double wheelAngle(double desiredAngle, double currentAngle, int id, double kp, double ki, double kd)
   {
      double errorA;
      double powerCmd;
	   double errSum;
      double deltaError;

	   errorA = 0;
      errSum = 0;
      deltaError = 0;

      switch(id) {
      case 0:
      errorA = desiredAngle - currentAngle; 
         deltaError = (errorA - flErrPrev)*50;
         flErrPrev = errorA;
         flErrSum = flErrSum + (errorA*0.02);
         errSum = flErrSum;
         break;
      case 1:
         errorA = desiredAngle - currentAngle; 
         deltaError = (errorA - frErrPrev)*50;
         frErrPrev = errorA;
         frErrSum = frErrSum + (errorA*0.02);
         errSum = flErrSum;
         break;
      case 2:
         errorA = desiredAngle - currentAngle; 
         deltaError = (errorA - blErrPrev)*50;
         blErrPrev = errorA;
         blErrSum = blErrSum + (errorA*0.02);
         errSum = flErrSum;
         break;
      case 3:
         errorA = desiredAngle - currentAngle; 
         deltaError = (errorA - brErrPrev)*50;
         brErrPrev = errorA;
         brErrSum = brErrSum + (errorA*0.02);
         errSum = brErrSum;
         break;
      }

      powerCmd = (kp * errorA) + (ki * errSum) + (kd * deltaError);
	    if (powerCmd > 0.5){powerCmd = 0.5;}
       if (powerCmd < -0.5){powerCmd = -0.5;}
      return(powerCmd);
   }
}

