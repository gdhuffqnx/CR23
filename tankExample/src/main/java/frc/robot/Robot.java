// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private DifferentialDrive m_myRobot2;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private boolean timeInit;
  private double time1;
  private double time2;
  private int counter;
  private double distance;
  private double distanceInit;
  private double flcmd = 0;
  private double frcmd = 0;
  private double blcmd = 0;
  private double brcmd = 0;
  private int state; 

  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;

  private final CANSparkMax m_fLeftMotor  = new CANSparkMax(2,MotorType.kBrushless);
  private final CANSparkMax m_fRightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_bLeftMotor  = new CANSparkMax(4,MotorType.kBrushless);
  private final CANSparkMax m_bRightMotor = new CANSparkMax(3, MotorType.kBrushless);
  private RelativeEncoder m_encoderLeft;
  //  private RelativeEncoder m_encoderRight;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    //m_myRobot = new DifferentialDrive(m_fLeftMotor, m_fRightMotor);
    //m_myRobot2 = new DifferentialDrive(m_bLeftMotor, m_bRightMotor);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    timeInit = false;
    time1 = 0;
    counter = 0;
    distance = 0;
    // Starts recording to data log
    DataLogManager.start();
    m_encoderLeft  = m_fLeftMotor.getEncoder();
    //m_encoderRight = m_fRightMotor.getEncoder();
    // Set up custom log entries
    DataLog log = DataLogManager.getLog();
    myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    myDoubleLog = new DoubleLogEntry(log, "/my/double");
    myStringLog = new StringLogEntry(log, "/my/string");
  }

  @Override
  public void autonomousInit() {
    state = 0;
    time1 = 0;
    timeInit = false;
    myBooleanLog.append(true);
    myDoubleLog.append(distance);
    myStringLog.append("init");
    //m_encoderLeft.
  }

  @Override
  public void autonomousPeriodic() {
  switch(state) {
    case 0: 
      if(driveInches(12, 0.05)) {
        state++; 
      }
    break;
    case 1:
      if(turnDegrees(90, 0.05)) {
        state++;
      }
      break; 
  }
  
  }


  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
    //m_myRobot2.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());


    double gain = 0.3;

    if (m_leftStick.getX() > 0.1) {
      flcmd = gain*m_leftStick.getX();
      frcmd = gain*m_leftStick.getX();
      blcmd = gain*-m_leftStick.getX();
      brcmd = gain*m_leftStick.getX();
      
    } else if (m_leftStick.getX() < -0.1) {
      flcmd = gain*m_leftStick.getX();
      frcmd = gain*m_leftStick.getX();
      blcmd = gain*-m_leftStick.getX();
      brcmd = gain*m_leftStick.getX();
    }
    if (m_leftStick.getY() > 0.1) {
      flcmd = gain*m_leftStick.getY();
      frcmd = gain*m_leftStick.getY();
      blcmd = gain*m_leftStick.getY();
      brcmd = -gain*m_leftStick.getY();
    } else if (m_leftStick.getY() < -0.1) {
      flcmd = gain*m_leftStick.getY();
      frcmd = gain*m_leftStick.getY();
      blcmd = gain*m_leftStick.getY();
      brcmd = -gain*m_leftStick.getY();
    }
    if (m_leftStick.getRawButton(3)) {
      flcmd = 0.3; //left motor
      frcmd = 0.3; //left motor
    }
    if (m_leftStick.getRawButton(6)) {
      brcmd = -0.3; //right motor
      blcmd = 0.3; //right motor
    }

   // m_myRobot.tankDrive(flcmd, frcmd);
    //m_myRobot2.tankDrive(blcmd, brcmd);
    

  }
  public boolean driveInches (double inches, double power){
    boolean complete = false; 
    double blcmd;
    double brcmd;
    double flcmd;
    double frcmd;
    //double error;


    //error = m_encoderLeft.getPosition() - m_encoderRight.getPosition();
    
    distance = -(m_encoderLeft.getPosition())/8.45;
    //encoder.getRate(); 

   if (counter > 9)
   {
      //distance = distance + 0.5;
      myBooleanLog.append(true);
      myDoubleLog.append(distance);
      myStringLog.append("distance");
      counter = 0;
   } else {
      counter = counter +1;
   }
        
   //double error = leftEncoder.getDistance() - rightEncoder.getDistance();

   if (timeInit == false)  {
      time1 = Timer.getFPGATimestamp();
      distanceInit = distance;
      timeInit = true;
      //telemetry.addData("Time: ", timeInit);
      //telemetry.update();
   }
   time2 = Timer.getFPGATimestamp();
    
  if ((distance - distanceInit) < (inches*0.0645)) {
      flcmd = -power;//right negative foward
      frcmd = -power;//right negative forward
      blcmd = power;//0.2;
      brcmd = power;//-0.20;
      m_fLeftMotor.set(flcmd);
      m_fRightMotor.set(frcmd);
      m_bLeftMotor.set(blcmd);
      m_bRightMotor.set(brcmd);
      //m_bRightMotor.set(speed:-0.3)

  } else {
      m_fLeftMotor.set(0.0);
      m_fRightMotor.set(0.0);
      m_bLeftMotor.set(0.0);
      m_bRightMotor.set(0.0);  
      complete = true; 
      timeInit = false; 
    }
    return(complete);
  }
  public boolean turnDegrees (double degrees, double power){
    boolean complete = false; 

    double blcmd;
    double brcmd;
    double flcmd;
    double frcmd;
    //double error;


    //error = m_encoderLeft.getPosition() - m_encoderRight.getPosition();
    
    distance = (m_encoderLeft.getPosition())/8.45;
    //encoder.getRate(); 

   if (counter > 9)
   {
      //distance = distance + 0.5;
      myBooleanLog.append(true);
      myDoubleLog.append(distance);
      myStringLog.append("distance");
      counter = 0;
   } else {
      counter = counter +1;
   }
        
   //double error = leftEncoder.getDistance() - rightEncoder.getDistance();

   if (timeInit == false)  {
      time1 = Timer.getFPGATimestamp();
      distanceInit = distance;
      timeInit = true;
      //telemetry.addData("Time: ", timeInit);
      //telemetry.update();
   }
   time2 = Timer.getFPGATimestamp();
    
  if ((distance - distanceInit) < (degrees*0.015)) {
      flcmd = power;//right negative foward
      frcmd = power;//right negative forward
      blcmd = power;//0.2;
      brcmd = power;//-0.20;
      m_fLeftMotor.set(flcmd);
      m_fRightMotor.set(frcmd);
      m_bLeftMotor.set(blcmd);
      m_bRightMotor.set(brcmd);
      //m_bRightMotor.set(speed:-0.3)

  } else {
      m_fLeftMotor.set(0.0);
      m_fRightMotor.set(0.0);
      m_bLeftMotor.set(0.0);
      m_bRightMotor.set(0.0);  
      complete = true; 
      timeInit = false; 
    }
    return(complete);
  }
}
