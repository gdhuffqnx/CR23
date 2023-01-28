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
//import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.sensors.CANCoder;

//import com.kauailabs.navx.frc.AHRS;
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
  private float yawInit;
  private double flcmd = 0;
  private double frcmd = 0;
  private double blcmd = 0;
  private double brcmd = 0;
  private double armCmd;
  private double Pgain;
  private double Igain;
  private double errSum;
  private int state; 
  float pitch;
  float yaw;

  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  //private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final CANSparkMax m_fLeftMotor  = new CANSparkMax(2,MotorType.kBrushless);
  private final CANSparkMax m_fRightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_bLeftMotor  = new CANSparkMax(4,MotorType.kBrushless);
  private final CANSparkMax m_bRightMotor = new CANSparkMax(3, MotorType.kBrushless);
  //private final CANSparkMax m_arm         = new CANSparkMax(5, MotorType.kBrushless);
  private RelativeEncoder m_encoderLeft;
  //private RelativeEncoder m_encoderArm;
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
    pitch = 0;
    errSum = 0;
    yaw = 0;
    // Starts recording to data log
    DataLogManager.start();
    m_encoderLeft  = m_fLeftMotor.getEncoder();
    //m_encoderArm = m_arm.getEncoder();
    gyro.calibrate();
    //m_encoderRight = m_fRightMotor.getEncoder();
    // Set up custom log entries
    DataLog log = DataLogManager.getLog();
    myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    myDoubleLog = new DoubleLogEntry(log, "/my/double");
    myStringLog = new StringLogEntry(log, "/my/string");
  }

  @Override
  public void autonomousInit() {
    state = 3;
    time1 = 0;
    timeInit = false;
    Pgain = 0.002;
    Igain = 0.000001;
    myBooleanLog.append(true);
    myDoubleLog.append(distance);
    myStringLog.append("init");
    //m_encoderLeft.
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
         //state++;
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
    //m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
    //m_myRobot2.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());


    double gain = 0.3;
    flcmd = 0.0;
    frcmd = 0.0;
    blcmd = 0.0;
    brcmd = 0.0;
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
       armCmd = -0.15;
    } else {
      armCmd = 0.0;
    }
    if (m_leftStick.getRawButton(6)) {

    }
    //m_fLeftMotor
    distance = m_encoderLeft.getPosition();
    m_fLeftMotor.set(armCmd);
    //m_fRightMotor.set(frcmd);
    //m_bLeftMotor.set(blcmd);
    //m_bRightMotor.set(brcmd);
    //m_arm.set(armCmd);
    // m_myRobot.tankDrive(flcmd, frcmd);
    //m_myRobot2.tankDrive(blcmd, brcmd);
    //distance = m_encoderArm.getPosition();

    if (counter > 9)
    {
       //distance = distance + 0.5;
       myBooleanLog.append(true);
       myDoubleLog.append(distance);
       myStringLog.append("armEncoder");
       counter = 0;
    } else {
       counter = counter +1;
    }

  }
  public boolean driveInches (double inches, double power){
    boolean complete = false; 
    double blcmd;
    double brcmd;
    double flcmd;
    double frcmd;
    //double error;
    //double newPitch;
    //newPitch = double(pitch);

    //error = m_encoderLeft.getPosition() - m_encoderRight.getPosition();
    
    distance = -(m_encoderLeft.getPosition())/8.45;
    //encoder.getRate(); 

   if (counter > 9)
   {
      //distance = distance + 0.5;
      myBooleanLog.append(true);
      myDoubleLog.append(pitch);
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
    double error;
    


    //error = m_encoderLeft.getPosition() - m_encoderRight.getPosition();
    
    //yaw =  = (m_encoderLeft.getPosition())/8.45;
    //encoder.getRate(); 
   distance = Double.valueOf(yaw);

        
   //double error = leftEncoder.getDistance() - rightEncoder.getDistance();

   if (timeInit == false)  {
      time1 = Timer.getFPGATimestamp();
      distanceInit = distance;
      timeInit = true;
      yawInit = yaw - 45;
      errSum = 0;
      //telemetry.addData("Time: ", timeInit);
      //telemetry.update();
   }
   time2 = Timer.getFPGATimestamp();
    
   error = Double.valueOf(yaw - yawInit);
   errSum = errSum + error; 
  if (true) {
      blcmd = (-Pgain * error) + (-Igain * errSum);
      if (blcmd > 1.0) {
        blcmd = 1.0;
      }
      if (blcmd < -1.0) {
        blcmd = -1.0;
      }

      m_fLeftMotor.set(blcmd);
      m_fRightMotor.set(blcmd);
      m_bLeftMotor.set(blcmd);
      m_bRightMotor.set(blcmd);
      //m_bRightMotor.set(speed:-0.3)

  } else {
      m_fLeftMotor.set(0.0);
      m_fRightMotor.set(0.0);
      m_bLeftMotor.set(0.0);
      m_bRightMotor.set(0.0);  
      complete = true; 
      timeInit = false; 
    }
  

    if (counter > 9)
    {
       //distance = distance + 0.5;
       myBooleanLog.append(true);
       //myDoubleLog.append(distance);
       myDoubleLog.append(error);
       myStringLog.append("distance");
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
    //double error;


    //error = m_encoderLeft.getPosition() - m_encoderRight.getPosition();
    
    //distance = (m_encoderLeft.getPosition())/8.45;
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
      //distanceInit = distance;
      timeInit = true;
      //telemetry.addData("Time: ", timeInit);
      //telemetry.update();
   }
   time2 = Timer.getFPGATimestamp();
    
  if ((pitch) < (-5.0)) {
      flcmd = -power;//right negative foward
      frcmd = -power;//right negative forward
      blcmd = power;//0.2;
      brcmd = power;//-0.20;
      m_fLeftMotor.set(flcmd);
      m_fRightMotor.set(frcmd);
      m_bLeftMotor.set(blcmd);
      m_bRightMotor.set(brcmd);
      //m_bRightMotor.set(speed:-0.3)
  } else if(pitch > 5.0) {
    flcmd = power;//right negative foward
    frcmd = power;//right negative forward
    blcmd = -power;//0.2;
    brcmd = -power;//-0.20;
    m_fLeftMotor.set(flcmd);
    m_fRightMotor.set(frcmd);
    m_bLeftMotor.set(blcmd);
    m_bRightMotor.set(brcmd);
  
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

