// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
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

  private final MotorController m_fLeftMotor  = new CANSparkMax(2,MotorType.kBrushless);
  private final MotorController m_fRightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final MotorController m_bLeftMotor  = new CANSparkMax(4,MotorType.kBrushless);
  private final MotorController m_bRightMotor = new CANSparkMax(3, MotorType.kBrushless);


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_fLeftMotor, m_fRightMotor);
    m_myRobot2 = new DifferentialDrive(m_bLeftMotor, m_bRightMotor);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
  }
}
