// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public final class Autos {
  /** Example static factory for an autonomous command. */


  /**
   * 
   * @param driveMotors this should be the motors responsible for driving the robot
   * 
   */
  public static Command exampleAuto(TalonFX... driveMotors) {
    return new DiveTalonFX(driveMotors);
  }

  /**
   * 
   * @param driveMotors this should be the motors responsible for driving the robot
   * 
   */
  public static Command exampleAuto(SparkMax... driveMotors) {
    return new DriveSparkMax(driveMotors);
  }

  public static class DiveTalonFX extends Command{
    private final TalonFX[] m_motors;
    private Timer m_timer = new Timer();


    public DiveTalonFX(TalonFX[] motors) {
      m_motors = motors;
    }

    @Override
    public void initialize() {
      m_timer.start();
      for (TalonFX motor : m_motors) {
        motor.set(0.5);
      }
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
      for (TalonFX motor : m_motors) {
        motor.set(0);
      }
    }
  }

  public static class DriveSparkMax extends Command{
    private final SparkMax[] m_motors;

    private Timer m_timer = new Timer();
  
    public DriveSparkMax(SparkMax[] motors) {
      m_motors = motors;
    }
    @Override
    public void initialize() {
      m_timer.start();
      for (SparkMax motor : m_motors) {
        motor.set(0.5);
      }
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
      for (SparkMax motor : m_motors) {
        motor.set(0);
      }
    }
  }
}