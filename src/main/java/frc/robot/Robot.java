// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
// import com.google.gson.Gson;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static final CommandXboxController joystick = new CommandXboxController(0);

  public static double currentAngle;
  public static double targetAngle = RobotContainer.imu.getYaw().getValueAsDouble();

  public static SparkMax test = new SparkMax(21, MotorType.kBrushless);


  double targetRotationalRate;

  public static final double JOYSTICK_YAW_MULTIPLIER = 4;

  public final int port = 1234;
  public final String ipAddress = "0.0.0.0";
  public static String[] lastMessage = new String[]{"No messages to display"};

  String[] split_strings;
  ArrayList<AprilTagDetected> tags;

  public static ArrayList<AprilTagDetected> currentDetectedAprilTags = new ArrayList<AprilTagDetected>();


  private Thread IPAddressListenerThread = RobotContainer.getIPAddressListenerThread(port, ipAddress, lastMessage);

  //public static PIDController yawPIDController = new PIDController(0.02, 0.00003, 0.0015);
  // public static PIDController yawPIDController = new PIDController(0.0325, 0.00007, 0.002);
  public static PIDController yawPIDController = new PIDController(0.02, 0.0001, 0.00);

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    if (!IPAddressListenerThread.isAlive()){
      IPAddressListenerThread.start();
    }

    currentAngle = RobotContainer.imu.getYaw().getValueAsDouble();
    if(Math.abs(joystick.getRightX()) > 0.1){
      targetAngle -= joystick.getRightX()*JOYSTICK_YAW_MULTIPLIER;
    }
    
    yawPIDController.setSetpoint(targetAngle);
    targetRotationalRate = yawPIDController.calculate(currentAngle);

    RobotContainer.targetRotationalRate = targetRotationalRate;

    split_strings = lastMessage[0].split("-next-detection-");
    tags = new ArrayList<>();
    // System.out.println("split strings length: " + split_strings.length);
    // System.out.println("split strings[0]: " + split_strings[0]);
    // System.out.println("split strings[1]: " + split_strings[1]);

    for (String tag:split_strings){
      try{

        tags.add(AprilTagDetected.parseTag(tag));
        System.out.println("tag id" +  AprilTagDetected.parseTag(tag).getTagId());
        } catch (Exception e){
          // System.out.println("Invalid tag: " + tag);
          
        }

    }

    

    currentDetectedAprilTags = tags;

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
