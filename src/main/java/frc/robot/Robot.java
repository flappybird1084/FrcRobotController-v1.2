// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
public class Robot extends TimedRobot {

  public static boolean shouldDisableDrive = false;


  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static final CommandXboxController joystick = new CommandXboxController(Constants.driverJoystickPort);
  public static final CommandXboxController coJoystick = new CommandXboxController(Constants.coDriverJoystickPort);

  public static Elevator elevator = new Elevator();
  public static MessageListener messageListener = new MessageListener();
  public static Drive drive = new Drive();
  // public static Intake intake = new Intake();

  public static boolean alternativeElevatorMode = false;


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();   

    // System.out.println("passed commandscheduler");

    //Left is up, Right is down
    // elevator.setSpeed(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis());


    // if(joystick.getHID().getAButtonPressed()){
    //   System.out.println("Button A was pressed");

    // }

    // elevatorNeo1.set(joystick.getRightY()*JOYSTICK_ELEVATOR_MULTIPLIER);
    // elevatorNeo2.set(-joystick.getRightY()*JOYSTICK_ELEVATOR_MULTIPLIER);

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
  public void autonomousPeriodic() { 
}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


  }

  @Override
  public void teleopPeriodic() {

    // m_robotContainer.setMaxSpeed(m_robotContainer.originalMaxSpeed * Math.max((1-elevator.getPercentageUp()), 0.1));

    drive.drive(joystick);
    elevator.setSpeed(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());

    if(coJoystick.getHID().getBackButtonPressed()){
      elevator.elevatorOffset = elevator.getPosition()-Constants.MIN_ELEVATOR_POSITION;
      System.out.println("elevator offset reset");
    }

    if(coJoystick.getHID().getBButtonPressed()){ //preset for coral processor
      elevator.setPosition(-2.0);
      //add intake
    }

    

    // intake.setIntakePower(coJoystick.getRightY());
    // intake.setPitchPower(coJoystick.getLeftY());



  }

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


