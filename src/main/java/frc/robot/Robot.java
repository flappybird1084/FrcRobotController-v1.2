// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
public class Robot extends TimedRobot {


  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static final CommandXboxController joystick = new CommandXboxController(Constants.driverJoystickPort);

  public static Elevator elevator = new Elevator();
  public static MessageListener messageListener = new MessageListener();
  public static Drive drive = new Drive();


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();   

    // System.out.println("passed commandscheduler");


    elevator.setSpeed(joystick.getRightY());

    drive.drive(joystick);


    if(joystick.getHID().getAButtonPressed()){
      System.out.println("Button A was pressed");

    }

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


