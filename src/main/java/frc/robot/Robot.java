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

  public static boolean shouldDisableDrive = false;


  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static final CommandXboxController joystick = new CommandXboxController(Constants.driverJoystickPort);
  public static final CommandXboxController coJoystick = new CommandXboxController(Constants.coDriverJoystickPort);

  public static Elevator elevator = new Elevator();
  public static MessageListener messageListener = new MessageListener();
  public static Drive drive = new Drive();
  public static Intake intake = new Intake();
  // public static Hang hang = new Hang();

  double startTime;

  @Override
  public boolean isDisabled() {
    return super.isDisabled() || shouldDisableDrive;


  }

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

  @SuppressWarnings("unused")
  @Override
  public void teleopPeriodic() {

    // m_robotContainer.setMaxSpeed(m_robotContainer.originalMaxSpeed * Math.max((1-elevator.getPercentageUp()), 0.1));

    
    // if(joystick.getHID().getYButton() && messageListener.timeSinceLastMessage() < 1000){
    //   System.out.println("Running AprilTag with PID Reading: "+messageListener.getAprilTagPIDReading().toString());
    //   drive.centerAprilTagWithPIDReading(messageListener.getAprilTagPIDReading());
    // }
    // if(joystick.getHID().getXButtonPressed() && messageListener.timeSinceLastMessage() < 1000){
    //   System.out.println("Running AprilTag better with PID Reading: "+messageListener.getAprilTagPIDReading().toString());
    //   drive.centerAprilTagPathPlanner(messageListener.getAprilTagPIDReading());
    // }
    // // if(joystick.getHID().getYButtonPressed()){
    // //   drive.followLambdaPath(new Translation2d(0.5,0.5));
    // // }
    // // if(joystick.getHID().getYButtonPressed()){
    // //   drive.centerWithDistanceReading(drive.getPose().plus(new Transform2d(0.5, 0.5, new Rotation2d(0))).getTranslation());
    // // }
    // else{
    //   drive.drive(joystick);
    // }

    if (joystick.getHID().getRightBumperButton()) {
      drive.drive(joystick, 0.25);
    } else {
      drive.drive(joystick, 1);

    }

    intake.setIntakePower(coJoystick.getRightY());
    // elevator.setSpeedNoLimit(joystick.getRightY());
    //elevator.setSpeedNoLimit(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis());
    elevator.setSpeed(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis());

    // if(coJoystick.getHID().getBackButtonPressed()){
    //   elevator.elevatorOffset = elevator.getPosition()-Constants.MIN_ELEVATOR_POSITION;
    //   System.out.println("elevator offset reset");
    // }
    

    //TODO: MUST RECALIBRATE 
    // if(false){
    //   if(coJoystick.getHID().getBButton()){ //empty preset
    //     elevator.setPosition(-0.48);
    //     // intake.setIntakePower(-0.25);
    //     if(elevator.getPosition()>-0.55){
    //       intake.setIntakePower(-0.25);
    //     }
        
    //   }
    //   if(coJoystick.getHID().getYButton()){ //preset for L3
    //     elevator.setPosition(-2.25);
    //     // intake.setIntakePower(0.15);
    //     if(elevator.getPosition()<-2.0){
    //       intake.setIntakePower(0.07);
    //     }
    //   }
    // }

      
    
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


