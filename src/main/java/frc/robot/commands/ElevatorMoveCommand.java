package frc.robot.commands;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorMoveCommand extends Command {
    // private final SparkMax elevatorNeo1;
    // private final SparkMax elevatorNeo2;
    private final CommandXboxController joystick;
    // private double power;
    private final Elevator elevator;

    public ElevatorMoveCommand(Elevator elevator, CommandXboxController joystick) {
        // Initialize the SparkMax objects in the constructor or initialize method

        this.elevator = elevator;
        this.joystick = joystick;

        addRequirements(elevator);
        // addRequirements(elevatorNeo1, elevatorNeo2);
    }

    


    public void initialize() {


    }


    public boolean isFinished() {
        return false;
    }

    public void execute() {
        elevator.setSpeed(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis());
        // power = joystick.getRightY(); // Adjust the power based on the joystick input

        // elevatorNeo1.set(power);
        // elevatorNeo2.set(-power);

        //nuh uhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
    }

    public void end(boolean interrupted) {
    }
    



}
