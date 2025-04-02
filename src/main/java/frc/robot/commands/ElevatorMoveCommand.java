package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorMoveCommand extends Command {
    // private final SparkMax elevatorNeo1;
    // private final SparkMax elevatorNeo2;
    private final double position;
    // private double power;
    private final Elevator elevator;

    public ElevatorMoveCommand(Elevator elevator, double position) {
        // Initialize the SparkMax objects in the constructor or initialize method

        this.elevator = elevator;

        this.position = position;
        // addRequirements(elevatorNeo1, elevatorNeo2);
    }

    public void initialize() {
    }


    public boolean isFinished() {
        return false;
    }

    public void execute() {
        if (elevator.getPosition() > position) {
            elevator.setSpeed(2);
        } else if (elevator.getPosition() < position) {
            elevator.setSpeed(-2);
        }
    }

    public void end(boolean interrupted) {
    }
    



}
