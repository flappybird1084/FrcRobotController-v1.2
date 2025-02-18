package frc.robot.commands;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;

public class ElevatorMoveCommand extends Command {
    private final SparkMax elevatorNeo1;
    private final SparkMax elevatorNeo2;
    private final CommandXboxController joystick;
    private double power;

    public ElevatorMoveCommand(CommandXboxController joystick) {
        // Initialize the SparkMax objects in the constructor or initialize method
        this.elevatorNeo1 = new SparkMax(TunerConstants.ELEVATOR_NEO_CAN_ID_1, MotorType.kBrushless);
        this.elevatorNeo2 = new SparkMax(TunerConstants.ELEVATOR_NEO_CAN_ID_2, MotorType.kBrushless);
        this.joystick = joystick;

    }


    public void initialize() {


    }


    public boolean isFinished() {
        return false;
    }

    public void execute() {
        power = joystick.getRightY(); // Adjust the power based on the joystick input

        elevatorNeo1.set(power);
        elevatorNeo2.set(-power);

        //nuh uhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
    }

    public void end(boolean interrupted) {
    }
    



}
