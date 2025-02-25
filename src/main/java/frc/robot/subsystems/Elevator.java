package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {
    // Add any necessary fields or methods here
    private final SparkMax elevatorNeo1;
    private final SparkMax elevatorNeo2;
    // private final CommandXboxController joystick;
    private double power;

    public Elevator() {
        System.out.println("Initializing Elevator subsystem...");
        elevatorNeo1 = Constants.elevatorNeo1;
        elevatorNeo2 = Constants.elevatorNeo2;
        // joystick = Constants.joystick;
        // Initialize any necessary components here

        SparkMaxConfig elevatorNeo1Config = new SparkMaxConfig();
        elevatorNeo1Config.inverted(true);

        elevatorNeo1.configure(elevatorNeo1Config, null, null);
    }

    public void setSpeed(double power){
        this.power = power * Constants.JOYSTICK_ELEVATOR_MULTIPLIER;

        // Limits
        if (this.power < 0 && getPosition() <= Constants.MAX_ELEVATOR_POSITION) this.power = 0;
        if (this.power > 0 && getPosition() >= Constants.MIN_ELEVATOR_POSITION) this.power = 0;

        // Speed Limiting when near the limits
        if (this.power < 0 && getPosition() <= Constants.MAX_ELEVATOR_POSITION + Constants.ELEVATOR_SPEED_LIMIT_OFFSET) this.power = Math.max(Constants.JOYSTICK_ELEVATOR_MULTIPLIER*Constants.ELEVATOR_SPEED_LIMIT_MULTIPLIER, this.power);
        if (this.power > 0 && getPosition() >= Constants.MIN_ELEVATOR_POSITION-Constants.ELEVATOR_SPEED_LIMIT_OFFSET) this.power = Math.min(-Constants.JOYSTICK_ELEVATOR_MULTIPLIER*Constants.ELEVATOR_SPEED_LIMIT_MULTIPLIER, this.power);


        elevatorNeo1.set(this.power);
        elevatorNeo2.set(this.power);
    }

    public double getPosition() {
        return elevatorNeo2.getEncoder().getPosition();
    }

    public void stop() {
        elevatorNeo1.stopMotor();
        elevatorNeo2.stopMotor();
    }

    public void stopGently(){
        elevatorNeo1.set(0);
        elevatorNeo2.set(0);

    }


}
