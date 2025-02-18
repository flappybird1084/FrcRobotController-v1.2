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
        elevatorNeo1.set(this.power);
        elevatorNeo2.set(this.power);
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
