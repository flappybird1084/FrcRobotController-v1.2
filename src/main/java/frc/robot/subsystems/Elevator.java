package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {
    // Add any necessary fields or methods here
    private final SparkMax elevatorNeo1;
    private final SparkMax elevatorNeo2;
    // private final CommandXboxController joystick;
    private double power;
    private double targetPosition;
    private PIDController pidController;
    private double neoOffset;

    public Elevator() {
        System.out.println("Initializing Elevator subsystem...");
        elevatorNeo1 = Constants.elevatorNeo1;
        elevatorNeo2 = Constants.elevatorNeo2;
        targetPosition = getPosition();
        pidController = Constants.elevatorPidController;
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

        // PIDController elevatorPidController = new PIDController(1, 0, 0); // PID Controller for Elevator
        // double slowdownUp = elevatorPidController.calculate(this.getPosition(), Constants.MAX_ELEVATOR_POSITION);
        // double slowdownDown = elevatorPidController.calculate(this.getPosition(), Constants.MIN_ELEVATOR_POSITION);
    
        // this.power = power - Math.min(slowdownUp, slowdownDown);
    



        elevatorNeo1.set(this.power);
        elevatorNeo2.set(this.power);
    }

    public double getPosition() {
        return elevatorNeo2.getEncoder().getPosition();
    }

    public void setPosition(double setpoint){
        setpoint += Constants.ELEVATOR_SETPOINT_CONSTANT;
        
        if(setpoint > Constants.MIN_ELEVATOR_POSITION+neoOffset)
            setpoint = Constants.MIN_ELEVATOR_POSITION+neoOffset;
        if(setpoint < Constants.MAX_ELEVATOR_POSITION+neoOffset)
            setpoint = Constants.MAX_ELEVATOR_POSITION+neoOffset;

        this.targetPosition = setpoint;
        double error = pidController.calculate(this.getPosition(), targetPosition);

        this.power = Math.min(error, 0.5); // Slow down the motor to prevent overshooting

        elevatorNeo1.set(this.power);
        elevatorNeo2.set(this.power);
    }

    public void setPositionRelative(double setpoint){
        targetPosition = targetPosition + setpoint;
        setPosition(targetPosition);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void calibrateBottomPosition(){
        //assume we're at the bottom position
        neoOffset = -Constants.MIN_ELEVATOR_POSITION+getPosition();
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
