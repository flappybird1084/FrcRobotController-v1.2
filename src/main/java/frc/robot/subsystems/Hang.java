package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Hang extends SubsystemBase{
    private SparkMax hangMotor;

    public Hang() {
        hangMotor = Constants.hangNeo;
    }

    public void setPower(double power){
        hangMotor.set(power);
    }

    public double getPosition() {
        return hangMotor.getEncoder().getPosition();
    }

    public void setPowerLimited(double power, double maxPosition, double minPosition) {
        // Ensure minimal movement when power input is zero
        if (power == 0){
            power = 0.02;
        }
        // Adjust power based on joystick multiplier constant
        // power *= Constants.JOYSTICK_ELEVATOR_MULTIPLIER;
        power *= Constants.JOYSTICK_HANG_MULTIPLIER;
        
        // Get the current position of the elevator
        double currentPosition = getPosition();
        
        // Enforce absolute positional limits
        if (power < 0 && currentPosition <= maxPosition) {
            power = 0;
        }
        if (power > 0 && currentPosition >= minPosition) {
            power = 0;
        }
    
        // Proportional Slowdown Near Maximum Position (Upward Movement)
        if (power < 0 && currentPosition < (maxPosition + Constants.HANG_SPEED_LIMIT_OFFSET)) {
            double distanceToMax = currentPosition - maxPosition;
            // Calculate slowdown factor (clamped between 0 and 1)
            double slowdownFactor = Math.max(0.0, distanceToMax / Constants.HANG_SPEED_LIMIT_OFFSET);
            // Apply the slowdown factor to the motor power
            power *= slowdownFactor;
        }
    
        // Proportional Slowdown Near Minimum Position (Downward Movement)
        if (power > 0 && currentPosition > (minPosition - Constants.HANG_SPEED_LIMIT_OFFSET)) {
            double distanceToMin = minPosition - currentPosition;
            // Calculate slowdown factor (clamped between 0 and 1)
            double slowdownFactor = Math.max(0.0, distanceToMin / Constants.HANG_SPEED_LIMIT_OFFSET);
            // Apply the slowdown factor to the motor power
            power *= slowdownFactor;
        }

        hangMotor.set(power);
    }

}
