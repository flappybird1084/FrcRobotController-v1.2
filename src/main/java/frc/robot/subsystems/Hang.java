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

    public void setPitchPowerLimited(double power) {
        // negative power = up
        double maxPosition = Constants.MAX_CORAL_POSITION;
        double minPosition = Constants.MIN_CORAL_POSITION;

        // Ensure minimal movement when power input is zero

        // Adjust power based on joystick multiplier constant
        // power *= Constants.JOYSTICK_ELEVATOR_MULTIPLIER;
        power *= Constants.JOYSTICK_HANG_MULTIPLIER;
        
        // Get the current position of the elevator
        double currentPosition = getPosition();
        
        // Enforce absolute positional limits
        if (power > 0 && currentPosition >= maxPosition) {
            power = 0;
        }
        if (power < 0 && currentPosition <= minPosition) {
            power = 0;
        }
    
        // Proportional Slowdown Near Maximum Position (Upward Movement)
        if (power > 0 && currentPosition > (maxPosition - Constants.HANG_SPEED_LIMIT_OFFSET)) {
            double distanceToMax = maxPosition-currentPosition;
            // Calculate slowdown factor (clamped between 0 and 1)
            double slowdownFactor = Math.max(0.0, distanceToMax / Constants.HANG_SPEED_LIMIT_OFFSET);
            // Apply the slowdown factor to the motor power
            power *= slowdownFactor;
        }
    
        // Proportional Slowdown Near Minimum Position (Downward Movement)
        if (power < 0 && currentPosition < (minPosition + Constants.HANG_SPEED_LIMIT_OFFSET)) {
            double distanceToMin = currentPosition - minPosition;
            // Calculate slowdown factor (clamped between 0 and 1)
            double slowdownFactor = Math.max(0.0, distanceToMin / Constants.HANG_SPEED_LIMIT_OFFSET) * (1+5*power); //when power is too high the slowdown hits like a slap to the face
            // Apply the slowdown factor to the motor power
            power *= -slowdownFactor;
        }
    
        // Set the motor output with the adjusted power (only one motor)
        hangMotor.set(power);
    }


}
