package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase{
    private final SparkMax pitchMotor;
    private final SparkMax intakeMotor;

    private final SparkMax algaeIntake1;
    private final SparkMax algaeIntake2;


    public Intake(){
        pitchMotor = Constants.intakeNeoPitch;
        intakeMotor = Constants.intakeNeoWheel;
        algaeIntake1 = Constants.algaeNeo1;
        algaeIntake2 = Constants.algaeNeo2;

        System.out.println("resetting pitch motor encoder");
    }

    public void resetEncoderToValue(double value){
        pitchMotor.getEncoder().setPosition(value);
    }

    public void setPitchPower(double power){
        pitchMotor.set(power*Constants.JOYSTICK_CORAL_MULTIPLIER);
    }
    public void setIntakePower(double power){
        intakeMotor.set(power);
    }

    public void setAlgaePower(double power){
        algaeIntake1.set(power);
        algaeIntake2.set(-power);
    }


    public double getPosition(){
        return pitchMotor.getEncoder().getPosition();
    }

    public void setPitchPowerLimited(double power) {
        // negative power = up
        double maxPosition = Constants.MAX_CORAL_POSITION;
        double minPosition = Constants.MIN_CORAL_POSITION;

        // Ensure minimal movement when power input is zero
        if(power > -0.1 && power < 0.1){
            power += (maxPosition-getPosition())/maxPosition * -0.5;
            // System.out.println(power);
        }

        // Adjust power based on joystick multiplier constant
        // power *= Constants.JOYSTICK_ELEVATOR_MULTIPLIER;
        power *= Constants.JOYSTICK_CORAL_MULTIPLIER;
        
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
        if (power > 0 && currentPosition > (maxPosition - Constants.CORAL_PITCH_SPEED_LIMIT_OFFSET)) {
            double distanceToMax = maxPosition-currentPosition;
            // Calculate slowdown factor (clamped between 0 and 1)
            double slowdownFactor = Math.max(0.0, distanceToMax / Constants.CORAL_PITCH_SPEED_LIMIT_OFFSET);
            // Apply the slowdown factor to the motor power
            power *= slowdownFactor;
        }
    
        // Proportional Slowdown Near Minimum Position (Downward Movement)
        if (power < 0 && currentPosition < (minPosition + Constants.CORAL_PITCH_SPEED_LIMIT_OFFSET)) {
            double distanceToMin = currentPosition - minPosition;
            // Calculate slowdown factor (clamped between 0 and 1)
            double slowdownFactor = Math.max(0.0, distanceToMin / Constants.CORAL_PITCH_SPEED_LIMIT_OFFSET) * (1+5*power); //when power is too high the slowdown hits like a slap to the face
            // Apply the slowdown factor to the motor power
            power *= -slowdownFactor;
        }
    
        // Set the motor output with the adjusted power (only one motor)
        pitchMotor.set(power);
    }

}
