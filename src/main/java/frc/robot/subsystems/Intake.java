package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.constants.Constants;

public class Intake {
    private final SparkMax pitchMotor;
    private final SparkMax intakeMotor;

    public Intake(){
        pitchMotor = Constants.intakeNeoPitch;
        intakeMotor = Constants.intakeNeoWheel;

    }

    public void setPitchPower(double power){
        pitchMotor.set(power);
    }
    public void setIntakePower(double power){
        intakeMotor.set(power);
    }



}
