package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase{
    private final SparkMax intakeMotor;

    public Intake(){
        intakeMotor = Constants.intakeNeoWheel;

        System.out.println("resetting pitch motor encoder");
    }
    public void setIntakePower(double power){
        intakeMotor.set(power);
    }

    public double getPosition(){
        return intakeMotor.getEncoder().getPosition();
    }
}
