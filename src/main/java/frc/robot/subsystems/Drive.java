package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class Drive {
    private double currentAngle;
    private double targetAngle;
    private double targetRotationalRate;
    private static PIDController yawPIDController;
    private final double JOYSTICK_YAW_MULTIPLIER = Constants.JOYSTICK_YAW_MULTIPLIER;

    public Drive() {
        this.targetAngle = Constants.imu.getYaw().getValueAsDouble();
        yawPIDController = Constants.yawPIDController;
    }

    public void drive(CommandXboxController joystick) {
        currentAngle = Constants.imu.getYaw().getValueAsDouble();
        if (Math.abs(joystick.getRightX()) > 0.1) {
            targetAngle -= joystick.getRightX() * JOYSTICK_YAW_MULTIPLIER;
        }
    
        yawPIDController.setSetpoint(targetAngle);
        targetRotationalRate = yawPIDController.calculate(currentAngle);
        RobotContainer.targetRotationalRate = targetRotationalRate;
        RobotContainer.targetY = joystick.getLeftY();
        RobotContainer.targetX = joystick.getLeftX();
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getTargetRotationalRate() {
        return targetRotationalRate;
    }
}
