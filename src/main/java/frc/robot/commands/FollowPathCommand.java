// src/main/java/frc/robot/commands/FollowPathCommand.java

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.constants.Constants;

public class FollowPathCommand extends Command {

    private final Drive drive;
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final HolonomicDriveController holonomicController;

    /**
     * Constructs a new FollowPathCommand.
     *
     * @param trajectory The trajectory to follow.
     * @param drive The drive subsystem to control.
     */
    public FollowPathCommand(Trajectory trajectory, Drive drive) {
        this.trajectory = trajectory;
        this.drive = drive;
        addRequirements(drive);

        // Initialize PID controllers with constants
        xController = new PIDController(Constants.translationConstants.kP, Constants.translationConstants.kI, Constants.translationConstants.kD);
        yController = new PIDController(Constants.translationConstants.kP, Constants.translationConstants.kI, Constants.translationConstants.kD);
        thetaController = new ProfiledPIDController(Constants.rotationConstants.kP, Constants.rotationConstants.kI, Constants.rotationConstants.kD, Constants.translationConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicController = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // Reset odometry to the starting pose of the trajectory.
        drive.resetPose(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(currentTime);

        Pose2d currentPose = drive.getPose();

        // Calculate the desired chassis speeds using the holonomic controller
        var targetChassisSpeeds = holonomicController.calculate(
                currentPose,
                desiredState,
                desiredState.poseMeters.getRotation()
        );

        // Command the drive subsystem to move
        drive.driveRobotRelative(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}