// src/main/java/frc/robot/pathplanning/CustomPathPlanner.java

package frc.robot.pathplanning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drive;

import java.util.List;

public class CustomPathPlanner {

    /**
     * Generates a trajectory based on waypoints.
     *
     * @param startPose The starting pose of the trajectory.
     * @param interiorWaypoints A list of interior waypoints.
     * @param endPose The ending pose of the trajectory.
     * @return The generated trajectory.
     */
    public static Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> interiorWaypoints, Pose2d endPose, Drive drive) {
        // Create trajectory configuration
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.MAX_SPEED,
                Constants.MAX_ACCELERATION
        );

        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(drive.getKinematics());

        // Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                interiorWaypoints,
                endPose,
                config
        );

        return trajectory;
    }
}