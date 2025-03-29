package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.util.AprilTagPIDReading;
import frc.robot.util.ProportionalSlowdownController;

public class Drive extends SubsystemBase{
    private double currentAngle;
    private double targetAngle;
    private double targetRotationalRate;
    private static PIDController yawPIDController;
    private static ProportionalSlowdownController yawProportionalSlowdownController;
    private final double JOYSTICK_YAW_MULTIPLIER = Constants.JOYSTICK_YAW_MULTIPLIER;

    private SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    
    private Field2d field = new Field2d();
    private Pigeon2 gyro = new Pigeon2(Constants.pigeonId);

    private double limitedTargetX = 0.0;
    private double limitedTargetY = 0.0;
    private final double MAX_DELTA = 0.05; // Max joystick change (to prevent sudden acc)

    private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
    .withDeadband(RobotContainer.MaxSpeed * 0.04)
    .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.04);

    public static RobotConfig config;


    public Drive() {
        this.targetAngle = gyro.getYaw().getValueAsDouble();
        yawPIDController = Constants.yawPIDController;
        yawProportionalSlowdownController = new ProportionalSlowdownController();

        modules = drivetrain.getModules(); // does this kill robotcontainer code??
        kinematics = drivetrain.getKinematics(); // does this kill robotcontainer code??
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

        //start pathplanner breaking my head
        try{
        config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder
        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            new PPHolonomicDriveController(
            Constants.translationConstants,
            Constants.rotationConstants
            ),
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
        }
        catch(Exception e){
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));


        SmartDashboard.putData("Field", field);

    }

    public void periodic(){

        odometry.update(gyro.getRotation2d(), getPositions());

        field.setRobotPose(getPose());
    }

    /**
     * Limits the change of a value to within the specified maximum delta.
     *
     * @param currentValue The current limited target value.
     * @param desiredValue The desired target value from the joystick.
     * @param maxDelta The maximum allowed change per update.
     * @return The new limited target value after applying the rate limit.
     */
    private double limitDelta(double currentValue, double desiredValue, double maxDelta) {
        double delta = desiredValue - currentValue;
        
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        
        return currentValue + delta;
    }
    

    public void drive(CommandXboxController joystick) {
        currentAngle = gyro.getYaw().getValueAsDouble();
        if (Math.abs(joystick.getRightX()) > 0.1) {
            targetAngle -= joystick.getRightX() * JOYSTICK_YAW_MULTIPLIER;
        }
    
        // yawPIDController.setSetpoint(targetAngle);
        // targetRotationalRate = yawPIDController.calculate(currentAngle);
        if(Math.abs(joystick.getRightX()) < 0.1){
            targetRotationalRate = yawProportionalSlowdownController.calculate(currentAngle, targetAngle, 0.01);
        }
        // if(Math.abs(joystick.getRightX()) < 0.03){targetRotationalRate=0;}
        else{
        targetRotationalRate = yawProportionalSlowdownController.calculate(currentAngle, targetAngle);
        }

        RobotContainer.targetRotationalRate = targetRotationalRate;
        // Get the desired joystick inputs
        double desiredY = joystick.getLeftY();
        double desiredX = joystick.getLeftX();

        // Apply rate limiting to the joystick inputs
        
        // limitedTargetY = limitDelta(limitedTargetY, desiredY, MAX_DELTA);
        // limitedTargetX = limitDelta(limitedTargetX, desiredX, MAX_DELTA);
        limitedTargetX = desiredX;
        limitedTargetY = desiredY;

        // Update targetY and targetX with the limited values
        RobotContainer.targetY = limitedTargetY;
        RobotContainer.targetX = limitedTargetX;
    }

    public Pose2d getPose() {
    return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        System.out.println(pose);
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
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

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        double targetPowerX = targetSpeeds.vxMetersPerSecond/RobotContainer.MaxSpeed;
        double targetPowerY = targetSpeeds.vyMetersPerSecond/RobotContainer.MaxSpeed;
        double targetRotationalPower = targetSpeeds.omegaRadiansPerSecond;
        
        // SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        // setStates(targetStates);
         // old, not working for us

         System.out.println("targetPowerX: " + targetPowerX);
         System.out.println("targetPowerY: " + targetPowerY);
         System.out.println("targetRotationalPower: " + targetRotationalPower);

        // RobotContainer.targetX = targetPowerX;
        // RobotContainer.targetY = targetPowerY;
        // RobotContainer.targetRotationalRate = targetRotationalPower;

        // RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive
        // .withVelocityX(targetPowerX*(0.2))
        // .withVelocityY(targetPowerY*(0.2))
        // .withRotationalRate(targetRotationalPower))
        // // .withTimeout(0.2)
        // .schedule();

        RobotContainer.drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(targetPowerX)
        .withVelocityY(targetPowerY)
        .withRotationalRate(targetRotationalPower))
        // .withTimeout(0.2)
        .schedule();



    }

    public void driveRobotRelativeWithFeedFwd(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards driveFeedforward) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        double targetPowerX = targetSpeeds.vxMetersPerSecond/RobotContainer.MaxSpeed;
        double targetPowerY = targetSpeeds.vyMetersPerSecond/RobotContainer.MaxSpeed;
        double targetRotationalPower = targetSpeeds.omegaRadiansPerSecond;
        
        // SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        // setStates(targetStates);
         // old, not working for us

         System.out.println("targetPowerX: " + targetPowerX);
         System.out.println("targetPowerY: " + targetPowerY);
         System.out.println("targetRotationalPower: " + targetRotationalPower);

        // RobotContainer.targetX = targetPowerX;
        // RobotContainer.targetY = targetPowerY;
        // RobotContainer.targetRotationalRate = targetRotationalPower;

        // RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive
        // .withVelocityX(targetPowerX*(0.2))
        // .withVelocityY(targetPowerY*(0.2))
        // .withRotationalRate(targetRotationalPower))
        // // .withTimeout(0.2)
        // .schedule();

        RobotContainer.drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(targetPowerX)
        .withVelocityY(targetPowerY)
        .withRotationalRate(targetRotationalPower))
        // .withTimeout(0.2)
        .schedule();



    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.maxSwerveModuleSpeed);

        for (int i = 0; i < modules.length; i++) {
        ModuleRequest request = new ModuleRequest();
        request.withState(targetStates[i]);
        modules[i].apply(request);

        }
    }

    public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition(false);
    }
    return positions;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void centerAprilTagWithPIDReading(AprilTagPIDReading pidReading){
    double damper = 0.5;


    RobotContainer.drivetrain.applyRequest(() -> driveRobotCentric
    .withVelocityX(pidReading.getPidZ()*damper*RobotContainer.MaxSpeed)
    .withVelocityY(pidReading.getPidX()*damper*RobotContainer.MaxSpeed)
    .withRotationalRate(pidReading.getPidYaw()*RobotContainer.MaxAngularRate))
    .withTimeout(0.02)
    .schedule();
  }

public void centerAprilTagPathPlanner(AprilTagPIDReading aprilTagReading) {
    double damper = 0.1; 
// Pull X and Y offsets from AprilTag
    double targetMetersX = aprilTagReading.getMetersX();
    double targetMetersY = aprilTagReading.getMetersY()*0.9;

    // Get the tag's rotation in degrees (assume getTagRotation() returns degrees)
    double targetRotation = -aprilTagReading.getTagRotation(); 
    // Convert degrees -> Rotation2d
    Rotation2d tagRotation2d = Rotation2d.fromDegrees(targetRotation);

    // Get the robot’s current pose
    Pose2d currentPose = getPose();
    Rotation2d currentHeading = currentPose.getRotation();

    // Build the offset in the robot's coordinate frame
    Translation2d localOffset = new Translation2d(targetMetersY, -targetMetersX);

    // Rotate that offset by the current heading to express it in field coordinates
    Translation2d fieldOffset = localOffset.rotateBy(currentHeading);

    // Start pose uses the robot’s current heading
    Pose2d startPos = new Pose2d(
        currentPose.getTranslation(),
        currentHeading
    );

    // END POSE: You can decide how to use the rotation:
    //
    // Option 1: COMPLETELY replace the heading with the AprilTag heading (absolute).
    // Rotation2d endHeading = tagRotation2d;
    //
    // Option 2: Add the AprilTag heading to the current heading (relative).
    Rotation2d endHeading = currentHeading.plus(tagRotation2d);

    // Create the end pose
    Pose2d endPos = new Pose2d(
        currentPose.getTranslation().plus(fieldOffset),
        endHeading
    );

    // Generate the path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(
            4.0 * damper, 
            4.0,
            Units.degreesToRadians(360), 
            Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
    );

    // Prevent flipping if your path coordinates are already correctly oriented
        path.preventFlipping = true;

    // Follow the path
        AutoBuilder.followPath(path).finallyDo(() -> {
            System.out.println("Path complete");
            RobotContainer.drivetrain.setDefaultCommand(RobotContainer.driveDefaultCommand);
        }).schedule();

} 

public void followLambdaPath(Translation2d end){
    double damper = 0.1; 
// Pull X and Y offsets from AprilTag
    double targetMetersX = end.getX();
    double targetMetersY = end.getY();

    // Get the tag's rotation in degrees (assume getTagRotation() returns degrees)
    double targetRotation = end.getAngle().getDegrees();
    // Convert degrees -> Rotation2d
    Rotation2d tagRotation2d = Rotation2d.fromDegrees(targetRotation);

    // Get the robot’s current pose
    Pose2d currentPose = getPose();
    Rotation2d currentHeading = currentPose.getRotation();

    // Build the offset in the robot's coordinate frame
    Translation2d localOffset = new Translation2d(targetMetersY, -targetMetersX);

    // Rotate that offset by the current heading to express it in field coordinates
    Translation2d fieldOffset = localOffset.rotateBy(currentHeading);

    // Start pose uses the robot’s current heading
    Pose2d startPos = new Pose2d(
        currentPose.getTranslation(),
        currentHeading
    );

    // END POSE: You can decide how to use the rotation:
    //
    // Option 1: COMPLETELY replace the heading with the AprilTag heading (absolute).
    // Rotation2d endHeading = tagRotation2d;
    //
    // Option 2: Add the AprilTag heading to the current heading (relative).
    Rotation2d endHeading = currentHeading.plus(tagRotation2d);

    // Create the end pose
    Pose2d endPos = new Pose2d(
        currentPose.getTranslation().plus(fieldOffset),
        endHeading
    );

    // Generate the path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(
            4.0 * damper, 
            4.0,
            Units.degreesToRadians(360), 
            Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
    );

    // Prevent flipping if your path coordinates are already correctly oriented
        path.preventFlipping = true;

    new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelativeWithFeedFwd, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        ).schedule();

}
  public void stop(){
    for(int i = 0; i < modules.length; i++){
        modules[i].apply(new ModuleRequest().withWheelForceFeedforwardX(0.0).withWheelForceFeedforwardY(0.0));
    }
  }
}
