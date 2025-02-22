package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;

public class Drive extends SubsystemBase{
    private double currentAngle;
    private double targetAngle;
    private double targetRotationalRate;
    private static PIDController yawPIDController;
    private final double JOYSTICK_YAW_MULTIPLIER = Constants.JOYSTICK_YAW_MULTIPLIER;

    private SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    
    private Field2d field = new Field2d();
    private Pigeon2 gyro = new Pigeon2(Constants.pigeonId);

    CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    public Drive() {
        this.targetAngle = gyro.getYaw().getValueAsDouble();
        yawPIDController = Constants.yawPIDController;

        modules = drivetrain.getModules(); // does this kill robotcontainer code??
        kinematics = drivetrain.getKinematics(); // does this kill robotcontainer code??
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

        //start pathplanner breaking my head
        try{
        RobotConfig config = RobotConfig.fromGUISettings();

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
    

    public void drive(CommandXboxController joystick) {
        currentAngle = gyro.getYaw().getValueAsDouble();
        if (Math.abs(joystick.getRightX()) > 0.1) {
            targetAngle -= joystick.getRightX() * JOYSTICK_YAW_MULTIPLIER;
        }
    
        yawPIDController.setSetpoint(targetAngle);
        targetRotationalRate = yawPIDController.calculate(currentAngle);
        RobotContainer.targetRotationalRate = targetRotationalRate;
        RobotContainer.targetY = joystick.getLeftY();
        RobotContainer.targetX = joystick.getLeftX();
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

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
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

  public void stop(){
    for(int i = 0; i < modules.length; i++){
        modules[i].apply(new ModuleRequest().withWheelForceFeedforwardX(0.0).withWheelForceFeedforwardY(0.0));
    }
  }
}
