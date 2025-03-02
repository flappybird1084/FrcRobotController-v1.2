// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.pathplanning.CustomPathPlanner;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final double JOYSTICK_LEFT_Y_MULTIPLIER = Constants.JOYSTICK_LEFT_Y_MULTIPLIER;
    private final double JOYSTICK_LEFT_X_MULTIPLIER = Constants.JOYSTICK_LEFT_X_MULTIPLIER;

    
    //accessible by drive.java
    public static double targetRotationalRate = 0.0;
    public static double targetX = 0.0;
    public static double targetY = 0.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    //Subsystems
    Drive driveSubsystem = new Drive();
    Elevator elevatorSubsystem = new Elevator();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, joystick));
        // elevatorSubsystem.setDefaultCommand(new ElevatorMoveCommand(elevatorSubsystem,joystick));

        SmartDashboard.putData("Example Path", new PathPlannerAuto("New Auto"));


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
            drivetrain.applyRequest(() ->
                drive.withVelocityX(targetY * MaxSpeed * JOYSTICK_LEFT_Y_MULTIPLIER) // Drive forward with negative Y (forward)
                    .withVelocityY(targetX * MaxSpeed * JOYSTICK_LEFT_X_MULTIPLIER) // Drive left with negative X (left)
                    .withRotationalRate(targetRotationalRate*MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            
        );

        

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("New Auto");
            // Define the start pose, interior waypoints, and end pose
        Pose2d startPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        List<Translation2d> interiorWaypoints = List.of(
                new Translation2d(1.0, 1.0),
                new Translation2d(2.0, -1.0)
        );
        Pose2d endPose = new Pose2d(3.0, 0.0, new Rotation2d(0.0));

        // Generate the trajectory
        edu.wpi.first.math.trajectory.Trajectory trajectory = CustomPathPlanner.generateTrajectory(startPose, interiorWaypoints, endPose, driveSubsystem);

        // Create and return the command to follow the trajectory
        return new FollowPathCommand(trajectory, driveSubsystem);
    
    
    }

    // deprecated
//     private static Thread getIPAddressListenerThread(int port, String ipAddress, String[] lastMessage){
//         Thread packetListener = new Thread() {
//         @Override
//         public void run() {
//         try (DatagramSocket socket = new DatagramSocket(port, InetAddress.getByName(ipAddress))) {
//                 System.out.println("Listening for packets on IP: " + ipAddress + ", Port: " + port);

//                 byte[] receiveData = new byte[16384]; // Buffer to hold incoming data
//                 DatagramPacket packet = new DatagramPacket(receiveData, receiveData.length);

//                 while (true) {
//                     // Receive a packet
//                     socket.receive(packet);

//                     // Extract the packet data
//                     String receivedMessage = new String(packet.getData(), 0, packet.getLength());
//                     // System.out.println("Received packet: " + receivedMessage);

//                     // // Print the sender's address and port
//                     // System.out.println("From IP: " + packet.getAddress().getHostAddress() + ", Port: " + packet.getPort());4
//                     // System.out.println(receivedMessage);
//                     lastMessage[0] = receivedMessage;
//                     // Send a response back to the sender if needed
//                     // socket.send(packet); // Uncomment this line if you want to send a response

//                 }

//             } catch (Exception e) {
//                 e.printStackTrace();
//             }
//         }
//     };

//   return packetListener;

//     }   



 }
