// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.BooleanSupplier;

import javax.sound.sampled.SourceDataLine;
import javax.swing.text.Position;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CenterOnPoint;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final double originalMaxSpeed = MaxSpeed; // max speed for the robot

    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final static double JOYSTICK_LEFT_Y_MULTIPLIER = Constants.JOYSTICK_LEFT_Y_MULTIPLIER;
    private final static double JOYSTICK_LEFT_X_MULTIPLIER = Constants.JOYSTICK_LEFT_X_MULTIPLIER;

    public static boolean doTeleopDriving;

    
    //accessible by drive.java
    public static double targetRotationalRate = 0.0;
    public static double targetX = 0.0;
    public static double targetY = 0.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.04).withRotationalDeadband(MaxAngularRate * 0.04 ) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static Command driveDefaultCommand = drivetrain.applyRequest(() ->
    drive.withVelocityX(targetY * MaxSpeed * JOYSTICK_LEFT_Y_MULTIPLIER) // Drive forward with negative Y (forward)
        .withVelocityY(targetX * MaxSpeed * JOYSTICK_LEFT_X_MULTIPLIER) // Drive left with negative X (left)
        .withRotationalRate(targetRotationalRate*MaxAngularRate) // Drive counterclockwise with negative X (left)
);


    //Subsystems
    Drive driveSubsystem = new Drive();
    Elevator elevatorSubsystem = new Elevator();
    Intake intakeSubsystem = new Intake();

    // Test Positions
    ElevatorMoveCommand elevatorMoveCommandUp = new ElevatorMoveCommand(elevatorSubsystem, 0);
    ElevatorMoveCommand elevatorMoveCommandDown = new ElevatorMoveCommand(elevatorSubsystem, 0);


    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("ElevatorUp", elevatorMoveCommandUp);
        NamedCommands.registerCommand("ElevatorDown", elevatorMoveCommandDown);

        NamedCommands.registerCommand("IntakeOut", new IntakeCommand(intakeSubsystem));
    }

    public void setMaxSpeed(double speed) {
        MaxSpeed = speed;
    }

    public double getMaxSpeed() {
        return MaxSpeed;
    }

    public static void setDefault() {
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
    }



    private void configureBindings() {
        // driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, joystick));
        // elevatorSubsystem.setDefaultCommand(new ElevatorMoveCommand(elevatorSubsystem,joystick));
        setDefault();
        
        // SmartDashboard.putData("Example Path", new PathPlannerAuto("squarish auto"));
        // SmartDashboard.putData("Run Test Lambda Path", new InstantCommand(() -> {
        //     driveSubsystem.pauseTeleopDriving();
        //     driveSubsystem.followLambdaPath(new Translation2d(0,0)); //will resume when done
        // }));

        

        // SmartDashboard.putData("Run Center W/Pose2d", new InstantCommand(() -> {
        //     driveSubsystem.pauseTeleopDriving();
        //     driveSubsystem.centerWithDistanceReading(new Pose2d(0.5,0.5, new Rotation2d(0)));
        // }));

        // SmartDashboard.putData("Run Center W/Pose2d", new CenterOnPoint(driveSubsystem, new Pose2d(0.5,0.5,new Rotation2d(0))));

        // Trigger t = joystick.x();
        // t.whileTrue(new CenterOnPoint(driveSubsystem, new Pose2d(0.5,0.5,new Rotation2d(0))));

        // t.whileTrue(new InstantCommand(() -> System.out.println("hoho hop")));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        

       
        

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

    
        // return Commands.sequence(
        //     Commands.runOnce(() -> {
        //         // TODO: This maybe is off by 180?
        //         drivetrain.resetRotation(drivetrain.getOperatorForwardDirection());
        //     }),    
        // drivetrain.applyRequest(
        //         () ->
        //         // TODO: Might be going wrong way
        //             drive.withVelocityX(-0.5*MaxSpeed) // Drive forward with negative Y (forward)
        //                 .withVelocityY(0) // Drive left with negative X (left)
        //                 .withRotationalRate(0))
        //                     .withTimeout(3)
        //                         .andThen(
        //                             ()->
        //                                 drive.withVelocityX(0) // Drive forward with negative Y (forward)
        //                                     .withVelocityY(0) // Drive left with negative X (left)
        //                                     .withRotationalRate(0)));

            // return Commands.print("No autonomous command configured");
            // return new PathPlannerAuto("New Auto");

            boolean shouldDoFancyAuto = false;

            if(shouldDoFancyAuto){
                try{
                    AllianceStationID allianceStationID = DriverStation.getRawAllianceStation();

                    if (allianceStationID.equals(AllianceStationID.Red1)) {
                        return new PathPlannerAuto("red-auto-1");
                    } else if (allianceStationID.equals(AllianceStationID.Blue1)) {
                        return new PathPlannerAuto("blue-auto-1");
                    } else if (allianceStationID.equals(AllianceStationID.Red2)) {
                        return new PathPlannerAuto("red-auto-2");
                    } else if (allianceStationID.equals(AllianceStationID.Blue2)) {
                        return new PathPlannerAuto("blue-auto-2");
                    } else if (allianceStationID.equals(AllianceStationID.Red3)) {
                        return new PathPlannerAuto("red-auto-3");
                    } else if (allianceStationID.equals(AllianceStationID.Blue3)) {
                        return new PathPlannerAuto("blue-auto-3");
                    } else {
                        // Handle unknown station ID case
                        return new PathPlannerAuto("emergency auto");
                    }
                }
                catch(Exception e){
                    System.out.println(e.getMessage());
                    return new PathPlannerAuto("emergency auto");
                }
            }
            else{
                return new PathPlannerAuto("emergency auto");
            }

            // return new PathPlannerAuto("squarish auto");
            

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
