// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagDetected;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final double JOYSTICK_LEFT_Y_MULTIPLIER = 0.3;
    public final double JOYSTICK_LEFT_X_MULTIPLIER = 0.3;


    public static Pigeon2 imu = new Pigeon2(TunerConstants.kPigeonId);
    public static double targetRotationalRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
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
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed * JOYSTICK_LEFT_Y_MULTIPLIER) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed * JOYSTICK_LEFT_X_MULTIPLIER) // Drive left with negative X (left)
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
        return Commands.print("No autonomous command configured");
    
    }

    public static Thread getIPAddressListenerThread(int port, String ipAddress, String[] lastMessage){
        Thread packetListener = new Thread() {
        @Override
        public void run() {
        try (DatagramSocket socket = new DatagramSocket(port, InetAddress.getByName(ipAddress))) {
                System.out.println("Listening for packets on IP: " + ipAddress + ", Port: " + port);

                byte[] receiveData = new byte[1024]; // Buffer to hold incoming data
                DatagramPacket packet = new DatagramPacket(receiveData, receiveData.length);

                while (true) {
                    // Receive a packet
                    socket.receive(packet);

                    // Extract the packet data
                    String receivedMessage = new String(packet.getData(), 0, packet.getLength());
                    // System.out.println("Received packet: " + receivedMessage);

                    // // Print the sender's address and port
                    // System.out.println("From IP: " + packet.getAddress().getHostAddress() + ", Port: " + packet.getPort());4
                    // System.out.println(receivedMessage);
                    lastMessage[0] = receivedMessage;
                    // Send a response back to the sender if needed
                    // socket.send(packet); // Uncomment this line if you want to send a response

                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

  return packetListener;

    }   



}
