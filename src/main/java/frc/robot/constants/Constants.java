package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

public class Constants {

    //Numbers
    public static final int ELEVATOR_NEO_CAN_ID_1 = 23;
    public static final int ELEVATOR_NEO_CAN_ID_2 = 22;

    // public static final int INTAKE_NEO_PITCH_CAN_ID = 23;
    public static final int INTAKE_NEO_WHEEL_CAN_ID = 24; 

    public static final int ALGAE_NEO_CAN_ID_1 = 27;
    public static final int ALGAE_NEO_CAN_ID_2 = 28; 

    public static final int HANG_NEO_CAN_ID = 29;
  
    public static final double JOYSTICK_YAW_MULTIPLIER = 4;
    public static final double JOYSTICK_ELEVATOR_MULTIPLIER =-0.5;
    public static final double ELEVATOR_SETPOINT_CONSTANT = 0.5;

    // public static final double MAX_ELEVATOR_POSITION = -3.28; // 3.2 normal
    // public static final double MIN_ELEVATOR_POSITION = 0;

    public static final double MAX_ELEVATOR_POSITION = -2.6; // 3.2 normal
    public static final double MIN_ELEVATOR_POSITION = 0;

    public static final double ELEVATOR_SPEED_LIMIT_OFFSET = 0.6;
    public static final double ELEVATOR_SPEED_LIMIT_MULTIPLIER = 0.3;
    public static final double ELEVATOR_OFFSET_POS = 0;

    public static final double JOYSTICK_CORAL_MULTIPLIER = -0.1;
    public static final double CORAL_PITCH_SPEED_LIMIT_OFFSET = 0.75;
    public static final double CORAL_BOOST_STOP = 3.38;
    public static final double MIN_CORAL_POSITION = 0;
    public static final double MAX_CORAL_POSITION = 4.4;

    public static final double JOYSTICK_HANG_MULTIPLIER = -0.7;
    public static final double HANG_SPEED_LIMIT_OFFSET = 0.6;
    public static final double MIN_HANG_POSITION = -62.358;
    public static final double MAX_HANG_POSITION = 38.714;

  
    public static final int IP_ADDRESS_LISTEN_PORT = 1234;
    public static final String LISTEN_IP_ADDRESS = "0.0.0.0";

    public static final double JOYSTICK_LEFT_Y_MULTIPLIER = 0.3;
    public static final double JOYSTICK_LEFT_X_MULTIPLIER = 0.3;

    // public static final double JOYSTICK_LEFT_Y_MULTIPLIER = 0.5;
    // public static final double JOYSTICK_LEFT_X_MULTIPLIER = 0.5;

    // public static final double JOYSTICK_LEFT_Y_MULTIPLIER = 1.0;
    // public static final double JOYSTICK_LEFT_X_MULTIPLIER = 1.0;

    public static final int driverJoystickPort = 0;
    public static final int coDriverJoystickPort = 1;
    public static final int pigeonId = 15;

    public static final double maxSwerveModuleSpeed = 1; //original 4.5

    public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0); //need to update
    public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.5).in(MetersPerSecondPerSecond);

    public static final TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION);



    //Motors + IDs
    public static SparkMax elevatorNeo1 = new SparkMax(ELEVATOR_NEO_CAN_ID_1, MotorType.kBrushless);
    public static SparkMax elevatorNeo2 = new SparkMax(ELEVATOR_NEO_CAN_ID_2, MotorType.kBrushless);

    // public static SparkMax intakeNeoPitch = new SparkMax(INTAKE_NEO_PITCH_CAN_ID, MotorType.kBrushless);
    public static SparkMax intakeNeoWheel = new SparkMax(INTAKE_NEO_WHEEL_CAN_ID, MotorType.kBrushless);

    public static SparkMax algaeNeo1 = new SparkMax(ALGAE_NEO_CAN_ID_1, MotorType.kBrushless);
    public static SparkMax algaeNeo2 = new SparkMax(ALGAE_NEO_CAN_ID_2, MotorType.kBrushless);

    public static SparkMax hangNeo = new SparkMax(HANG_NEO_CAN_ID, MotorType.kBrushless);


    public static final CommandXboxController joystick = new CommandXboxController(0);

    //Devices

    public static final Pigeon2 imu = new Pigeon2(pigeonId);


    //PID Controllers
        //Yaw
        public static PIDController yawPIDController = new PIDController(0.025, 0.000015, 0.0015);
        /**
         * https://www.reddit.com/r/FRC/comments/11a8auf/finally_got_our_swerve_drive_working/
         * sometimes fast one
         */
        // public static PIDController yawPIDController = new PIDController(0.0325, 0.00007, 0.002);

        //public static PIDController yawPIDController = new PIDController(0.02, 0.0001, 0.00);

        //Drive
        // public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0); //original p 5
        // public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0); //original p 5
        public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0); //original p 5
        public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0); //original p 5


        //Elevator
        public static PIDController elevatorPidController = new PIDController(1, 0, 0); // PID Controller for Elevator

        //Intake
        public static PIDController intakePidController = new PIDController(0.12, 0.0, 0.0); // PID Controller for Intake





}
