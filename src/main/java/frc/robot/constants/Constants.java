package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {

    //Numbers
    public static final int ELEVATOR_NEO_CAN_ID_1 = 21;
    public static final int ELEVATOR_NEO_CAN_ID_2 = 22;
  
    public static final double JOYSTICK_YAW_MULTIPLIER = 4;
    public static final double JOYSTICK_ELEVATOR_MULTIPLIER =-0.7;
    public static final double ELEVATOR_SETPOINT_CONSTANT = 0.5;

    public static final double MAX_ELEVATOR_POSITION = -3.2; // 3.2 normal
    public static final double MIN_ELEVATOR_POSITION = -0.3;

    public static final double ELEVATOR_SPEED_LIMIT_OFFSET = 0.6;
    public static final double ELEVATOR_SPEED_LIMIT_MULTIPLIER = 0.3;
  
    public static final int IP_ADDRESS_LISTEN_PORT = 1234;
    public static final String LISTEN_IP_ADDRESS = "0.0.0.0";

    public static final double JOYSTICK_LEFT_Y_MULTIPLIER = 0.3;
    public static final double JOYSTICK_LEFT_X_MULTIPLIER = 0.3;

    public static final int driverJoystickPort = 0;
    public static final int pigeonId = 15;

    public static final double maxSwerveModuleSpeed = 1; //original 4.5

    public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0); //need to update
    public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);


    //Motors + IDs
    public static SparkMax elevatorNeo1 = new SparkMax(ELEVATOR_NEO_CAN_ID_1, MotorType.kBrushless);
    public static SparkMax elevatorNeo2 = new SparkMax(ELEVATOR_NEO_CAN_ID_2, MotorType.kBrushless);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    //Devices

    public static final Pigeon2 imu = new Pigeon2(pigeonId);


    //PID Controllers
        //Yaw
        public static PIDController yawPIDController = new PIDController(0.02, 0.00003, 0.0015);
        /**
         * https://www.reddit.com/r/FRC/comments/11a8auf/finally_got_our_swerve_drive_working/
         * sometimes fast one
         */
        // public static PIDController yawPIDController = new PIDController(0.0325, 0.00007, 0.002);

        //public static PIDController yawPIDController = new PIDController(0.02, 0.0001, 0.00);

        //Drive
        public static final PIDConstants translationConstants = new PIDConstants(1.0, 0.0, 0.0); //original p 5
        public static final PIDConstants rotationConstants = new PIDConstants(1.0, 0.0, 0.0); //original p 5

        //Elevator
        public static PIDController elevatorPidController = new PIDController(1, 0, 0); // PID Controller for Elevator




}
