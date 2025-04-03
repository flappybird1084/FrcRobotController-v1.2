package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class JoystickDriveCommand extends Command {
    private final Drive driveSubsystem;
    private final CommandXboxController joystick;

    public JoystickDriveCommand(Drive subsystem, CommandXboxController joystick) {
        this.driveSubsystem = subsystem;
        this.joystick = joystick;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(joystick, 1);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}