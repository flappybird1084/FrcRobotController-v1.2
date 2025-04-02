package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    public void initialize() {
    }


    public boolean isFinished() {
        return false;
    }

    public void execute() {
        intake.setIntakePower(.5);     
    }

    public void end(boolean interrupted) {
    }
}
 ;