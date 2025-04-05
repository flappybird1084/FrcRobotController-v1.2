package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class CenterOnPoint extends Command {
    private Drive driveSubsystem;
    private Pose2d targetPose2d;

    public CenterOnPoint(Drive driveSubsystem, Pose2d target){
        this.driveSubsystem = driveSubsystem;
        this.targetPose2d = target;
        addRequirements(driveSubsystem);

    }
    
    @Override
    public void initialize(){
        driveSubsystem.stopCentering = false;
    }

    @Override
    public void execute(){
        while (!driveSubsystem.stopCentering) {
            driveSubsystem.centerWithDistanceReading(targetPose2d);
        }
        
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopCentering = true;
    }
}
