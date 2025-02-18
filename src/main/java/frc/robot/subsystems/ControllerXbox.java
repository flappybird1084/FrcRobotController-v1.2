package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class ControllerXbox extends CommandXboxController{
    private CommandXboxController controller;

    private boolean a;
    private boolean b;
    private boolean x;
    private boolean y;
    private boolean leftBumper;
    private boolean rightBumper;


    public ControllerXbox(int port) {
        super(port);

        this.controller = Constants.joystick;




        controller.a().onTrue(new InstantCommand(() -> a = true));
        controller.a().onFalse(new InstantCommand(() -> a = false));
        controller.b().onTrue(new InstantCommand(() -> b = true));
        controller.b().onFalse(new InstantCommand(() -> b = false));
        controller.x().onTrue(new InstantCommand(() -> x = true));
        controller.x().onFalse(new InstantCommand(() -> x = false));
        controller.y().onTrue(new InstantCommand(() -> y = true));
        controller.y().onFalse(new InstantCommand(() -> y = false));
        controller.leftBumper().onTrue(new InstantCommand(() -> leftBumper = true));
        controller.leftBumper().onFalse(new InstantCommand(() -> leftBumper = false));
        controller.rightBumper().onTrue(new InstantCommand(() -> rightBumper = true));
        controller.rightBumper().onFalse(new InstantCommand(() -> rightBumper = false));

    }

    public boolean getA(){
        return a;
    }
    public boolean getB(){
        return b;
    }
    public boolean getX(){
        return x;
    }
    public boolean getY(){
        return y;
    }
    public boolean getLeftBumper(){
        return leftBumper;
    }
    public boolean getRightBumper(){
        return rightBumper;
    }


}
