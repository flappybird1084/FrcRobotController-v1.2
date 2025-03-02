package frc.robot.util;
//basically a pid class but i can't comprehend wpilib garbage
public class ProportionalSlowdownController {
    double baseKp = 0.05; // initial proportional gain
    double errorScalingFactor = 0.05; // scaling factor for the error term

    public ProportionalSlowdownController() {

    }

    public double calculate(double input, double setpoint) {

        double error = input - setpoint;
        double adjustedKp = baseKp / (1 + errorScalingFactor * Math.abs(error));
        double output= -adjustedKp * error;

        if (Math.abs(output) < 0.1){
            output *=3;
        }
        return output;
    }

    public double calculate(double input, double setpoint, double tempBaseKp) {
        double error = input - setpoint;
        double adjustedKp = tempBaseKp / (1 + errorScalingFactor * Math.abs(error));
        double output = -adjustedKp * error;
        if (Math.abs(output) < 0.1){
            output *=3;
        }
        return output;
    }

}
