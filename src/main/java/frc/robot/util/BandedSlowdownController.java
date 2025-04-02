package frc.robot.util;


public class BandedSlowdownController {
    
    private double bottomPos;
    private double topPos;
    private double speedLimitOffset;

    public BandedSlowdownController(double bottomPos, double topPos, double speedLimitOffset) {
        this.bottomPos = bottomPos;
        this.topPos = topPos;
        this.speedLimitOffset = speedLimitOffset;
    }

    public double calculateAdjustedPower(double power, double currentPosition) {
        if (currentPosition < bottomPos + speedLimitOffset) {
            // Calculate how close we are to the lower limit and apply proportional power reduction
            double distanceFromBottom = currentPosition - bottomPos;
            double availableRange = speedLimitOffset;
            double ratio = (distanceFromBottom / availableRange);
            // return Math.max(power * ratio, 0.0); // Ensures non-negative power with smooth slowdown
            return power*ratio;
        } else if (currentPosition > topPos - speedLimitOffset) {
            // Calculate how close we are to the upper limit and apply proportional power reduction
            double distanceFromTop = currentPosition - (topPos - speedLimitOffset);
            double availableRange = speedLimitOffset;
            double ratio = (distanceFromTop / availableRange);
            // return Math.max(power * ratio, 0.0); // Ensures non-negative power with smooth slowdown
            return power*ratio;
        } else {
            // // Normal operating range with proportional power application
            // double normalizedPosition = (currentPosition - bottomPos) / (topPos - bottomPos);
            // return Math.max(power * normalizedPosition, 0.0);
            return power;
        }
    }
}
