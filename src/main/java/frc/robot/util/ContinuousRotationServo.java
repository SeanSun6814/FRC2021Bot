package frc.robot.util;

import edu.wpi.first.wpilibj.PWMSpeedController;

public class ContinuousRotationServo extends PWMSpeedController {
    private final double kMargin = 0.005;

    public ContinuousRotationServo(int channel, double deadband) {
        super(channel);
        setBounds(0.5 - kMargin, 1.5, 2.5 + kMargin, deadband);
    }

    private void setBounds(double min, double center, double max, double deadband) {
        double lowerRange = Math.abs(center - min), upperRange = Math.abs(center - max);
        double lowerDeadbandValue = center - lowerRange * deadband;
        double upperDeadbandValue = center + upperRange * deadband;
        super.setBounds(max, upperDeadbandValue, center, lowerDeadbandValue, min);
    }
}