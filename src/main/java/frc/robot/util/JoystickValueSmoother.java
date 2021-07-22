package frc.robot.util;

public class JoystickValueSmoother {
    private double previousValue = 0;

    private final double deltaInc;
    private final double deltaDec;
    private final double deadbandVal;

    public JoystickValueSmoother(double timeToFullPower, double timeToZeroPower, double fullPower, double updateFreq,
            double deadband) {
        deltaInc = fullPower / updateFreq / timeToFullPower;
        deltaDec = fullPower / updateFreq / timeToZeroPower;
        this.deadbandVal = deadband;
    }

    private double deadband(double value) {
        return Math.abs(value) > deadbandVal ? value : 0;
    }

    public double calc(double value) {
        value = deadband(value);
        // SmartDashboard.putNumber("JoySmootherReceivedValue", value);

        if (value > 0) {
            if (value - previousValue > deltaInc) {
                value = previousValue + deltaInc;
            } else if (value - previousValue < -deltaDec) {
                value = previousValue - deltaDec;
            }
        } else {
            if (value - previousValue > deltaDec) {
                value = previousValue + deltaDec;
            } else if (value - previousValue < -deltaInc) {
                value = previousValue - deltaInc;
            }
        }

        previousValue = value;

        return value;
    }
}