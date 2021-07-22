package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class MeasurementSmoother {
    private double position, velocity, noisyPosition;
    private double prevTimestamp, prevPosition;
    private final RunningAverage runningAverage;

    public MeasurementSmoother(int avgWindow, double initialValue) {
        runningAverage = new RunningAverage(avgWindow);
        reset(initialValue);
    }

    public MeasurementSmoother(int avgWindow) {
        this(avgWindow, 0);
    }

    public void update(double newPosition) {
        noisyPosition = newPosition;
        position = runningAverage.updateRunningAverage(noisyPosition);
        updateVelocity(position);
    }

    private void updateVelocity(double newPosition) {
        velocity = (newPosition - prevPosition) / (Timer.getFPGATimestamp() - prevTimestamp);
        prevTimestamp = Timer.getFPGATimestamp();
        prevPosition = newPosition;
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getNoisyPosition() {
        return noisyPosition;
    }

    public void reset(double resetTo) {
        runningAverage.resetAverage(resetTo);
    }

}
