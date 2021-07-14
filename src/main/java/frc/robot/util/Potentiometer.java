package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class Potentiometer {
    private RunningAverage runningAverage;
    private AnalogInput encoder;
    private double kEncoderConstant;
    private double prevEncoderPosition = 0, prevTimestamp = 0;
    private double encoderVelocity = 0;

    public Potentiometer(int port, int runnningAverageWindow, double kEncoderConstant) {
        runningAverage = new RunningAverage(runnningAverageWindow);
        encoder = new AnalogInput(port);
        this.kEncoderConstant = kEncoderConstant;
    }

    public void update() {
        runningAverage.updateRunningAverage(getNoisyEncoderPosition());
        updateEncoderVelocity();
    }

    public double getEncoderPosition() {
        return runningAverage.getRunningAverage();
    }

    @Deprecated
    public double getNoisyEncoderPosition() {
        return encoder.getVoltage() / RobotController.getVoltage5V() * kEncoderConstant;
    }

    private void updateEncoderVelocity() {
        encoderVelocity = (getEncoderPosition() - prevEncoderPosition) / (Timer.getFPGATimestamp() - prevTimestamp);
        prevTimestamp = Timer.getFPGATimestamp();
        prevEncoderPosition = getEncoderPosition();
    }

    public double getEncoderVelocity() {
        return encoderVelocity;
    }
}
