package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Units;

public class MyPIDController {
    private final double kInfinity = Double.POSITIVE_INFINITY;
    private double kP = 0f, kI = 0f, kD = 0f, kILimit = kInfinity, kIMaxVal = kInfinity;
    private boolean kClearIntegrationOnErrorSignChange = false;
    private double kPosTolerance = kInfinity, kVelTolerance = kInfinity;
    private boolean kContinuous = false;
    private double kInputRange = kInfinity;

    private double setpoint = 0f;

    private double dt = 0f;
    private double error = 0f;
    private double errorRate = 0f;
    private double errorSum = 0f;
    private double outputSpeed = 0f;
    private double lastTimestamp = 0f;
    private double lastError = 0f;
    private double lastSensorPosition = 0f;

    /**
     * Some systems have the input range wrapped around. Ex: an angle motor that
     * goes back to 0 degs after 360 degs. Because of this, the control on these
     * systems may contain shortcuts. Ex: to go from 350 degs to 10 degs, you don't
     * have to spin 340 degs; instead, realize that you can wrap around from the
     * other side and only travel 20 degs. This function scans for these kind of
     * shortcuts and returns new error values that describe the motion plan for the
     * shortcut.
     * 
     * @param error the simple motion plan to get to setpoint
     * @return new motion plan (the error) to the setpoint
     */
    private double recalculateMotionUsingContinuous(double error) {
        error %= kInputRange;
        if (Math.abs(error) > kInputRange / 2.0) {
            if (error > 0.0) {
                error -= kInputRange;
            } else {
                error += kInputRange;
            }
        }
        return error;
    }

    public double calculate(double sensorPosition) {

        error = setpoint - sensorPosition;
        if (kContinuous)
            error = recalculateMotionUsingContinuous(error);

        dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < kILimit) {
            errorSum += error * dt;
            errorSum = Units.clamp(errorSum, -kIMaxVal / kI, kIMaxVal / kI);
            if (kClearIntegrationOnErrorSignChange && !Units.sameSign(error, lastError))
                errorSum = 0;
        }

        errorRate = (lastSensorPosition - sensorPosition) / dt; // prevent derivative kick

        outputSpeed = kP * error + kI * errorSum + kD * errorRate;

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        lastSensorPosition = sensorPosition;

        return outputSpeed;
    }

    public double getOutputSpeed() {
        return outputSpeed;
    }

    public double getError() {
        return error;
    }

    public double getErrorRate() {
        return errorRate;
    }

    public double getErrorSum() {
        return errorSum;
    }

    public void resetIntegration() {
        errorSum = 0;
    }

    public boolean onTarget() {
        return Math.abs(error) < kPosTolerance && Math.abs(errorRate) < kVelTolerance;
    }

    public boolean iskClearIntegrationOnErrorSignChange() {
        return kClearIntegrationOnErrorSignChange;
    }

    public void setkClearIntegrationOnErrorSignChange(boolean clearIntegrationOnErrorSignChange) {
        this.kClearIntegrationOnErrorSignChange = clearIntegrationOnErrorSignChange;
    }

    public double getkIMaxVal() {
        return kIMaxVal;
    }

    public void setkIMaxVal(double kIMaxVal) {
        this.kIMaxVal = kIMaxVal;
    }

    public double getkILimit() {
        return kILimit;
    }

    public void setkILimit(double kILimit) {
        this.kILimit = kILimit;
    }

    public double getkP() {
        return kP;
    }

    public double getVelTolerance() {
        return kVelTolerance;
    }

    public void setVelTolerance(double velTolerance) {
        this.kVelTolerance = velTolerance;
    }

    public double getPosTolerance() {
        return kPosTolerance;
    }

    public void setPosTolerance(double posTolerance) {
        this.kPosTolerance = posTolerance;
    }

    public void setTolerance(double posTolerance, double velTolerance) {
        this.kPosTolerance = posTolerance;
        this.kVelTolerance = velTolerance;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setPID(double kP, double kI, double kD, double iLimit) {
        setPID(kP, kI, kD);
        this.kILimit = iLimit;
    }

    public void setPID(double kP, double kI, double kD, double iLimit, double iMaxVal) {
        setPID(kP, kI, kD, iLimit);
        this.kIMaxVal = iMaxVal;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean isContinuous() {
        return kContinuous;
    }

    public void setContinuous(boolean kContinuous, double kInputRange) {
        this.kContinuous = kContinuous;
        this.kInputRange = kInputRange;

    }

    public double getInputRange() {
        return kInputRange;
    }

    public MyPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public MyPIDController(double kP, double kI, double kD, boolean continuous, double inputRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kContinuous = continuous;
        this.kInputRange = inputRange;
    }

    public MyPIDController(double kP, double kI, double kD, double kILimit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kILimit = kILimit;
    }

    public MyPIDController(double kP, double kI, double kD, double kILimit, double kIMaxVal) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kILimit = kILimit;
        this.kIMaxVal = kIMaxVal;
    }

    public MyPIDController(double kP, double kI, double kD, double kILimit, double kIMaxVal, double kPosTolerance,
            double kVelTolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kILimit = kILimit;
        this.kIMaxVal = kIMaxVal;
        this.kPosTolerance = kPosTolerance;
        this.kVelTolerance = kVelTolerance;
    }

}
