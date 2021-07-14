package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class BangBangController {

    private double motorPower;
    private double positionTolerance;
    private double velocityTolerance;

    private boolean oneSidedControl;

    private double sensor;
    private double prevSensor;
    private double prevTimestamp;
    private double velocity;
    private double setpoint;

    public BangBangController(double motorPower, double positionTolerance, double velocityTolerance,
            boolean oneSidedControl) {
        this.motorPower = motorPower;
        this.positionTolerance = positionTolerance;
        this.oneSidedControl = oneSidedControl;
    }

    public BangBangController(double motorPower, double positionTolerance, double velocityTolerance) {
        this(motorPower, positionTolerance, velocityTolerance, false);
    }

    public double calculate(double sensor) {
        this.sensor = sensor;
        double dMin = (Timer.getFPGATimestamp() - prevTimestamp) / 60.0;
        velocity = (sensor - prevSensor) / dMin;
        prevSensor = sensor;
        prevTimestamp = Timer.getFPGATimestamp();

        if (sensor < setpoint - positionTolerance)
            return motorPower;
        else if ((sensor > setpoint + positionTolerance) && !oneSidedControl)
            return -motorPower;
        else
            return 0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getSensor() {
        return sensor;
    }

    public double getMotorPower() {
        return motorPower;
    }

    public double getTolerance() {
        return positionTolerance;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setTolerance(double tolerance) {
        this.positionTolerance = tolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }

    /**
     * Set velocity tolerance in RPM
     * 
     * @param velocityTolerance in RPM
     */
    public void setVelocityTolerance(double velocityTolerance) {
        this.velocityTolerance = velocityTolerance;
    }

    public boolean onTarget() {
        return (sensor > setpoint - positionTolerance) //
                && ((sensor < setpoint + positionTolerance) || oneSidedControl) //
                && Math.abs(velocity) < velocityTolerance;
    }
}