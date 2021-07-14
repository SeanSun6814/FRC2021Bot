package frc.robot.util;

public class BangBangController {

    private double motorPower;
    private double positionTolerance;
    private double velocityTolerance;

    private boolean oneSidedControl;

    private double sensor;
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

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public double getTolerance() {
        return positionTolerance;
    }

    public void setTolerance(double tolerance) {
        this.positionTolerance = tolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }

    public void setVelocityTolerance(double velocityTolerance) {
        this.velocityTolerance = velocityTolerance;
    }

    public boolean onTarget() {
        return (sensor > setpoint - positionTolerance) && ((sensor < setpoint + positionTolerance) || oneSidedControl);
    }
}