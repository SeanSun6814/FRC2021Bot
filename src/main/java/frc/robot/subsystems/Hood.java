package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.ContinuousRotationServo;
import frc.robot.util.Potentiometer;

public class Hood extends SubsystemBase {
    private static Hood instance;

    private ContinuousRotationServo servo1 = new ContinuousRotationServo(HoodConstants.kMotorPort1,
            HoodConstants.kDeadband);
    private ContinuousRotationServo servo2 = new ContinuousRotationServo(HoodConstants.kMotorPort2,
            HoodConstants.kDeadband);
    private Potentiometer encoder = new Potentiometer(HoodConstants.kPotentiometerPort,
            HoodConstants.kPotentiometerAvgWindow, HoodConstants.kPercentageToRotations);
    private PIDController controller = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

    private double power = 0;

    public static Hood getInstance() {
        if (instance == null)
            instance = new Hood();
        return instance;
    }

    private Hood() {
        controller.setTolerance(//
                HoodConstants.kPositionToleranceRotations, HoodConstants.kVelocityToleranceRPM);
    }

    @Override
    public void periodic() {
        encoder.update();
        updateTelemetry();
    }

    /**
     * Operate the motors using percentage power
     * 
     * @param power percentage to set to motors
     */
    public void setMotor(double power) {

        if (power > HoodConstants.kMaxPowerLimit)
            power = HoodConstants.kMaxPowerLimit;
        else if (power < -HoodConstants.kMaxPowerLimit)
            power = -HoodConstants.kMaxPowerLimit;

        if (Math.abs(power) < HoodConstants.kDeadband)
            power = 0;

        servo1.set(power);
        servo2.set(-power);
        this.power = power;
    }

    /**
     * Stop the motors
     */
    public void stop() {
        setMotor(0);
    }

    public void setPosition(double setpoint) {
        setMotor(controller.calculate(getEncoderPosition(), setpoint));
    }

    /**
     * Indicates whether the @see #intakeOneBall() command is finished. It is
     * considered finished if the PID position is within the set positional
     * tolerance and velocity is within the set velocity tolerance.
     * 
     * @return true if on target
     */
    public boolean onTarget() {
        return controller.atSetpoint();
    }

    public double getEncoderPosition() {
        return encoder.getEncoderPosition();
    }

    /**
     * 
     * @return the velocity of the motors in RPM
     */
    public double getEncoderVelocity() {
        return encoder.getEncoderVelocity();
    }

    /**
     * Send debug info to SmartDashboard. Make sure to call this function in @see
     * #periodic()
     */
    @SuppressWarnings("deprecation")
    private void updateTelemetry() {
        SmartDashboard.putNumber(getName() + " Encoder", getEncoderPosition());
        SmartDashboard.putNumber(getName() + " Setpoint", controller.getSetpoint());
        SmartDashboard.putBoolean(getName() + " On Target", onTarget());
        SmartDashboard.putNumber(getName() + " RPM", getEncoderVelocity());
        SmartDashboard.putNumber(getName() + " % Output", power);
        SmartDashboard.putNumber(getName() + " Potentiometer", encoder.getNoisyEncoderPosition());
        SmartDashboard.putNumber(getName() + " Denoised Potentiometer", getEncoderPosition());

    }
}
