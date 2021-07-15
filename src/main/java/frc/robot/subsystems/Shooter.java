package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    private TalonFX motor1 = new TalonFX(ShooterConstants.kMotorPort1);
    private TalonFX motor2 = new TalonFX(ShooterConstants.kMotorPort2);

    private Shooter() {
        configMotor(motor1);
        configMotor(motor2);
    }

    private void configMotor(TalonFX motor) {
        motor.setInverted(ShooterConstants.kMotor1Inverted);

        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ShooterConstants.kPIDSlot,
                Constants.kCANTimeoutMS);

        motor.config_kP(ShooterConstants.kPIDSlot, ShooterConstants.kP, Constants.kCANTimeoutMS);
        motor.config_kI(ShooterConstants.kPIDSlot, ShooterConstants.kI, Constants.kCANTimeoutMS);
        motor.config_kD(ShooterConstants.kPIDSlot, ShooterConstants.kD, Constants.kCANTimeoutMS);
        motor.config_kF(ShooterConstants.kPIDSlot, ShooterConstants.kF, Constants.kCANTimeoutMS);

        motor.config_IntegralZone(ShooterConstants.kPIDSlot, ShooterConstants.kIZone, Constants.kCANTimeoutMS);

        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ShooterConstants.kCurrentLimitAmps,
                ShooterConstants.kCurrentLimitAmps, ShooterConstants.kCurrentLimitDelay));
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ShooterConstants.kCurrentLimitAmps,
                ShooterConstants.kCurrentLimitAmps, ShooterConstants.kCurrentLimitDelay));
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Operate the motors using percentage power
     * 
     * @param power percentage to set to motors
     */
    public void setMotor(double power) {
        motor1.set(ControlMode.PercentOutput, power);
        motor2.set(ControlMode.PercentOutput, power);
    }

    /**
     * Stop the motors
     */
    public void stop() {
        setMotor(0);
    }

    public void setRPM(double setpoint) {
        // System.out.println(setpoint / ShooterConstants.kEncoderTicksToRPM);
        motor1.set(ControlMode.Velocity, setpoint / ShooterConstants.kEncoderTicksToRPM);
        motor2.set(ControlMode.Velocity, setpoint / ShooterConstants.kEncoderTicksToRPM);
    }

    /**
     * Indicates whether the @see #intakeOneBall() command is finished. It is
     * considered finished if the PID position is within the set positional
     * tolerance and velocity is within the set velocity tolerance.
     * 
     * @return true if on target
     */
    public boolean onTarget() {
        return Math.abs(motor1.getClosedLoopError()) //
                < ShooterConstants.kMotorVelocityToleranceRPM / ShooterConstants.kEncoderTicksToRPM;
    }

    /*
     * 
     * @return the position of the motors in Rotations
     */
    public double getEncoderPosition() {
        return (motor1.getSelectedSensorPosition() * ShooterConstants.kEncoderTicksToRotations);
    }

    /**
     * 
     * @return the velocity of the motors in RPM
     */
    public double getEncoderVelocity() {
        return (motor1.getSelectedSensorVelocity() * ShooterConstants.kEncoderTicksToRPM);
    }

    /**
     * Resets the encoder position value to 0 meters
     */
    public void resetEncoder() {
        motor1.setSelectedSensorPosition(0);
    }

    /**
     * Send debug info to SmartDashboard. Make sure to call this function in @see
     * #periodic()
     */
    @SuppressWarnings("deprecation")
    private void updateTelemetry() {

        if (Constants.debug) {
            SmartDashboard.putNumber(getName() + " Encoder", getEncoderPosition());
            SmartDashboard.putBoolean(getName() + " On Target", onTarget());
            SmartDashboard.putNumber(getName() + " RPM", getEncoderVelocity());
            SmartDashboard.putNumber(getName() + " % Output1", motor1.getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + " % Output2", motor2.getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + " Current (Amps)", motor1.getOutputCurrent());
            SmartDashboard.putNumber(getName() + " Current1 (Amps)", motor2.getOutputCurrent());
            SmartDashboard.putNumber(getName() + " Voltage (Volts)", motor1.getMotorOutputVoltage());
            SmartDashboard.putNumber(getName() + " Voltage1 (Volts)", motor2.getMotorOutputVoltage());
            SmartDashboard.putNumber(getName() + " Temp1", motor1.getTemperature());
            SmartDashboard.putNumber(getName() + " Temp2", motor2.getTemperature());
        }

        if (Math.abs(motor1.getOutputCurrent() - motor2.getOutputCurrent()) > 20)
            DriverStation.reportError(getName() + "Motors current mismatch!", false);
        if (Math.abs(Math.abs(motor1.getMotorOutputVoltage()) - Math.abs(motor2.getMotorOutputVoltage())) > 2)
            DriverStation.reportError(getName() + "Motors voltage mismatch!", false);
        if (Math.abs(Math.abs(motor1.getMotorOutputPercent()) - Math.abs(motor2.getMotorOutputPercent())) > 2)
            DriverStation.reportError(getName() + "Motors outputs mismatch!", false);
        if (Math.abs(motor1.getTemperature() - motor2.getTemperature()) > 20)
            DriverStation.reportError(getName() + "Motors temperature mismatch!", false);

    }
}
