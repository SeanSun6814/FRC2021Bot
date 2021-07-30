package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
    private static Feeder instance;

    private TalonFX motor1 = new TalonFX(FeederConstants.kMotorPort1);
    private TalonFX motor2 = new TalonFX(FeederConstants.kMotorPort2);
    private DigitalInput feederInput = new DigitalInput(FeederConstants.kInputSensorPort);
    private DigitalInput feederOutput = new DigitalInput(FeederConstants.kOutputSensorPort);
    // private DoubleSolenoid solenoid = new DoubleSolenoid(//
    // FeederConstants.kPistonForwardChannel,
    // FeederConstants.kPistonReverseChannel);

    private PIDController controller = new PIDController(FeederConstants.kP, 0, 0);

    // private BangBangController contorller = new BangBangController(//
    // FeederConstants.kMaxMotorIntakePower, //
    // FeederConstants.kMotorIntakeBangBangPositionToleranceRotations,
    // FeederConstants.kMotorIntakeBangBangVelocityToleranceRPM);

    private int ballCount = 0;

    private double lastBallPositionAtInput;

    public static Feeder getInstance() {
        if (instance == null)
            instance = new Feeder();
        return instance;
    }

    private Feeder() {
        motor1.setInverted(FeederConstants.kmotor1Inverted);
        motor2.setInverted(FeederConstants.kmotor2Inverted);

        controller.setTolerance(FeederConstants.kMotorIntakePositionToleranceRotations,
                FeederConstants.kMotorIntakeVelocityToleranceRPM / 60);
    }

    @Override
    public void periodic() {
        updateLastBallPositionAtInput();
        updateTelemetry();
    }

    private void updateLastBallPositionAtInput() {
        if (getBallEntrySensor()) {
            lastBallPositionAtInput = getEncoderPosition();
        }
    }

    /**
     * There is a sensor at the beginning of the feeder indicating whether there is
     * a ball waiting to be indexed
     * 
     * @return whether the sensor currently sensed a ball
     */
    public boolean getBallEntrySensor() {
        return !feederInput.get();
    }

    public boolean getBallExitSensor() {
        return !feederOutput.get();
    }

    /**
     * Gets the lateset ball's position when it was at the input sensor
     * 
     * @return the encoder value of the ball in rotations
     */
    public double getLastBallPositionAtInput() {
        return lastBallPositionAtInput;
    }

    public boolean isEmpty() {
        return getEncoderPosition() > getLastBallPositionAtInput() + FeederConstants.kFeederLengthRotations;
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

    /**
     * Feeder moves forward by distance of one ball. Use PID or dead-reckoning to
     * spin motors by the diameter of a ball plus some margin.
     */
    public void setPosition(double setpoint) {
        double power = controller.calculate(getEncoderPosition(), setpoint);
        power = clamp(power, FeederConstants.kMotorIntakePower, -FeederConstants.kMotorIntakePower);
        setMotor(power);
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

    /*
     * 
     * @return the position of the feeder in Rotations
     */
    public double getEncoderPosition() {
        return (motor1.getSelectedSensorPosition() * FeederConstants.kEncoderTicksToRotations);
    }

    /**
     * 
     * @return the velocity of the feeder in RPM
     */
    public double getEncoderVelocity() {
        return (motor1.getSelectedSensorVelocity() * FeederConstants.kEncoderTicksToRPM);
    }

    /**
     * Resets the encoder position value to 0 meters
     */
    public void resetEncoder() {
        motor1.setSelectedSensorPosition(0);
    }

    /**
     * Set the position of the piston that blocks the ball from the shooter
     * flywheel.
     * 
     * @param block Request to block the feeder if true, unblock the feeder if
     *              false.
     */
    public void setHardStopPiston(boolean block) {
        // boolean direction = block ^ FeederConstants.kPistonInverted;
        // solenoid.set((direction) ? Value.kForward : Value.kReverse);
    }

    public int getBallCount() {
        return ballCount;
    }

    public void setBallCount(int ballCount) {
        this.ballCount = ballCount;
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
            SmartDashboard.putNumber(getName() + " Setpoint", controller.getSetpoint());
            SmartDashboard.putBoolean(getName() + " In IR Sensor", getBallEntrySensor());
            SmartDashboard.putBoolean(getName() + " Out IR Sensor", getBallExitSensor());
            SmartDashboard.putNumber(getName() + " RPM", getEncoderVelocity());
            SmartDashboard.putNumber(getName() + " % Output1", motor1.getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + " % Output2", motor2.getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + " Current (Amps)", motor1.getOutputCurrent());
            SmartDashboard.putNumber(getName() + " Current1 (Amps)", motor2.getOutputCurrent());
            SmartDashboard.putNumber(getName() + " Voltage (Volts)", motor1.getMotorOutputVoltage());
            SmartDashboard.putNumber(getName() + " Voltage1 (Volts)", motor2.getMotorOutputVoltage());
            SmartDashboard.putNumber(getName() + " Temp1", motor1.getTemperature());
            SmartDashboard.putNumber(getName() + " Temp2", motor2.getTemperature());
            SmartDashboard.putBoolean(getName() + " isEmpty", isEmpty());
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

    private double clamp(double value, double max, double min) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }
}
