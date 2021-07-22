// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final AnalogInput absoluteEncoder;
    private final double absoluteEncoderOffset;
    private final boolean absoluteEncoderReversed;
    private final boolean driveEncoderReversed;
    private final boolean turningEncoderReversed;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder turningEncoder;

    private final CANPIDController drivePIDController;
    private final CANPIDController turningPIDController;

    private SwerveModuleState desiredState = new SwerveModuleState();

    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveEncoderReversed,
            boolean turningEncoderReversed, AnalogInput absoluteEncoder, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {

        this.absoluteEncoder = absoluteEncoder;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        this.driveEncoderReversed = driveEncoderReversed;
        this.turningEncoderReversed = turningEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        // driveMotor.setInverted(isInverted);

        driveMotor.setOpenLoopRampRate(ModuleConstants.kDriveRampSec2FullThrottle);
        turningMotor.setOpenLoopRampRate(ModuleConstants.kTurningRampSec2FullThrottle);
        driveMotor.setClosedLoopRampRate(ModuleConstants.kDriveRampSec2FullThrottle);
        turningMotor.setClosedLoopRampRate(ModuleConstants.kTurningRampSec2FullThrottle);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        drivePIDController = driveMotor.getPIDController();
        turningPIDController = turningMotor.getPIDController();

        drivePIDController.setFeedbackDevice(driveEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        drivePIDController.setP(ModuleConstants.kDrivePID[0]);
        drivePIDController.setI(ModuleConstants.kDrivePID[1]);
        drivePIDController.setD(ModuleConstants.kDrivePID[2]);
        drivePIDController.setIZone(ModuleConstants.kDrivePID[3]);
        drivePIDController.setFF(ModuleConstants.kDrivePID[4]);
        drivePIDController.setOutputRange(ModuleConstants.kDrivePID[5], ModuleConstants.kDrivePID[6]);

        // turningPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, slotID)

        // TODO: adjust CANTimeouts and update rates

        turningPIDController.setP(ModuleConstants.kTurningPID[0]);
        turningPIDController.setI(ModuleConstants.kTurningPID[1]);
        turningPIDController.setD(ModuleConstants.kTurningPID[2]);
        turningPIDController.setIZone(ModuleConstants.kTurningPID[3]);
        turningPIDController.setFF(ModuleConstants.kTurningPID[4]);
        turningPIDController.setOutputRange(ModuleConstants.kTurningPID[5], ModuleConstants.kTurningPID[6]);

        resetEncoders();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition() * (driveEncoderReversed ? -1.0 : 1.0);
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition() * (turningEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity() * (driveEncoderReversed ? -1.0 : 1.0);
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity() * (turningEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.0001) {
            stop();
            return;
        }
        SmartDashboard.putString("SwerveBef[" + absoluteEncoder.getChannel() + "] state", state.toString());
        // System.out.println("SwerveBef[" + absoluteEncoder.getChannel() + "] " +
        // state.angle.getDegrees() + ", "
        // + Math.toDegrees(getTurningPosition()));
        state = optimizeMotion(state);
        desiredState = state;
        // SmartDashboard.putString("SwerveOpt[" + absoluteEncoder.getChannel() + "]
        // state", state.toString());
        // drivePIDController.setReference(state.speedMetersPerSecond *
        // (driveEncoderReversed ? -1.0 : 1.0),
        // ControlType.kVelocity);
        SmartDashboard.putNumber("Actual motor outputs",
                state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
        drivePIDController.setReference(state.speedMetersPerSecond * 1.3 / DriveConstants.kMaxSpeedMetersPerSecond
                * (driveEncoderReversed ? -1.0 : 1.0), ControlType.kDutyCycle);
        turningPIDController.setReference(state.angle.getRadians() * (turningEncoderReversed ? -1.0 : 1.0),
                ControlType.kPosition);
    }

    public static void main(String[] args) {
        for (var j = -7300; j < 7300; j++) {
            var startingAngle = Math.toRadians(j);

            for (var i = 0; i < 360; i++) {
                var input = i;
                var desiredState = new SwerveModuleState(1, Rotation2d.fromDegrees(input));
                desiredState = optimizeMotion(desiredState, startingAngle);
                if (j == -370) {
                    System.out.print("Starting angle: " + Math.toDegrees(startingAngle));
                    System.out.print("    Setpoint: " + input + " ");
                    System.out.println(desiredState);
                }
                var travel = desiredState.angle.getDegrees() - Math.toDegrees(startingAngle);
                if (Math.abs(travel) > 90.001) {
                    System.out.println(i + " " + j);
                }
            }
        }
        System.out.println("Done!");
    }

    /**
     * This function was for testing purposes. Feel free to delete me.
     * 
     * @param targetState
     * @param angle
     * @return
     */
    private static SwerveModuleState optimizeMotion(SwerveModuleState targetState, double angle) {
        double targetAngle = Units.normalizeRad(targetState.angle.getRadians());
        double targetSpeed = targetState.speedMetersPerSecond;
        double absAngle = Units.normalizeRad(angle);

        // step 1
        double delta = targetAngle - absAngle;
        if (delta > Math.PI) {
            targetAngle -= 2.0 * Math.PI;
        } else if (delta < -Math.PI) {
            targetAngle += 2.0 * Math.PI;
        }

        // step 2
        delta = targetAngle - absAngle;
        if (delta > Math.PI / 2.0) {
            targetAngle -= Math.PI;
            targetSpeed *= -1.0;
        } else if (delta < -Math.PI / 2.0) {
            targetAngle += Math.PI;
            targetSpeed *= -1.0;
        }

        // step 3
        delta = targetAngle - absAngle;
        return new SwerveModuleState(targetSpeed, new Rotation2d(angle + delta));
    }

    /**
     * Optimizes the angle setpoint in 2 different ways to guarantee the travel
     * distance is < 90degs. (1) If the setpoint is > 180degs away, then just spin
     * in the opposite direction to go to the same angle with less travel. (2) If
     * the setpoint is > 90 degs away, then invert the drive motor's speed and spin
     * the other way to get the same result.
     * 
     * This function also converts absolute angle setpoints into continuous
     * setpoints for the SparkMax PID controller
     * 
     * @param targetState the unoptimized target state
     * @return the optimized target state
     */
    private SwerveModuleState optimizeMotion(SwerveModuleState targetState) {
        double angle = getState().angle.getRadians(); // All real numbers
        double targetAngle = Units.normalizeRad(targetState.angle.getRadians());
        double targetSpeed = targetState.speedMetersPerSecond;
        double absAngle = Units.normalizeRad(angle);

        // step 1
        double delta = targetAngle - absAngle;
        if (delta > Math.PI) {
            targetAngle -= 2.0 * Math.PI;
        } else if (delta < -Math.PI) {
            targetAngle += 2.0 * Math.PI;
        }

        // step 2
        delta = targetAngle - absAngle;
        if (delta > Math.PI / 2.0) {
            targetAngle -= Math.PI;
            targetSpeed *= -1.0;
        } else if (delta < -Math.PI / 2.0) {
            targetAngle += Math.PI;
            targetSpeed *= -1.0;
        }

        // step 3
        delta = targetAngle - absAngle;
        // SmartDashboard.putNumber("SwerveDelta[" + absoluteEncoder.getChannel() + "]",
        // delta);

        return new SwerveModuleState(targetSpeed, new Rotation2d(angle + delta));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void resetEncoders() {
        resetDriveEncoder(0);
        resetTurningEncoder(Units.normalizeRad(getAbsoluteEncoderRad() - absoluteEncoderOffset));
    }

    public void resetDriveEncoder(double toMeters) {
        driveEncoder.setPosition(toMeters * (driveEncoderReversed ? -1.0 : 1.0));
    }

    public void resetTurningEncoder(double toRad) {
        turningEncoder.setPosition(toRad * (turningEncoderReversed ? -1.0 : 1.0));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public double getAbsoluteEncoderRad() {
        double angle = (1.0 - absoluteEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
        // angle += absoluteEncoderOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0)
            angle += 2.0 * Math.PI;

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

}
