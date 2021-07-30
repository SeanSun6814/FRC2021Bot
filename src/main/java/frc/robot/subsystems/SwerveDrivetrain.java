// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Units;
import frc.robot.swerve.SwerveModule;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrivetrain extends SubsystemBase {
    private static SwerveDrivetrain instance;

    public static SwerveDrivetrain getInstance() {
        if (instance == null)
            instance = new SwerveDrivetrain();
        return instance;
    }

    // ----------\ singleton /----------/ main code \--------------

    private final SwerveModule frontLeft = new SwerveModule(//
            DriveConstants.kFrontLeftDriveMotorPort, //
            DriveConstants.kFrontLeftTurningMotorPort, //
            DriveConstants.kFrontLeftDriveEncoderReversed, //
            DriveConstants.kFrontLeftTurningEncoderReversed, //
            new AnalogInput(DriveConstants.kFrontLeftDriveAbsoluteEncoderPort), //
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, //
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(//
            DriveConstants.kBackLeftDriveMotorPort, //
            DriveConstants.kBackLeftTurningMotorPort, //
            DriveConstants.kBackLeftDriveEncoderReversed, //
            DriveConstants.kBackLeftTurningEncoderReversed, //
            new AnalogInput(DriveConstants.kBackLeftDriveAbsoluteEncoderPort), //
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, //
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(//
            DriveConstants.kFrontRightDriveMotorPort, //
            DriveConstants.kFrontRightTurningMotorPort, //
            DriveConstants.kFrontRightDriveEncoderReversed, //
            DriveConstants.kFrontRightTurningEncoderReversed, //
            new AnalogInput(DriveConstants.kFrontRightDriveAbsoluteEncoderPort), //
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, //
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(//
            DriveConstants.kBackRightDriveMotorPort, //
            DriveConstants.kBackRightTurningMotorPort, //
            DriveConstants.kBackRightDriveEncoderReversed, //
            DriveConstants.kBackRightTurningEncoderReversed, //
            new AnalogInput(DriveConstants.kBackRightDriveAbsoluteEncoderPort), //
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, //
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private Gyro gyro = Gyro.getInstance();

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d());

    private SwerveDrivetrain() {
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        updateTelemetry();
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, new Rotation2d());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getAngVelDegPerSec();
    }

    private void updateTelemetry() {
        SmartDashboard.putString("Gyro", getRotation2d().toString());

        SmartDashboard.putNumberArray("Robot X,Y,theta", new Double[] { //
                getPose().getX(), //
                getPose().getY(), //
                getPose().getRotation().getDegrees() //
        });

        SmartDashboard.putNumberArray("frontLeft [ud],[y]", new Double[] { //
                // frontLeft.getDesiredState().angle.getDegrees(), //
                // frontLeft.getDesiredState().speedMetersPerSecond, //
                frontLeft.getState().angle.getDegrees() % 360, //
                Math.toDegrees(Units.normalizeRad(frontLeft.getState().angle.getRadians())), //
                frontLeft.getState().speedMetersPerSecond, //
                frontLeft.getAbsoluteEncoderRad(), //
                Math.toDegrees(frontLeft.getAbsoluteEncoderRad()) //
        });

        SmartDashboard.putNumberArray("frontRight [ud],[y]", new Double[] { //
                // frontRight.getDesiredState().angle.getDegrees(), //
                // frontRight.getDesiredState().speedMetersPerSecond, //
                frontRight.getState().angle.getDegrees() % 360, //
                Math.toDegrees(Units.normalizeRad(frontRight.getState().angle.getRadians())), //
                frontRight.getState().speedMetersPerSecond, //
                frontRight.getAbsoluteEncoderRad(), //
                Math.toDegrees(frontRight.getAbsoluteEncoderRad()) //
        });

        SmartDashboard.putNumberArray("backLeft [ud],[y]", new Double[] { //
                // backLeft.getDesiredState().angle.getDegrees(), //
                // backLeft.getDesiredState().speedMetersPerSecond, //
                backLeft.getState().angle.getDegrees() % 360, //
                Math.toDegrees(Units.normalizeRad(backLeft.getState().angle.getRadians())), //
                backLeft.getState().speedMetersPerSecond, //
                backLeft.getAbsoluteEncoderRad(), //
                Math.toDegrees(backLeft.getAbsoluteEncoderRad()) //
        });

        SmartDashboard.putNumberArray("backRight [ud],[y]", new Double[] { //
                // backRight.getDesiredState().angle.getDegrees(), //
                // backRight.getDesiredState().speedMetersPerSecond, //
                backRight.getState().angle.getDegrees() % 360, //
                Math.toDegrees(Units.normalizeRad(backRight.getState().angle.getRadians())), //
                backRight.getState().speedMetersPerSecond, //
                backRight.getAbsoluteEncoderRad(), //
                Math.toDegrees(backRight.getAbsoluteEncoderRad()) //
        });
    }
}
