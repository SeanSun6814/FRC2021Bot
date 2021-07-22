package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.JoystickValueSmoother;

public class SwerveTeleDrive extends CommandBase {
    private final SwerveDrivetrain swerveDrivetrain = SwerveDrivetrain.getInstance();
    private final Supplier<Double> xSpd;
    private final Supplier<Double> ySpd;
    private final Supplier<Double> rotSpd;
    private final boolean fieldOriented;

    private final JoystickValueSmoother xSmooth, ySmooth, rotSmooth;

    public SwerveTeleDrive(double xSpd, double ySpd, double angSpeed, boolean fieldOriented) {
        this(() -> xSpd, () -> ySpd, () -> angSpeed, fieldOriented);
    }

    public SwerveTeleDrive(Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> angSpeed,
            boolean fieldOriented) {
        this.xSpd = xSpd;
        this.ySpd = ySpd;
        this.rotSpd = angSpeed;
        this.fieldOriented = fieldOriented;

        this.xSmooth = new JoystickValueSmoother(1, 1, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 50,
                OIConstants.kDeadband);
        this.ySmooth = new JoystickValueSmoother(1, 1, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 50,
                OIConstants.kDeadband);
        this.rotSmooth = new JoystickValueSmoother(1, 0.5, DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, 50,
                OIConstants.kDeadband);

        addRequirements(swerveDrivetrain);
    }

    @Override
    public void initialize() {
        swerveDrivetrain.stopModules();
        System.out.println(getName() + " command started.");
    }

    @Override
    public void execute() {
        // xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        // ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        // rotSpd *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;
        var swerveModuleStates = calcDrive(//
                xSmooth.calc(xSpd.get() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond), //
                ySmooth.calc(ySpd.get() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond), //
                rotSmooth.calc(rotSpd.get() * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond), //
                fieldOriented);
        swerveDrivetrain.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrivetrain.stopModules();
        System.out.println(getName() + " command ended, interrupted=[" + interrupted + "].");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private SwerveModuleState[] calcDrive(double xSpeed, double ySpeed, double rotSpd, boolean fieldRelative) {
        SmartDashboard.putNumber("xJoy", xSpeed);
        SmartDashboard.putNumber("yJoy", ySpeed);
        SmartDashboard.putNumber("rotJoy", rotSpd);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpd, swerveDrivetrain.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpd));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        return swerveModuleStates;
    }

}