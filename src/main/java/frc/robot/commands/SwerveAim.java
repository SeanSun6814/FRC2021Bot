package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.MyPIDController;

public class SwerveAim extends CommandBase {
    private final SwerveDrivetrain swerveDrivetrain = SwerveDrivetrain.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final SwerveTeleDrive swerveTeleDrive;
    private final MyPIDController xController, yController;
    // private boolean enableXSetpoint, enableYSetpoint;
    // private double xSetpoint, ySetpoint;
    private final double normalizePanSpdDriverToFull, normalizeRotSpdDriverToFull;

    public SwerveAim(double xSetpoint, double ySetpoint, boolean enableXSetpoint, boolean enableYSetpoint) {
        addRequirements(swerveDrivetrain);
        // this.xSetpoint = xSetpoint;
        // this.ySetpoint = ySetpoint;
        // this.enableXSetpoint = true;
        // this.enableYSetpoint = true;

        // Because this command drives the swerve through the tele-drive command,
        // the output gets automatically mapped to the slower "user max speed" settings.
        // While this speed cap is fine (since it shouldn't need to go faster), the PID
        // constants will change every time the user changes the "user max speed"
        // numbers. By normalizing it here, the PID constants will remain constant.
        normalizePanSpdDriverToFull = DriveConstants.kMaxSpeedMetersPerSecond
                / DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        normalizeRotSpdDriverToFull = DriveConstants.kMaxAngularSpeedRadiansPerSecond
                / DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        xController = new MyPIDController( //
                LimelightConstants.kXPIDIZone[0] * normalizeRotSpdDriverToFull, //
                LimelightConstants.kXPIDIZone[1] * normalizeRotSpdDriverToFull, //
                LimelightConstants.kXPIDIZone[2] * normalizeRotSpdDriverToFull, //
                LimelightConstants.kXPIDIZone[3] //
        );
        yController = new MyPIDController( //
                LimelightConstants.kYPIDIZone[0] * normalizePanSpdDriverToFull, //
                LimelightConstants.kYPIDIZone[1] * normalizePanSpdDriverToFull, //
                LimelightConstants.kYPIDIZone[2] * normalizePanSpdDriverToFull, //
                LimelightConstants.kYPIDIZone[3] //
        );

        xController.setkClearIntegrationOnErrorSignChange(true);
        yController.setkClearIntegrationOnErrorSignChange(true);

        xController.setSetpoint(xSetpoint);
        yController.setSetpoint(ySetpoint);

        swerveTeleDrive = new SwerveTeleDrive( //
                () -> { // y speed
                    if (!enableYSetpoint)
                        return 0.0;
                    if (!limelight.isValid())
                        return 0.0;
                    return yController.calculate(limelight.getY()); // TODO: inverted?
                }, //
                () -> 0.0, //
                () -> {
                    if (!enableXSetpoint)
                        return 0.0;
                    if (!limelight.isValid()) // If not found, then scan around
                        return LimelightConstants.kScanningSpeed * normalizeRotSpdDriverToFull;
                    return -xController.calculate(limelight.getX());
                }, //
                false);

        // swerveTeleDrive = new SwerveTeleDrive( //
        // () -> limelight.isValid() ? -limelight.getY() / 20.0 / 1 : 0.0, //
        // () -> 0.0, //
        // () -> limelight.isValid() ? limelight.getX() / 27.0 / 4 : 0.0, //
        // false);
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " command started.");
        swerveDrivetrain.stopModules();
        swerveTeleDrive.initialize();
        // Calling initialize doesn't activates tele-drive's required subsystems, but
        // that's okay because SwerveAim requires it instead.
    }

    @Override
    public void execute() {
        swerveTeleDrive.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveTeleDrive.end(interrupted);
        swerveDrivetrain.stopModules();
        System.out.println(getName() + " command ended, interrupted=[" + interrupted + "].");
    }

    @Override
    public boolean isFinished() {
        return swerveTeleDrive.isFinished();
    }

}