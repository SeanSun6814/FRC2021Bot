package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Limelight;

public class ShootCmd extends ParallelCommandGroup {
    Limelight limelight = Limelight.getInstance();

    public ShootCmd(double hoodPosition, double shooterRPM, double limelightX, double limelightY,
            boolean enableLimelightX, boolean enableLimelightY, Supplier<Boolean> confirmationButton) {
        addCommands( //

                new HoodSetPosition(hoodPosition), //
                new ShooterSetVelocity(shooterRPM), //
                new OuttakeBall(confirmationButton), //
                new SwerveAim(limelightX, limelightY, enableLimelightX, enableLimelightY) //
        );
    }

    public ShootCmd(double limelightX, boolean enableLimelightX, Supplier<Boolean> confirmationButton) {
        addCommands( //

                new HoodSetPosition(() -> calcHoodPos(limelight.getY())), //
                new ShooterSetVelocity(() -> calcShooterRPM(limelight.getY())), //
                new OuttakeBall(confirmationButton), //
                new SwerveAim(limelightX, 0, enableLimelightX, false) //
        );
    }

    private double calcShooterRPM(double limelightAngle) {
        // return Math.min(1200 + 150 * (-limelightAngle + 14.3) / 17.3, 1450);
        return Math.min(-7 * limelightAngle + 1315, 1450);
    }

    private double calcHoodPos(double limelightAngle) {
        return Math.min(1.8 + 0.5 * (-limelightAngle + 14.3) / 17.3, 2.4);
    }

    public static void main(String[] args) {
        // System.out.println(calcShooterRPM(-3));
        // System.out.println(calcShooterRPM(0));
        // System.out.println(calcShooterRPM(14.3));

        // System.out.println(calcHoodPos(-3));
        // System.out.println(calcHoodPos(0));
        // System.out.println(calcHoodPos(14.3));
    }

}
