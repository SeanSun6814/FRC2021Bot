package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShootCmd extends ParallelCommandGroup {
    public ShootCmd(double hoodPosition, double shooterRPM) {
        this(hoodPosition, shooterRPM, () -> true);
    }

    public ShootCmd(double hoodPosition, double shooterRPM, Supplier<Boolean> confirmationButton) {
        addCommands( //
                new HoodSetPosition(hoodPosition), //
                new ShooterSetVelocity(shooterRPM), //
                new OuttakeBall(confirmationButton) //
        );
    }

}
