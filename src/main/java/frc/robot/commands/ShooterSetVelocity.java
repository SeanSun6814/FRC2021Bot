package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterSetVelocity extends CommandBase {
    private final Shooter shooter = Shooter.getInstance();

    private Supplier<Double> rpm;

    public ShooterSetVelocity(double rpm) {
        this(() -> rpm);
    }

    public ShooterSetVelocity(Supplier<Double> rpm) {
        addRequirements(shooter);
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {
        shooter.setRPM(rpm.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getName() + " ending...");
    }
}
