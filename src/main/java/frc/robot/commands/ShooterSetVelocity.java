package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterSetVelocity extends CommandBase {
    private final Shooter shooter = Shooter.getInstance();

    private double rpm;

    public ShooterSetVelocity(double rpm) {
        addRequirements(shooter);
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {
        shooter.setRPM(rpm);
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
