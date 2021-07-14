package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterIdle extends CommandBase {
    private final Shooter shooter = Shooter.getInstance();
    private final Joystick joy1;

    public ShooterIdle(Joystick joy1) {
        addRequirements(shooter);
        this.joy1 = joy1;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {
        // shooter.stop();
        shooter.setMotor(joy1.getRawAxis(1) * 0.5);
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
