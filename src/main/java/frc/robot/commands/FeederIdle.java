package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeederIdle extends CommandBase {
    private final Feeder feeder = Feeder.getInstance();
    private final Joystick joy1;

    public FeederIdle(Joystick joy1) {
        addRequirements(feeder);
        this.joy1 = joy1;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {
        // feeder.stop();
        feeder.setMotor(joy1.getRawAxis(1) * 0.5);
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
