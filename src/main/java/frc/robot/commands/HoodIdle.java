package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class HoodIdle extends CommandBase {
    private final Hood hood = Hood.getInstance();
    private final Joystick joy1;

    public HoodIdle(Joystick joy1) {
        addRequirements(hood);
        this.joy1 = joy1;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {
        // hood.stop();
        hood.setMotor(joy1.getRawAxis(1) * 0.5);
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
