package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class HoodSetPosition extends CommandBase {
    private final Hood hood = Hood.getInstance();

    private Supplier<Double> setpoint;

    public HoodSetPosition(double setpoint) {
        this(() -> setpoint);
    }

    public HoodSetPosition(Supplier<Double> setpoint) {
        addRequirements(hood);
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {
        hood.setPosition(setpoint.get());
        // hood.stop();
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
