package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

public class IntakeBall extends CommandBase {
    private final Feeder feeder = Feeder.getInstance();

    private int state;
    private double setpoint;

    public IntakeBall() {
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        state = 0;
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {

        if (feeder.getBallExitSensor()) {
            feeder.stop();
            return; // put sequence on hold
        }

        if (state == 0)
            waitForBall();
        else if (state == 1)
            intakeBall();
    }

    private void waitForBall() {
        if (feeder.getBallEntrySensor()) {
            // if (feeder.getBallCount() < 5) {
            setpoint = feeder.getEncoderPosition() + FeederConstants.kRotationsPerBall;
            state = 1;
            // } else {
            // DriverStation.reportError("Cannot intake, too many balls!", false);
            // }
        }
    }

    private void intakeBall() {
        feeder.setPosition(setpoint);
        if (feeder.onTarget()) {
            // feeder.setBallCount(feeder.getBallCount() + 1);
            state = 0;
        }
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
