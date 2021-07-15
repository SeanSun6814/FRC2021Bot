package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class OuttakeBall extends CommandBase {
    private final Feeder feeder = Feeder.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();

    private Supplier<Boolean> confirmationButton;
    private int state;
    private double setpoint;
    private double delayStartTimestamp;

    public OuttakeBall(Supplier<Boolean> confirmationButton) {
        addRequirements(feeder);
        this.confirmationButton = confirmationButton;
    }

    @Override
    public void initialize() {
        state = 0;
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {

        if (feeder.isEmpty()) {
            feeder.stop();
            return;
        }

        if (state == 0)
            prepareBalls();
        if (state == 1)
            waitForSystemReady();
        else if (state == 2)
            loadBall();
    }

    private void prepareBalls() {
        feeder.setMotor(FeederConstants.kMotorIntakePower);
        if (feeder.getBallExitSensor()) {
            feeder.stop();
            state = 1;
        }
    }

    private void waitForSystemReady() {
        feeder.stop();
        if (hood.onTarget() && shooter.onTarget() && delayReady() && confirmationButton.get()) {
            setpoint = feeder.getEncoderPosition() + FeederConstants.kRotationsPerBall;
            state = 2;
        }
    }

    private void loadBall() {
        feeder.setPosition(setpoint);
        if (feeder.onTarget()) {
            // feeder.setBallCount(feeder.getBallCount() - 1);
            delayStartTimestamp = Timer.getFPGATimestamp();
            state = 0;
        }
    }

    private boolean delayReady() {
        return Timer.getFPGATimestamp() > delayStartTimestamp + FeederConstants.kOuttakeDelaySec;
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
