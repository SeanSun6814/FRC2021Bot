package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class OuttakeBall extends CommandBase {
    private final Feeder feeder = Feeder.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final LED led = LED.getInstance();

    private Supplier<Boolean> confirmationButton;
    private int state;
    private double setpoint;
    private double delayStartTimestamp;

    public OuttakeBall(Supplier<Boolean> confirmationButton) {
        addRequirements(feeder);
        this.confirmationButton = confirmationButton;
        updateTelemetry();
    }

    @Override
    public void initialize() {
        state = 0;

        led.setRed();
        led.setFlashing(false);
        System.out.println(getName() + " starting...");
    }

    @Override
    public void execute() {

        updateTelemetry();

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
        if (!delayReady())
            return;

        feeder.setMotor(FeederConstants.kMotorIntakePower);
        if (feeder.getBallExitSensor()) {
            feeder.stop();
            state = 1;
        }
    }

    private void waitForSystemReady() {
        feeder.stop();
        if (hood.onTarget() && shooter.onTarget() && limelight.onTarget() && confirmationButton.get()) {
            setpoint = feeder.getEncoderPosition() + FeederConstants.kRotationsPerBall;
            state = 2;
            led.setBlue();
            led.setFlashing(false);
        }
    }

    private void loadBall() {
        feeder.setPosition(setpoint);
        if (feeder.onTarget()) {
            // feeder.setBallCount(feeder.getBallCount() - 1);
            delayStartTimestamp = Timer.getFPGATimestamp();
            state = 0;
            led.setRed();
            led.setFlashing(false);
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

    private void updateTelemetry() {
        if (Constants.debug) {
            SmartDashboard.putBoolean("Shooter CMD Hood", hood.onTarget());
            SmartDashboard.putBoolean("Shooter CMD Flywheel", shooter.onTarget());
            SmartDashboard.putBoolean("Shooter CMD Delay", delayReady());
            SmartDashboard.putBoolean("Shooter CMD Joystick", confirmationButton.get());
        }
    }
}
