package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED {

    private static LED instance;

    public static LED getInstance() {
        if (instance == null)
            instance = new LED();
        return instance;
    }

    private Spark motor;
    private double speed;
    private double startTime;
    private double time;
    private boolean flashing = false;

    private LED() {
        motor = new Spark(2);
    }

    public void setRed() {
        speed = 0.61;
    }

    public void setYellow() {
        speed = 0.69;
    }

    public void setGreen() {
        speed = 0.77;
    }

    public void setBlue() {
        speed = 0.83;
    }

    public void setOff() {
        speed = 0.99;
    }

    public void setFlashing(boolean f) {
        flashing = f;
        startTime = Timer.getFPGATimestamp();
    }

    public void periodic() {
        time = Timer.getFPGATimestamp();
        if ((time - startTime) % 1.5 < 0.75 || !flashing) {
            motor.set(speed);
            // System.out.println("setting leds");
        } else {
            motor.set(0.99);
        }
        SmartDashboard.putNumber("LED speed", speed);
    }
}
