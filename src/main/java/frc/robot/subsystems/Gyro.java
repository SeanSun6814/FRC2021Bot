package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private static Gyro instance;

    public static Gyro getInstance() {
        if (instance == null)
            instance = new Gyro();
        return instance;
    }

    // ----------\ singleton /----------/ main code \--------------

    private AHRS gyro;
    private double gyroZero = 0;

    private Gyro() {
        try {
            gyro = new AHRS(SPI.Port.kMXP);
            // gyro = new AHRS(SerialPort.Port.kUSB);
        } catch (RuntimeException ex) {
            System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
        }

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                reset();
                System.out.println("GYRO RESET!");
            } catch (Exception e) {
            }
        }).start();
    }

    public double getDeg() {
        return gyro.getAngle() - gyroZero;
    }

    public double getRad() {
        return Math.toRadians(getDeg());
    }

    public double getAbsDeg() {
        return getDeg() % 360;
    }

    public double getAbsRad() {
        return getRad() % (2 * Math.PI);
    }

    public double getAngVelDegPerSec() {
        return gyro.getRate();
    }

    public double getAngVelRadPerSec() {
        return Math.toRadians(gyro.getRate());
    }

    synchronized public void reset() {
        reset(0);
    }

    public Rotation2d getRotation2d() {
        SmartDashboard.putNumber("GyroZero", gyroZero);
        return Rotation2d.fromDegrees(getDeg());
    }

    public void reset(double setCurrentAngleToDeg) {
        // System.out.println("RESETTING GYRO!!! Current angle is" + gyro.getAngle());
        gyroZero = gyro.getAngle() + setCurrentAngleToDeg;
        // gyro.reset();
    }
}
