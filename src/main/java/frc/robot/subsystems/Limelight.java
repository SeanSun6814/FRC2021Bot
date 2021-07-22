// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Units;
import frc.robot.util.MeasurementSmoother;
import edu.wpi.first.networktables.*;

public class Limelight extends SubsystemBase {
    private static Limelight instance;

    public static Limelight getInstance() {
        if (instance == null)
            instance = new Limelight();
        return instance;
    }

    // ----------\ singleton /----------/ main code \--------------

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry ta = table.getEntry("ta");
    private final NetworkTableEntry tv = table.getEntry("tv");

    private final MeasurementSmoother smoothX = new MeasurementSmoother(LimelightConstants.kRunningAvgWindow), //
            smoothY = new MeasurementSmoother(LimelightConstants.kRunningAvgWindow), //
            smoothArea = new MeasurementSmoother(LimelightConstants.kRunningAvgWindow);

    private boolean enableXSetpoint = false, enableYSetpoint = false;
    private double xSetpoint = 0, ySetpoint = 0;

    private Limelight() {
        System.out.println("Limelight init");
    }

    @Override
    public void periodic() {
        smoothX.update(getNoisyX());
        smoothY.update(getNoisyY());
        smoothArea.update(getNoisyArea());
        updateTelemetry();
    }

    public double getNoisyX() {
        return tx.getDouble(0.0);
    }

    public double getNoisyY() {
        return ty.getDouble(0.0);
    }

    public double getNoisyArea() {
        return ta.getDouble(0.0);
    }

    public double getX() {
        return smoothX.getPosition();
    }

    public double getY() {
        return smoothY.getPosition();
    }

    public double getArea() {
        return smoothArea.getPosition();
    }

    public double getXVel() {
        return smoothX.getVelocity();
    }

    public double getYVel() {
        return smoothY.getVelocity();
    }

    public double getAreaVel() {
        return smoothArea.getVelocity();
    }

    public Translation2d getPoint() {
        return new Translation2d(getX(), getY());
    }

    public boolean isValid() {
        return Units.doubleEquals(tv.getDouble(0.0), 1.0);
    }

    private void updateTelemetry() {
        if (Constants.debug) {
            SmartDashboard.putNumber("LimelightX", getX());
            SmartDashboard.putNumber("LimelightY", getY());
            SmartDashboard.putNumber("LimelightArea", getArea());
            SmartDashboard.putNumber("LimelightXVel", getXVel());
            SmartDashboard.putNumber("LimelightYVel", getYVel());
            SmartDashboard.putNumber("LimelightAreaVel", getAreaVel());
            SmartDashboard.putBoolean("LimelightValid", isValid());
            SmartDashboard.putBoolean("LimelightOnTarget", onTarget());
        }
    }

    public void setSetpoint(double hSetpoint, double vSetpoint) {
        this.enableYSetpoint = true;
        this.enableXSetpoint = true;
        this.ySetpoint = vSetpoint;
        this.xSetpoint = hSetpoint;
    }

    public void setHSetpoint(double hSetpoint) {
        this.enableXSetpoint = true;
        this.enableYSetpoint = false;
        this.xSetpoint = hSetpoint;
    }

    public void setVSetpoint(double vSetpoint) {
        this.enableYSetpoint = true;
        this.enableXSetpoint = false;
        this.ySetpoint = vSetpoint;
    }

    public boolean onTarget() {
        if (!isValid())
            return false;

        if (enableYSetpoint) {
            if (Math.abs(getY() - ySetpoint) > LimelightConstants.kYPosToleranceDegs || //
                    Math.abs(getYVel()) > LimelightConstants.kYVelToleranceDegsPerSec) {
                return false;
            }
        }

        if (enableXSetpoint) {
            if (Math.abs(getX() - xSetpoint) > LimelightConstants.kXPosToleranceDegs || //
                    Math.abs(getXVel()) > LimelightConstants.kXVelToleranceDegsPerSec) {
                return false;
            }
        }

        return true;
    }

}

class MyClass {
    public static void main(String[] args) {
        var limelight = Limelight.getInstance();
        System.out.println(limelight.getX());
    }
}