// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Units;
import edu.wpi.first.networktables.*;

public class Limelight extends SubsystemBase {
    private static Limelight instance;

    public static Limelight getInstance() {
        if (instance == null)
            instance = new Limelight();
        return instance;
    }

    // ----------\ singleton /----------/ main code \--------------

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    private Limelight() {
        System.out.println("Constructing an instance of limelight!");
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public Translation2d getPoint() {
        return new Translation2d(getX(), getY());
    }

    public boolean isValid() {
        return Units.doubleEquals(tv.getDouble(0.0), 1.0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("LimelightX", getX());
        SmartDashboard.putNumber("LimelightY", getY());
        SmartDashboard.putNumber("LimelightArea", getArea());
        SmartDashboard.putBoolean("LimelightValid", isValid());
    }

}

class MyClass {
    public static void main(String[] args) {
        var limelight = Limelight.getInstance();
        System.out.println(limelight.getX());
    }
}