package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final int kCANTimeoutMS = 10;
    public static final boolean debug = true;
    public static double kInf = 1e8;

    public static final class DriveConstants {
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -4.811;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.816;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.252;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.254;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 21 * Units.in2m;
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 25.5 * Units.in2m;

        // much more intuitive versions of trackwidth and wheelbase
        public static final double kHWidth = kTrackWidth;
        public static final double kVWidth = kWheelBase;
        public static final double kRadius = Math.sqrt(kHWidth * kHWidth + kVWidth * kVWidth); // 0.8390671404195545

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kMaxAngularSpeedRadiansPerSecond / 2;

    }

    public static final class LimelightConstants {
        public static final int kRunningAvgWindow = 3;
        public static final double[] kXPIDIZone = { 1 / 27.0 / 4, 0, 0.001, 0 }; // TODO:
        public static final double[] kYPIDIZone = { 1 / 20.0 / 1, 0, 0.001, 0 };// TODO:

        /**
         * FOR REFERENCE: new SwerveTeleDrive( () -> limelight.isValid() ?
         * -limelight.getY() / 20.0 / 1 : 0.0, () -> 0.0, () -> limelight.isValid() ?
         * limelight.getX() / 27.0 / 4 : 0.0, false)
         */
        public static final double kXPosToleranceDegs = 0.3;
        public static final double kXVelToleranceDegsPerSec = 0.01;
        public static final double kYPosToleranceDegs = 0.3;
        public static final double kYVelToleranceDegsPerSec = 0.01;
        public static final double kScanningSpeed = 0.2;
    }

    public static final class ModuleConstants {
        public static final double kEncoderTicksPerRotation = 42.0;
        public static final double kWheelDiameterMeters = 4 * Units.in2m;
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final double kDriveRampSec2FullThrottle = 1 / kInf;
        public static final double kTurningRampSec2FullThrottle = 1 / kInf;

        // P, I, D, IZone, FF, min, max
        public static final double[] kTurningPID = { 0.5, 0, 0, 1e8, 0, -1, 1 };
        public static final double[] kDrivePID = { 0.5, 0, 0, 1e8, 0, -1, 1 };
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final boolean kDriverYAxisInverted = true;
        public static final int kDriverXAxis = 0;
        public static final boolean kDriverXAxisInverted = false;
        public static final int kDriverRotAxis = 4;
        public static final boolean kDriverRotAxisInverted = false;

        public static final double kDeadband = 0.05;

    }

    public static final class AutoConstants {
        // 1.2 for fast, 5 for slow spinning
        public static final double kPXController = 1.5;// 5;
        public static final double kPYController = 1.5;// 5;
        public static final double kPThetaController = 3;

        // // Constraint for the motion profilied robot angle controller
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        // = //
        // new TrapezoidProfile.Constraints(//
        // DriveConstants.kMaxAngularSpeedRadiansPerSecond, //
        // DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared//
        // );

        // public static final TrajectoryConfig trajectoryConfig = new
        // TrajectoryConfig(//
        // DriveConstants.kMaxSpeedMetersPerSecond / 2, //
        // DriveConstants.kMaxAccelerationMetersPerSecondSquared / 2)//
        // .setKinematics(DriveConstants.kDriveKinematics);
    }

    public class DriverConstants {
        public static final double kHoodAngle1 = 1.4;
        public static final double kHoodAngle2 = 2.3;
        public static final double kShooterRPM1 = 2050;
        public static final double kShooterRPM2 = 1350;
        public static final double kShooterRPM3 = 4000;
        public static final double kShooterRPM4 = 5000;
        public static final double kShooterRPM5 = 6000;
        public static final double kLimeightX1 = 0;
        public static final double kLimeightY1 = -3;
        public static final boolean kEnableLimeightX = true;
        public static final boolean kEnableLimeightY = true; // TODO:

    }

    public class HoodConstants {
        public static final int kMotorPort1 = 0;
        public static final int kMotorPort2 = 1;
        public static final double kDeadband = 0.07;
        public static final double kPercentageToRotations = 10;
        public static final int kPotentiometerPort = 4;
        public static final int kPotentiometerAvgWindow = 3;
        public static final double kMaxPowerLimit = 0.6;
        public static final double kPositionToleranceRotations = 0.05;
        public static final double kVelocityToleranceRPM = 5;
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;

    }

    public class ShooterConstants {
        public static final int kMotorPort1 = 1;
        public static final int kMotorPort2 = 2;
        public static final boolean kMotor1Inverted = true;
        public static final boolean kMotor2Inverted = !kMotor1Inverted;

        public static final double kCurrentLimitAmps = 80;
        public static final double kCurrentLimitDelay = 0.25;

        public static final double kEncoderTicksToRotations = 1.0 / 2048.0;
        public static final double kEncoderTicksToRPM = 600.0 / 2048.0;
        public static final double kMotorVelocityToleranceRPM = 5; // TODO: was 10, is 5 ok?
        public static final int kPIDSlot = 0;
        public static final double kP = 0.5;
        public static final double kI = 0.001;
        public static final double kD = 1;
        public static final double kF = (1023.0 + 100) / 20660.0;
        public static final double kIZone = 150;
    }

    public class FeederConstants {
        public static final int kMotorPort1 = 5;
        public static final int kMotorPort2 = 10;
        public static final int kInputSensorPort = 0;
        public static final int kOutputSensorPort = 1;
        public static final int kPistonForwardChannel = 0;
        public static final int kPistonReverseChannel = 1;
        public static final boolean kPistonInverted = true; // TODO:

        public static final boolean kmotor1Inverted = true;
        public static final boolean kmotor2Inverted = !kmotor1Inverted;

        public static final double kMotorIntakePower = 0.5;
        public static final double kMotorIntakePositionToleranceRotations = 0.2;
        public static final double kMotorIntakeVelocityToleranceRPM = 15;
        public static final double kP = 0.3;

        public static final double kEncoderTicksToRotations = 1.0 / 2048.0;
        public static final double kEncoderTicksToRPM = 600.0 / 2048.0;

        public static final double kRotationsPerBall = 5.75;
        public static final double kFeederLengthRotations = 25;

        public static final double kOuttakeDelaySec = 0.75;
    }

    public static final class Units {
        public static final double in2m = 1 / 39.37;
        public static final double m2in = 1 * 39.27;
        public static final double ft2in = 1 * 12.0;
        public static final double in2ft = 1 / 12.0;
        public static final double ft2m = 1 / 3.281;
        public static final double m2ft = 1 * 3.281;

        public static double normalizeRad(double rad) {
            rad = rad % (2 * Math.PI);
            if (rad < 0)
                rad += 2.0 * Math.PI;
            return rad;
        }

        public static boolean sameSign(double... n) {
            boolean sign = n[0] >= 0 ? true : false;

            for (int i = 1; i < n.length; i++)
                if ((sign && n[i] < 0) || (!sign && n[i] >= 0))
                    return false;
            return true;
        }

        public static boolean doubleEquals(double a, double b) {
            return Math.abs(a - b) < 1 / kInf;
        }

        public static double clamp(double val, double min, double max) {
            return Math.max(min, Math.min(val, max));
        }
    }

}
