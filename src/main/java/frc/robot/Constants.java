package frc.robot;

public final class Constants {
    public static final int kCANTimeoutMS = 10;
    public static final boolean debug = true;

    public class DriverConstants {
        public static final double kHoodAngle1 = 1.4;
        public static final double kHoodAngle2 = 2.3;
        public static final double kShooterRPM1 = 800;
    }

    public class HoodConstants {
        public static final int kMotorPort1 = 0;
        public static final int kMotorPort2 = 1;
        public static final double kDeadband = 0.07;
        public static final double kPercentageToRotations = 10;
        public static final int kPotentiometerPort = 0;
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
        public static final double kMotorVelocityToleranceRPM = 30;
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

        public static final double kOuttakeDelaySec = 0.5;
    }

}
