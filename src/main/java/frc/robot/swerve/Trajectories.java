package frc.robot.swerve;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Units;

public class Trajectories {
        public static SwerveTrajectory getExampleTrajectory() {

                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
                                DriveConstants.kMaxSpeedMetersPerSecond / 4, //
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared / 4)//
                                                .setKinematics(DriveConstants.kDriveKinematics);

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(//
                                // x is forward, y is sideways
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), //
                                List.of(new Translation2d(2, 0), //
                                                new Translation2d(2, -1), //
                                                new Translation2d(0, -1)), //
                                // new Pose2d(0.05, 0.4, new Rotation2d(0)), //
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), //
                                trajectoryConfig//
                );

                double t = trajectory.getTotalTimeSeconds();
                AngularTrajectory angularTrajectory = new AngularTrajectory(0, 2 * 2 * Math.PI, t, t / 4, t / 4,
                                t / 10 * 0);

                Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10));
                Pose2d velTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5));
                boolean enableFeedback = true, reqReset = true;

                return new SwerveTrajectory(trajectory, angularTrajectory, tolerance, velTolerance, enableFeedback,
                                reqReset);
        }

        public static SwerveTrajectory getAutoNavBarrelTrajectory() {

                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
                                DriveConstants.kMaxSpeedMetersPerSecond * 3 / 4.0, //
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared * 3 / 4.0)//
                                                .setKinematics(DriveConstants.kDriveKinematics);

                double r = 0.85; // the radius for turning around each pole

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(//
                                // x is forward, y is sideways
                                new Pose2d(60 * Units.in2m - DriveConstants.kVWidth, //
                                                -(60 * Units.in2m + DriveConstants.kHWidth - 1.3 * Units.ft2m), //
                                                Rotation2d.fromDegrees(0)), //
                                List.of(//
                                        // around the first pole
                                                new Translation2d(150 * Units.in2m + r + 0.5, -(60 * Units.in2m + r)), //
                                                new Translation2d(150 * Units.in2m + r + 0.5,
                                                                -(60 * Units.in2m - r - 0.5)), //
                                                new Translation2d(150 * Units.in2m - r, -(60 * Units.in2m - r)), //
                                                new Translation2d(150 * Units.in2m - r, -(60 * Units.in2m + r)), //
                                                // around the second pole
                                                new Translation2d(240 * Units.in2m + r + 1 * Units.ft2m,
                                                                -(120 * Units.in2m - r)), //
                                                new Translation2d(240 * Units.in2m + r + 1 * Units.ft2m,
                                                                -(120 * Units.in2m + r)), //
                                                new Translation2d(240 * Units.in2m - r, -(120 * Units.in2m + r)), //
                                                new Translation2d(240 * Units.in2m - r, -(120 * Units.in2m - r)), //
                                                // around the thrid pole
                                                new Translation2d(300 * Units.in2m - r, -(60 * Units.in2m - r)), //
                                                new Translation2d(300 * Units.in2m + r + 1, -(60 * Units.in2m - r)), //
                                                new Translation2d(300 * Units.in2m + r + 1,
                                                                -(60 * Units.in2m + 1.7 * r)), //
                                                new Translation2d(240 * Units.in2m + r, -(120 * Units.in2m - 2 * r)), //
                                                // coming back
                                                new Translation2d(150 * Units.in2m + r, -(60 * Units.in2m + 1.5 * r)) //
                                ), //
                                   // to the finish line
                                new Pose2d(30 * Units.in2m, -(90 * Units.in2m), //
                                                Rotation2d.fromDegrees(180)), //
                                trajectoryConfig//
                );
                double t = trajectory.getTotalTimeSeconds();
                AngularTrajectory angularTrajectory = new AngularTrajectory(0, 10 * Math.PI, t, t / 4, t / 4,
                                t / 10 * 0);

                Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10));
                Pose2d velTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5));
                boolean enableFeedback = true, reqReset = true;

                return new SwerveTrajectory(trajectory, angularTrajectory, tolerance, velTolerance, enableFeedback,
                                reqReset);
        }

        public static SwerveTrajectory getAutoNavBarrelTrajectoryFast() {

                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
                                DriveConstants.kMaxSpeedMetersPerSecond * 0.9, //
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared * 0.9)//
                                                .setKinematics(DriveConstants.kDriveKinematics);

                double r = 0.8; // the radius for turning around each pole

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(//
                                // x is forward, y is sideways
                                new Pose2d(60 * Units.in2m - DriveConstants.kVWidth + 0.8 * Units.ft2m, //
                                                -(60 * Units.in2m + DriveConstants.kHWidth - 1.3 * 0 * Units.ft2m), //
                                                Rotation2d.fromDegrees(0)), //
                                List.of(//
                                        // around the first pole
                                                new Translation2d(150 * Units.in2m + r * 0.5,
                                                                -(60 * Units.in2m + r * 0.5)), //
                                                new Translation2d(150 * Units.in2m + r, -(60 * Units.in2m - r - 0.5)), //
                                                new Translation2d(150 * Units.in2m - 1.2 * r,
                                                                -(60 * Units.in2m - 1.2 * r)), //
                                                new Translation2d(150 * Units.in2m - r, -(60 * Units.in2m + r)), //
                                                // around the second pole
                                                new Translation2d(240 * Units.in2m + 0.8 * r,
                                                                -(120 * Units.in2m - 0.8 * r)), //
                                                new Translation2d(240 * Units.in2m + r + 0.3, -(120 * Units.in2m + r)), //
                                                new Translation2d(240 * Units.in2m - 2 * r, -(120 * Units.in2m + r)), //
                                                new Translation2d(240 * Units.in2m - 1.5 * r, -(120 * Units.in2m - r)), //
                                                // around the thrid pole
                                                new Translation2d(300 * Units.in2m - r, -(60 * Units.in2m - r)), //
                                                new Translation2d(300 * Units.in2m + r + 0.1, -(60 * Units.in2m - r)), //
                                                new Translation2d(300 * Units.in2m + r + 0.1, -(60 * Units.in2m + r)), //
                                                new Translation2d(240 * Units.in2m + r, -(120 * Units.in2m - r)), //
                                                new Translation2d(150 * Units.in2m + r, -(60 * Units.in2m + 1.5 * r)) //
                                ), //
                                   // to the finish line
                                new Pose2d(30 * Units.in2m - 2.0, -(90 * Units.in2m + 0.9), //
                                                Rotation2d.fromDegrees(180)), //
                                trajectoryConfig//
                );
                double t = trajectory.getTotalTimeSeconds();
                AngularTrajectory angularTrajectory = new AngularTrajectory(0, 0 * 10 * Math.PI, t, t / 4, t / 4,
                                t / 10 * 0);

                Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10));
                Pose2d velTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5));
                boolean enableFeedback = true, reqReset = true;

                return new SwerveTrajectory(trajectory, angularTrajectory, tolerance, velTolerance, enableFeedback,
                                reqReset);
        }

        public static SwerveTrajectory getAutoNavSlalomTrajectory() {

                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
                                DriveConstants.kMaxSpeedMetersPerSecond * 0.9, //
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared * 0.9)//
                                                .setKinematics(DriveConstants.kDriveKinematics);

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(//
                                // x is forward, y is sideways
                                new Pose2d(30 * Units.in2m, -(30 * Units.in2m), //
                                                Rotation2d.fromDegrees(0)), //
                                List.of(//
                                        // around the first pole
                                                new Translation2d(65 * Units.in2m, -(30 * Units.in2m)), //
                                                new Translation2d(80 * Units.in2m, -(90 * Units.in2m)), //
                                                new Translation2d(120 * Units.in2m, -(120 * Units.in2m)), //
                                                new Translation2d(240 * Units.in2m, -(120 * Units.in2m)), //
                                                // around the second pole
                                                new Translation2d(278 * Units.in2m, -(90 * Units.in2m)), //
                                                new Translation2d(280 * Units.in2m, -(30 * Units.in2m)), //
                                                new Translation2d(355 * Units.in2m, -(30 * Units.in2m)), //
                                                new Translation2d(350 * Units.in2m, -(90 * Units.in2m)), //
                                                new Translation2d(293 * Units.in2m, -(90 * Units.in2m)), //
                                                new Translation2d(287 * Units.in2m, -(30 * Units.in2m)), //
                                                // around the thrid pole
                                                new Translation2d(200 * Units.in2m, -(20 * Units.in2m)), //
                                                new Translation2d(120 * Units.in2m, -(30 * Units.in2m)), //
                                                new Translation2d(78 * Units.in2m, -(30 * Units.in2m)), //
                                                new Translation2d(65 * Units.in2m, -(90 * Units.in2m)), //
                                                new Translation2d(30 * Units.in2m, -(80 * Units.in2m)) //
                                ), //
                                   // to the finish line
                                new Pose2d(0 * Units.in2m, -(90 * Units.in2m), //
                                                Rotation2d.fromDegrees(180)), //
                                trajectoryConfig//
                );
                double t = trajectory.getTotalTimeSeconds();
                AngularTrajectory angularTrajectory = new AngularTrajectory(0, 0 * 10 * Math.PI, t, t / 4, t / 4,
                                t / 10 * 0);

                Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10));
                Pose2d velTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5));
                boolean enableFeedback = true, reqReset = true;

                return new SwerveTrajectory(trajectory, angularTrajectory, tolerance, velTolerance, enableFeedback,
                                reqReset);
        }

        public static void main(String[] args) {

                SwerveTrajectory swerveTrajectory = getAutoNavSlalomTrajectory();// getExampleTrajectory();
                Trajectory trajectory = swerveTrajectory.trajectory;
                double t = trajectory.getTotalTimeSeconds();
                AngularTrajectory angularTrajectory = swerveTrajectory.angularTrajectory;

                for (double i = 0; i <= t; i += 0.05) {
                        String f = "%.4f | %.4f %n";
                        // System.out.format(f, angularTrajectory.getPos(i),
                        // angularTrajectory.getVel(i));
                }
                double r = 1; // the radius for turning around each pole

                var utilPoints = List.of(//
                                new Translation2d(150 * Units.in2m, -60 * Units.in2m), //
                                new Translation2d(240 * Units.in2m, -120 * Units.in2m), //
                                new Translation2d(300 * Units.in2m, -60 * Units.in2m) //
                // // around the first pole
                // new Translation2d(150 * Units.in2m + r, -(60 * Units.in2m + r)), //
                // new Translation2d(150 * Units.in2m + r, -(60 * Units.in2m - r)), //
                // new Translation2d(150 * Units.in2m - r, -(60 * Units.in2m - r)), //
                // new Translation2d(150 * Units.in2m - r, -(60 * Units.in2m + r)), //
                // // around the second pole
                // new Translation2d(240 * Units.in2m + r, -(120 * Units.in2m - r)), //
                // new Translation2d(240 * Units.in2m + r, -(120 * Units.in2m + r)), //
                // new Translation2d(240 * Units.in2m - r, -(120 * Units.in2m + r)), //
                // new Translation2d(240 * Units.in2m - r, -(120 * Units.in2m - r)), //
                // // around the thrid pole
                // new Translation2d(300 * Units.in2m - r, -(60 * Units.in2m - r)), //
                // new Translation2d(300 * Units.in2m + r, -(60 * Units.in2m - r)), //
                // new Translation2d(300 * Units.in2m + r, -(60 * Units.in2m + r)), //
                // new Translation2d(300 * Units.in2m - r, -(60 * Units.in2m + r)) //
                );

                System.out.println("================================================");

                // for (double i = 0; i <= t; i += 0.05) {
                // System.out.println(trajectory.sample(i).poseMeters.getX());
                // }
                // for (var i : utilPoints)
                // System.out.println(i.getX());

                // System.out.println("================================================");
                // for (double i = 0; i <= t; i += 0.05) {
                // System.out.println(-trajectory.sample(i).poseMeters.getY());
                // }
                // for (var i : utilPoints)
                // System.out.println(-i.getY());

                for (double i = 0; i <= t; i += 0.05) {
                        System.out.println(",," + trajectory.sample(i).poseMeters.getX() + ","
                                        + trajectory.sample(i).poseMeters.getY());
                }
                // for (var i : utilPoints)
                // System.out.println(",," + i.getX() + "," + i.getY());

                System.out.println("TOTAL: " + t + " seconds");
        }
}
