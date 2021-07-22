package frc.robot.swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class SwerveTrajectory {
    public Trajectory trajectory;
    public AngularTrajectory angularTrajectory;
    public Pose2d tolerance, velTolerance;
    public boolean enableFeedback, reqReset;

    public SwerveTrajectory(Trajectory trajectory, AngularTrajectory angularProfile, Pose2d tolerance,
            Pose2d velTolerance, boolean enableFeedback, boolean reqReset) {
        this.trajectory = trajectory;
        this.angularTrajectory = angularProfile;
        this.tolerance = tolerance;
        this.velTolerance = velTolerance;
        this.enableFeedback = enableFeedback;
        this.reqReset = reqReset;
    }

}