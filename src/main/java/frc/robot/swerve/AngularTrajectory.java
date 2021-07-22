package frc.robot.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.*;
import frc.robot.Constants;

public class AngularTrajectory {

    private final TrapezoidProfile profile1, profile2, profile3;
    private final double profile2StartTime, profile3StartTime;

    /**
     * Constructs a trapezoidal profile for the theta controller when path-following
     * with swerve drivetrain. To use, construct this class on init and repeatedly
     * call the get() function with the current time since started path following to
     * get a dynamic theta setpoint. Note: this profile is supposed to assist the
     * main swerve path-follower. so this profile's time is in sync with it.
     * 
     * @param startRotationRad   robot starting rotation in radians, typically 0.
     * @param rotationRad        number of radians to rotate during path-following
     * @param trajectoryDuration the total time in seconds of the main
     *                           path-follower, you can find out by calling
     *                           trajectory.getTotalTimeSeconds()
     * @param accTime            the time for acceleration and deceleration of
     *                           rotation
     * @param idleTimeAtEnd      the amount of time to stop spinning at the end of
     *                           the trajectory
     */
    public AngularTrajectory(double startRotationRad, double rotationRad, double trajectoryDuration, double accTime,
            double deAccTime, double idleTimeAtEnd) {
        double angularProfileDuration = trajectoryDuration - idleTimeAtEnd;
        MaxVelMaxAcc config = calcMaxVelMaxAcc(rotationRad, accTime, deAccTime, angularProfileDuration);
        Constraints constraints1 = new Constraints(Math.abs(config.maxVel), Math.abs(config.maxAcc));
        Constraints constraints2 = new Constraints(Math.abs(config.maxVel), 1 / Constants.kInf);
        Constraints constraints3 = new Constraints(Math.abs(config.maxVel), Math.abs(config.maxDeAcc));

        State initialState = new State(startRotationRad, 0);
        State state1 = new State(config.pos1, config.maxVel);
        State state2 = new State(config.pos2, config.maxVel);
        State finalState = new State(startRotationRad + rotationRad, 0);

        // System.out.println(constraints1.maxVelocity + " " +
        // constraints1.maxAcceleration);
        // System.out.println(constraints2.maxVelocity + " " +
        // constraints2.maxAcceleration);
        // System.out.println(constraints3.maxVelocity + " " +
        // constraints3.maxAcceleration);
        // System.out.println(state2.position + " " + state2.velocity);
        // System.out.println(finalState.position + " " + finalState.velocity);
        // System.out.println(
        // initialState.velocity + "\n" + state1.velocity + "\n" + state2.velocity +
        // "\n" + finalState.velocity);

        profile1 = new TrapezoidProfile(constraints1, state1, initialState);
        profile2 = new TrapezoidProfile(constraints2, state2, state1);
        profile3 = new TrapezoidProfile(constraints3, finalState, state2);

        profile2StartTime = accTime;
        profile3StartTime = angularProfileDuration - deAccTime;
    }

    private MaxVelMaxAcc calcMaxVelMaxAcc(double rad, double accTime, double deAccTime, double totalTime) {
        double cruiseTime = totalTime - accTime - deAccTime;
        if (cruiseTime < -0.0001)
            throw new RuntimeException(
                    "ERROR: Acceleration times are too long and does not fit into the trajectory's duration");

        double v = 2 * rad / (accTime + deAccTime + 2 * cruiseTime);
        double a1 = v / accTime;
        double a2 = v / deAccTime;
        double p1 = accTime * v / 2.0;
        double p2 = rad - deAccTime * v / 2.0;
        return new MaxVelMaxAcc(v, a1, a2, p1, p2);
    }

    private State getCurrentState(double time) {
        if (time < profile2StartTime) {
            // System.out.print(" p1 ");
            return profile1.calculate(time);
        } else if (time < profile3StartTime) {
            // System.out.print(" p2 ");
            return profile2.calculate(time - profile2StartTime);
        } else {
            // System.out.print(" p3 ");
            return profile3.calculate(time - profile3StartTime);
        }
    }

    public Rotation2d get(double time) {
        return new Rotation2d(getCurrentState(time).position);

    }

    public double getPos(double time) {
        return getCurrentState(time).position;
    }

    public double getVel(double time) {
        return getCurrentState(time).velocity;
    }

    class MaxVelMaxAcc {
        final double maxVel, maxAcc, maxDeAcc;
        final double pos1, pos2;

        public MaxVelMaxAcc(double maxVel, double maxAcc, double maxDeAcc, double pos1, double pos2) {
            this.maxVel = maxVel;
            this.maxAcc = maxAcc;
            this.maxDeAcc = maxDeAcc;
            this.pos1 = pos1;
            this.pos2 = pos2;
        }

        @Override
        public String toString() {
            return "MaxVelMaxAcc [maxAcc=" + maxAcc + ", maxDeAcc=" + maxDeAcc + ", maxVel=" + maxVel + ", pos1=" + pos1
                    + ", pos2=" + pos2 + "]";
        }

    }

    public static void main(String[] args) {
        double t = 10;
        var profile = new AngularTrajectory(0, -2 * Math.PI, t, 0.3, 3, 0);
        for (double i = 0; i <= t + 0.001; i += 0.05) {
            String f = "%.2f: %.4f | %.4f %n";
            System.out.format(f, i, profile.getPos(i), profile.getVel(i));
        }
    }

}
