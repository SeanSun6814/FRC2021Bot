package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.swerve.AngularTrajectory;
import frc.robot.swerve.SwerveController;
import frc.robot.util.Logger;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an array. The desired wheel and module
 * rotation velocities should be taken from those and used in velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes to the angle given in the final state of the trajectory.
 */
@SuppressWarnings("MemberName")
public class SwerveFollowTrajectory extends CommandBase {
    private final SwerveDrivetrain swerveDrivetrain = SwerveDrivetrain.getInstance();
    private final Timer m_timer = new Timer();
    private final Logger logger;
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final SwerveDriveKinematics kinematics;
    private final SwerveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final AngularTrajectory angularProfile;
    private final Pose2d tolerance;
    private final Pose2d velTolerance;
    private final boolean reqReset;
    private boolean onTarget;

    /**
     * Constructs a new SwerveFollowTrajectory that when executed will follow the
     * provided trajectory. This command will not return output voltages but rather
     * raw module states from the position controllers which need to be put into a
     * velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path- this is left to the user, since it is not appropriate for paths
     * with nonstationary endstates.
     * 
     * @param swerveDrivetrain the swerve drivetrain subsystem instance
     * @param trajectory       the trajectory to follow
     * @param angularProfile   the trajectory for rotation to follow (enter null to
     *                         auto determine using trajectory poses)
     * @param tolerance        the amount of error allowed before this command exits
     *                         (enter null for no tolerance - command will finish as
     *                         soon as the trajectory's duration ends)
     */
    public SwerveFollowTrajectory(Trajectory trajectory,
            AngularTrajectory angularProfile, Pose2d tolerance, Pose2d velTolerance, boolean enableFeedback,
            boolean reqReset) {

        var thetaPIDController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);
        var xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
        var yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);

        if (angularProfile == null)
            angularProfile = new AngularTrajectory(trajectory.getInitialPose().getRotation().getRadians(),
                    trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation().getRadians(),
                    trajectory.getTotalTimeSeconds(), trajectory.getTotalTimeSeconds() / 3,
                    trajectory.getTotalTimeSeconds() / 3, trajectory.getTotalTimeSeconds() / 3);

        if (tolerance == null)
            tolerance = new Pose2d(Constants.kInf, Constants.kInf, new Rotation2d(Constants.kInf));
        if (velTolerance == null)
            velTolerance = new Pose2d(Constants.kInf, Constants.kInf, new Rotation2d(Constants.kInf));

        this.trajectory = trajectory;
        this.pose = swerveDrivetrain::getPose;
        this.kinematics = DriveConstants.kDriveKinematics;
        this.controller = new SwerveController(xPIDController, yPIDController, thetaPIDController);
        this.outputModuleStates = swerveDrivetrain::setModuleStates;
        this.angularProfile = angularProfile;
        this.tolerance = tolerance;
        this.velTolerance = velTolerance;
        this.reqReset = reqReset;
        this.controller.setEnabled(enableFeedback);
        this.logger = new Logger(getName());

        addRequirements(swerveDrivetrain);
    }

    @Override
    public void initialize() {
        onTarget = false;
        if (reqReset)
            swerveDrivetrain.resetOdometry(trajectory.getInitialPose());
        // System.out.println("INITIAL POSE" + trajectory.getInitialPose());
        m_timer.reset();
        m_timer.start();
        System.out.println(getName() + " command started.");
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = trajectory.sample(curTime);
        SmartDashboard.putString("Traj FF state", desiredState.toString());
        // var p = new Pose2d(pose.get().getY(), pose.get().getX(),
        // pose.get().getRotation());

        var targetChassisSpeeds = controller.calculate(pose.get(), desiredState, //
                new Rotation2d(angularProfile.getVel(curTime)), new Rotation2d(angularProfile.getPos(curTime)));
        SmartDashboard.putString("TargetChassisState", targetChassisSpeeds.toString());
        var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

        outputModuleStates.accept(targetModuleStates);

        if (Math.abs(targetChassisSpeeds.omegaRadiansPerSecond) < velTolerance.getRotation().getRadians() && //
                Math.abs(targetChassisSpeeds.vxMetersPerSecond) < velTolerance.getX() && //
                Math.abs(targetChassisSpeeds.vyMetersPerSecond) < velTolerance.getY()) {
            Pose2d target = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;

            if (Math.abs(target.getRotation().getRadians() - pose.get().getRotation().getRadians()) //
                    < tolerance.getRotation().getRadians() && //
                    Math.abs(target.getX() - pose.get().getX()) < tolerance.getX() && //
                    Math.abs(target.getY() - pose.get().getY()) < tolerance.getY()) {
                onTarget = true;
            }
        }

        // System.out.println(targetChassisSpeeds.omegaRadiansPerSecond + " " +
        // targetChassisSpeeds.vxMetersPerSecond + " "
        // + targetChassisSpeeds.vyMetersPerSecond);

        logger.log(pose.get().getX() + "," + pose.get().getY());
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        System.out.println(getName() + " command ended, interrupted=[" + interrupted + "].");
    }

    @Override
    public boolean isFinished() {
        // System.out.println("onTarget" + onTarget);
        if (m_timer.hasElapsed(trajectory.getTotalTimeSeconds()))
            if (onTarget)
                return true;
            else
                System.out.println("PATH DONE BUT TOLERANCE NOT MET, CORRECTING...");
        return false;
    }
}
