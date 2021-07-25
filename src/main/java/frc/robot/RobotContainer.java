package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShooterIdle;
import frc.robot.commands.SwerveFollowTrajectory;
import frc.robot.commands.SwerveTeleDrive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.swerve.AngularTrajectory;
import frc.robot.swerve.SwerveTrajectory;
import frc.robot.swerve.Trajectories;

public class RobotContainer {
        private Command autoCommand;

        private final SwerveDrivetrain swerveDrivetrain = SwerveDrivetrain.getInstance();
        private final Limelight limelight = Limelight.getInstance();

        private Shooter shooter = Shooter.getInstance();
        private Hood hood = Hood.getInstance();
        private Feeder feeder = Feeder.getInstance();

        private Joystick driverJoy = new Joystick(OIConstants.kDriverControllerPort);
        private Joystick operatorJoy = new Joystick(OIConstants.kOperatorControllerPort);

        public RobotContainer() {
                initAutonomousCommand();
                shooter.setDefaultCommand(new ShooterIdle(operatorJoy));
                hood.setDefaultCommand(new HoodSetPosition(DriverConstants.kHoodAngle1));
                feeder.setDefaultCommand(new IntakeBall());
                swerveDrivetrain.setDefaultCommand(new SwerveTeleDrive(//
                                () -> driverJoy.getRawAxis(OIConstants.kDriverYAxis) //
                                                * (OIConstants.kDriverYAxisInverted ? -1.0 : 1.0), //
                                () -> driverJoy.getRawAxis(OIConstants.kDriverXAxis) //
                                                * (OIConstants.kDriverXAxisInverted ? -1.0 : 1.0), //
                                () -> driverJoy.getRawAxis(OIConstants.kDriverRotAxis)
                                                * (OIConstants.kDriverRotAxisInverted ? -1.0 : 1.0), //
                                true)//
                );

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.putData(new InstantCommand(() -> swerveDrivetrain.zeroHeading()));

                new JoystickButton(driverJoy, 1).whileActiveOnce(new SwerveTeleDrive(//
                                () -> limelight.isValid() ? -limelight.getY() / 20.0 / 1 : 0.0, //
                                () -> 0.0, //
                                () -> limelight.isValid() ? limelight.getX() / 27.0 / 4 : 0.0, //
                                false)//
                );

                new JoystickButton(operatorJoy, 2).whenHeld(new ShootCmd( //
                                DriverConstants.kHoodAngle2, //
                                DriverConstants.kShooterRPM1, //
                                DriverConstants.kLimeightX1, //
                                DriverConstants.kLimeightY1, //
                                DriverConstants.kEnableLimeightX, //
                                DriverConstants.kEnableLimeightY, //
                                () -> operatorJoy.getRawButton(1) //
                ));

                // new JoystickButton(joy2, 3).whenHeld(new
                // HoodSetPosition(DriverConstants.kHoodAngle1));
                // new JoystickButton(joy2, 4).whenHeld(new
                // HoodSetPosition(DriverConstants.kHoodAngle2));

                // new JoystickButton(joy1, 3).whenHeld(new ShooterSetVelocity(1000));
                // new JoystickButton(joy1, 4).whenHeld(new ShooterSetVelocity(3000));
        }

        private void initAutonomousCommand() {
                SwerveTrajectory swerveTrajectory = Trajectories.getAutoNavSlalomTrajectory();
                // SwerveTrajectory swerveTrajectory = Trajectories.getExampleTrajectory();

                Trajectory trajectory = swerveTrajectory.trajectory;
                AngularTrajectory angularTrajectory = swerveTrajectory.angularTrajectory;
                Pose2d tolerance = swerveTrajectory.tolerance;
                Pose2d velTolerance = swerveTrajectory.velTolerance;
                boolean enableFeedback = swerveTrajectory.enableFeedback;
                boolean reqReset = swerveTrajectory.reqReset;

                var trajFollowCmd1 = new SwerveFollowTrajectory(trajectory, //
                                angularTrajectory, //
                                tolerance, //
                                velTolerance, //
                                enableFeedback, //
                                reqReset);

                autoCommand = trajFollowCmd1.andThen(new SwerveTeleDrive(0, 0, 0, true));
                // return new WaitCommand(20)
                // .andThen(trajFollowCmd1.andThen(new SwerveTeleDrive(swerveDrivetrain, 0, 0,
                // 0, true)));
        }

        public Command getAutonomousCommand() {
                return null;
                // if (autoCommand == null)
                //         initAutonomousCommand();
                // return autoCommand;
        }
}
