package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShooterIdle;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private Shooter shooter = Shooter.getInstance();
    private Hood hood = Hood.getInstance();
    private Feeder feeder = Feeder.getInstance();

    private Joystick joy1 = new Joystick(0);
    // private Joystick joy2 = new Joystick(1);

    public RobotContainer() {
        shooter.setDefaultCommand(new ShooterIdle(joy1));
        hood.setDefaultCommand(new HoodSetPosition(DriverConstants.kHoodAngle1));
        feeder.setDefaultCommand(new IntakeBall());

        new JoystickButton(joy1, 2).whenHeld(new ShootCmd( //
                DriverConstants.kHoodAngle2, //
                DriverConstants.kShooterRPM1, //
                () -> joy1.getRawButton(1) //
        ));

                new JoystickButton(joy1, 3).whenHeld(new ShootCmd( //
                DriverConstants.kHoodAngle2, //
                DriverConstants.kShooterRPM2, //
                () -> joy1.getRawButton(1) //
        ));

                   new JoystickButton(joy1, 4).whenHeld(new ShootCmd( //
                DriverConstants.kHoodAngle2, //
                DriverConstants.kShooterRPM3, //
                () -> joy1.getRawButton(1) //
        ));

                   new JoystickButton(joy1, 5).whenHeld(new ShootCmd( //
                DriverConstants.kHoodAngle2, //
                DriverConstants.kShooterRPM4, //
                () -> joy1.getRawButton(1) //
        ));

                   new JoystickButton(joy1, 6).whenHeld(new ShootCmd( //
                DriverConstants.kHoodAngle2, //
                DriverConstants.kShooterRPM5, //
                () -> joy1.getRawButton(1) //
        ));

        // new JoystickButton(joy2, 3).whenHeld(new HoodSetPosition(DriverConstants.kHoodAngle1));
        // new JoystickButton(joy2, 4).whenHeld(new HoodSetPosition(DriverConstants.kHoodAngle2));

        // new JoystickButton(joy1, 3).whenHeld(new ShooterSetVelocity(1000));
        // new JoystickButton(joy1, 4).whenHeld(new ShooterSetVelocity(3000));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
