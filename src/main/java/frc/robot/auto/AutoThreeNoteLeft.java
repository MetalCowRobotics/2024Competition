package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmToAngles;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoTwoNoteRight {
    public AutoThreeNoteLeft(){
        // auto variables
        Command ThreeNoteLeft;
        Swerve m_swerve = new Swerve();
        Intake i_intake = new Intake();
        Shooter s_shooter = new Shooter();
        WristSubsystem m_wristSubsystem = new WristSubsystem();
        ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
        int armMovementTimeout = 1;

        ThreeNoteLeft = new SequentialCommandGroup(
            //Auto Set Up
            new InstantCommand(() -> m_swerve.setHeading(new Rotation2d(180))),
            new InstantCommand(() -> m_swerve.resetModulesToAbsolute()),

            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),

            new InstantCommand(() -> s_shooter.setShootingSpeed()),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_shooter.setStopSpeed()),

            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),

            new InstantCommand(() -> i_intake.setIntakeTrue()),
            new ParallelRaceGroup(
                new ParallelRaceGroup(
                        new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                        new WaitCommand(armMovementTimeout)
                    ),
                new ParallelRaceGroup (
                        new InstantCommand(() -> m_swerve.driveToPoint(2.89, 7.00, -26.40)),
                        new WaitCommand(3)
                    ),
            ),

            new InstantCommand(() -> i_intake.setIntakeFalse()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_swerve.setHeading(new Roation2d(40))),
            new InstantCommand(() -> i_intake.setShootingSpeed()),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_shooter.setStopSpeed()),
            new WaitCommand(0.5),
            new InstantCommand(() -> i_intake.setIntakeTrue()),

            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                        new WaitCommand(armMovementTimeout)
                    ),
                ),
                new SequentialCommandGroup(
                    new ParallelRaceGroup (
                        new InstantCommand(() -> m_swerve.driveToPoint(2.52, 5.54, 0)),
                        new WaitCommand(3)
                    ),
                )

            ),
            new InstantCommand(() -> i_intake.setIntakeFalse()),
            new InstantCommand(() -> m_swerve.driveToPoint(2.52,5.52,0)),
           
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),

            new InstantCommand(() -> m_swerve.setHeading(new Rotation2d(40))),
            new InstantCommand(() -> i_intake.setShootingSpeed()),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_shooter.setStopSpeed())



        );
    }
}
