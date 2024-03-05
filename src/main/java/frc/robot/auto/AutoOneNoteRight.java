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

public class AutoOneNoteRight{
    public AutoOneNoteRight(){
        // auto variables
        Command twoNoteRight;
        Swerve m_swerve = new Swerve();
        Intake i_intake = new Intake();
        Shooter s_shooter = new Shooter();
        WristSubsystem m_wristSubsystem = new WristSubsystem();
        ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
        int armMovementTimeout = 1;

        OneNoteCenter = new SequentialCommandGroup(
            //Auto Set Up
            new InstantCommand(() -> m_swerve.setHeading(new Rotation2d(0))),
            new InstantCommand(() -> m_swerve.resetModulesToAbsolute()),
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
                new ParallelRaceGroup(
                        new InstantCommand(() -> m_swerve.driveToPoint(2.56, 6.90, 0.00)),
                        new WaitCommand(3)
                    ),
            ),   

            new InstantCommand(() -> i_intake.setIntakeFalse()),

            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),

            new InstantCommand(() -> m_swerve.setHeading(new Rotation2d(180))),
            new InstantCommand(() -> m_swerve.resetModulesToAbsolute()),

            new InstantCommand(() -> s_shooter.setShootingSpeed()),
            new WaitCommand(.5),
            new InstantCommand(() -> s_shooter.setStopSpeed()),

            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
        
            
        );  
    }
}
