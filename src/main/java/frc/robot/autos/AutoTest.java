package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoTest extends Command {
    private Swerve s_Swerve;
    private Shooter m_Shooter;
    private Intake m_Intake;

    public AutoTest() {
        new SequentialCommandGroup(
            // new ParallelRaceGroup (
            //     new DriveToPoint(s_Swerve, 0, -1.1, 0),
            //     new WaitCommand(2)
            // ),
            new InstantCommand(() -> m_Shooter.runShooter()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_Shooter.stopShooter()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_Intake.runIntake()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_Intake.stopIntake()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_Intake.runIntakeReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_Intake .stopIntake())
        );
    }
}