package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;

public class Leave extends Command {
    private Swerve s_Swerve;

    public Leave() {
        new SequentialCommandGroup(
            new ParallelRaceGroup (
                new DriveToPoint(s_Swerve, 0, -1.1, 0),
                new WaitCommand(10)
            )
        );
    }
}