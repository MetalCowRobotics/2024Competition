package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AprilTagToDrive extends Command {
    private Swerve s_Swerve;
    private Vision m_Vision;
    private int aprilTagID;

    public AprilTagToDrive() {
        m_Vision.periodic();
        aprilTagID = m_Vision.getID();
        /* Speaker Middle Tags || Speaker Side Tags (RED / BLUE) */
        if (aprilTagID == 4 || aprilTagID == 3) {
            new DriveToPoint(s_Swerve, 15.00, 5.50, 90);
        }
        else if (aprilTagID == 7 || aprilTagID == 8) {
            new DriveToPoint(s_Swerve, 1.50, 5.50, 270);
        }
        /* AMP Tags */
        else if (aprilTagID == 5) {
            new DriveToPoint(s_Swerve, 14.75, 7.50, 0);
        }
        else if (aprilTagID == 6) {
            new DriveToPoint(s_Swerve, 1.85, 7.50, 0);
        }
        /* Trap Tags */
    }
}
