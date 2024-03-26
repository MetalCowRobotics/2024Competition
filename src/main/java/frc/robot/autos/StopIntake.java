package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends InstantCommandBase{
    IntakeSubsystem i_intake;
    public StopIntake(IntakeSubsystem m_intake){
        i_intake = m_intake;
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));
        i_intake.stopintake();
        // i_intake.setIntakeFalse();
        SmartDashboard.putString("auto", "stopped");
    }
}
