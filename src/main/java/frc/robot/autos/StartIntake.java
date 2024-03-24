package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Intake;

public class StartIntake extends InstantCommandBase{
    Intake i_intake;
    public StartIntake(Intake m_intake){
        i_intake = m_intake;
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));

        i_intake.startIntake();
        // i_intake.setIntakeTrue();
        SmartDashboard.putString("auto", "running");
    }
}
