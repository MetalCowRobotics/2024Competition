// package frc.robot.autos;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.lib14.MCRCommand;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.IntakeSubsystem;

// public class DriveToPointA implements MCRCommand{
//     Swerve s_swerve;
//     double xCor;
//     double yCor;
//     double targetAngle;
//     boolean finished_flag = false;
//     boolean first_time = true;
//     IntakeSubsystem i_Intake;

//     public DriveToPointA(Swerve m_swerve, IntakeSubsystem m_Intake, double x, double y, double theta){
//         s_swerve = m_swerve;
//         xCor = x;
//         yCor = y;
//         targetAngle = theta;
//         i_Intake = m_Intake;
//         }

//     @Override
//     public void run(){
//         // i_Intake.startIntake();
//         s_swerve.driveToPoint(xCor, yCor, targetAngle);
//     }
    
//     @Override
//     public boolean isFinished(){
//         if(!first_time){
//             SmartDashboard.putBoolean("stopDriving", i_Intake.getStopDriving());
//             if (!finished_flag || i_Intake.getStopDriving()) //should be changed to "&&"
//                 finished_flag = true;
//             return finished_flag;
//         }
//         else{
//         i_Intake.setStartDriving();
//         finished_flag = false;
//         }
//         first_time = false;
//         SmartDashboard.putBoolean("finished flag", finished_flag);
//         SmartDashboard.putBoolean("first Time", first_time);
        
//         return finished_flag;
//     }
// }
