package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED {
    private static Spark ledController = new Spark(1);

    public static void runDefault(){
        ledController.set(SmartDashboard.getNumber("LED Value", 0.73));
    }
    public static void runOrange(){
        ledController.set(.65);
    }
    public static void runGradient(){
        ledController.set(.41);
    }
}