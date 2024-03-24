package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED {
    private static Spark ledController = new Spark(0);

    public static void runDefault(){
        ledController.set(.75);
    }
    public static void runOrange(){
        ledController.set(.63);
    }
    public static void runGradient(){
        ledController.set(.41);
    }
}