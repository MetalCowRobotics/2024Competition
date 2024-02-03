package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


public class Intake {
    private CANSparkMax motor1;   
    private CANSparkMax motor2;

    public Intake(){
        motor1 = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void start(double speed){
        motor1.set(speed);
        motor2.set(-speed);
    }
    public void stop(){
        motor1.set(0);
        motor2.set(0);
    }
}














