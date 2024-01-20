package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


public class Intake {
    private CANSparkMax mot1;   
    private CANSparkMax mot2;

    

    public Intake(int id1, int id2){
        mot1 = new CANSparkMax(id1, CANSparkLowLevel.MotorType.kBrushless);
        mot2 = new CANSparkMax(id2, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void start(int speed){
        mot1.set(-speed);
        mot2.set(speed);
    }
    public void stop(){
        mot1.set(0);
        mot2.set(0);
    }







}
