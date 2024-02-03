package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;


public class Intake {
   private CANSparkMax motor1;   
    private CANSparkMax motor2;
    private DigitalInput Intake;


    public Intake(){
        motor1 = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        Intake = new DigitalInput(0);
        
    }

    public void rununtilnote(){
        if (Intake.get()) {
            stop();
        }
        else {
            start(.5);
        }
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














