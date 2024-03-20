package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;


public class ConveyorBeltSubsystem {
    private CANSparkMax conveyorMotor;
    private double speed = 0;
    private DigitalInput converyorSensor;
    //private boolean notePresent = false; 

    public ConveyorBeltSubsystem() {
        conveyorMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        conveyorMotor.setInverted(true);
        converyorSensor = new DigitalInput(3);
    }

    public void periodic() {
        conveyorMotor.set(speed);//TODO Logic needs to be added for sensor once it is in place and everything is finished
    }

    public void startConveryor(){
        speed = .85;
    }

    public void feed(){
        speed = .85;
    }

    public void startConveryorReverse(){
        speed = -0.85;
    }

    public void stopConveryor(){
        speed = 0;
    }
}