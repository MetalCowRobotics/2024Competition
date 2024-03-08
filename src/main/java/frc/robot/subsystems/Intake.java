package frc.robot.subsystems;

import com.revrobotics.CANSparkLowlevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    private CANSparkMax intakeMotor;   
    private double speed = 0;
    private DigitalInput noteDetector;
    private boolean intakeStatus = false;

    public Intake() {
        intakeMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        noteDetector = new DigitalInput(0);
    }

    public void periodic() {
        if (intakeStatus) {
            intakeMotor.set(speed);
        }
        else {
             intakeMotor.set(0);
            }
    }
    public void setspeed(double i){
            speed = i; 
    }

    public void setIntakeTrue() {
        intakeStatus = true;
    }

    public void setIntakeFalse() {
        intakeStatus = false;
    }
}

//The only good code!!