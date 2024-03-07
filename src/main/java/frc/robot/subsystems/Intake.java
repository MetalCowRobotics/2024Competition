package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
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
        // set up the intake motor and the note detector
    }

    public void periodic() {
        if (intakeStatus) {
            intakeMotor.set(speed);
            // set speed of intake to 0.5
        }
        else {
             intakeMotor.set(0);
            }
            // set speed of intake to 0
        if (noteDetector.get()){
            intakeStatus = false;
            // if the note detector is pressed, then set the intake to false
        }    

    }

    public void setspeed(double i){
            speed = i; 
            // set speed to i
    }

    public void setIntakeTrue() {
        intakeStatus = true;
        // set intake to true
    }

    public void setIntakeFalse() {
        intakeStatus = false;
        // set intake to false
    }
}

//The only good code!!