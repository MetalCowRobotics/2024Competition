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
    }

    public void periodic() {


        /*
        if intakeStatus
            speed = .9

        else
            speed = 0
        if noteDetector
             //   intake status = false
        */

        if (intakeStatus) {
            speed = .9;
        }
        else {
             speed = 0;
            }

        if (noteDetector.get()){
            intakeStatus = false;
        }    



         
       // if (!noteDetector.get()) {
         //       speed = 0.90;
         //   }
          //   }
          //  else {
           //     speed = 0;
          //       intakeStatus = false;
          //  }

        intakeMotor.set(speed);
    }

    public void setIntakeTrue() {
        intakeStatus = true;
    }

    public void setIntakeFalse() {
        intakeStatus = false;
    }
}

//The only good code!!