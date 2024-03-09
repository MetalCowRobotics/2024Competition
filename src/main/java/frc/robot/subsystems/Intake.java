package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private CANSparkMax intakeMotor;   
    private double speed = 0;
    private boolean intakeStatus = false;
    private RelativeEncoder intakeEncoder;
    private PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    //private DigitalInput noteDetector;
    //private boolean notePresent = false; 

    public Intake() {
        intakeMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        intakeMotor.setInverted(true);
    }

    public void periodic() {
        if (notePresent()) {
             stopintake();
        }

        SmartDashboard.putNumber("Current",pdp.getCurrent(6));
    }

    public void setspeed(double i){
           // if (speed == 0) {
                speed = i;
            // } else {
            //     speed = 0;
            // }
            //speed = i; 
            intakeMotor.set(speed);
    }

    private boolean notePresent(){
        return pdp.getCurrent(6)>15;
    }

    private void stopintake(){
        intakeMotor.set(0);
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

//Travvypaddyepicgamer was here!!!!!