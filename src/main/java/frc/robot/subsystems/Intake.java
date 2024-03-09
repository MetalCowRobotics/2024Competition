package frc.robot.subsystems;

import com.revrobotics.CANSparkLowlevel;
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
        //noteDetector = new DigitalInput(0);
        SmartDashboard.putNumber("Velocity",intakeEncoder.getVelocity());
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