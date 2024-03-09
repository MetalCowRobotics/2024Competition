package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Intake {
    private CANSparkMax intakeMotor;   
    private double speed = 0;
    private RelativeEncoder intakeEncoder;
    private PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    private Timer timer = new Timer();
    private double expectedTime = .18;
    private boolean notedetected = false;
    private boolean retractReady = false;
    //private DigitalInput noteDetector;
    //private boolean notePresent = false; 

    public Intake() {
        intakeMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        intakeMotor.setInverted(true);
        SmartDashboard.putNumber("Velocity",intakeEncoder.getVelocity());
    }

    public void periodic() {
        if(speed >= 0){
            System.out.println("speed > 0");
            if (notePresent() && !notedetected) {
                notedetected = true;
                setRetractReady(true);
                timer.reset();
                timer.start();

            } 
            if (notedetected && timer.get() > expectedTime){
                stopintake();
                timer.stop();
            }
        } else System.out.println("speed < 0");
        intakeMotor.set(speed);
        SmartDashboard.putNumber("Current",pdp.getCurrent(6));
    }

    // public void setspeed(double i){
    //        // if (speed == 0) {
    //             speed = i;
    //         // } else {
    //         //     speed = 0;
    //         // }
    //         //speed = i; 
    //         intakeMotor.set(speed);
    // }

    private boolean notePresent(){
        return pdp.getCurrent(6)>15;
    }

    public void setRetractReady(boolean b){
        retractReady = b;
    }

    public boolean getRetractReady(){
        return retractReady;
    }

    public void startIntake(){
        speed = .9;
        setRetractReady(false);
        notedetected = false;
    }

    public void startIntakeReverse(){
        speed = -.9;
    }

    public void stopintake(){
        speed = 0;
    }
}

//Travvypaddyepicgamer was here!!!!!