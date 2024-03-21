package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Intake {
    private CANSparkMax intakeMotor;
    private double speed = 0;
    private PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    private Timer timer = new Timer();
    private Timer startUp = new Timer();
    // private double expectedTime = 0.18;
    private double expectedTime = 0.0;
    private boolean notedetected = false;
    private boolean retractReady = false;
    private boolean driving = false;
    //private boolean notePresent = false; 

    public Intake() {
        intakeMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
    }

    public void periodic() {
        if ((notedetected)) {
            LED.runOrange();
        }
        SmartDashboard.putNumber("startUp", startUp.get());
        SmartDashboard.putBoolean("Note detected", notedetected);
        SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("Current",pdp.getCurrent(6));
        if(startUp.get() < 0.5){
            intakeMotor.set(speed);
            return;
        }
        if (noteAcquired()) {
            setRetractReady(true);
            stopintake();
            setStopDriving();
        }
        // if(speed >= 0){
        //     System.out.println("speed > 0");
        //     if (notePresent() && !notedetected) {
        //         notedetected = true;
        //         setRetractReady(true);
        //         timer.reset();
        //         timer.start();
        //     } 
        //     if (notedetected && timer.get() > expectedTime){
        //         stopintake();
        //         SmartDashboard.putString("auto", "stopped");
        //         // setStopDriving();
        //         timer.stop();
        //     }
        intakeMotor.set(speed);   
    }

    private boolean noteAcquired() {
        if (notePresent() && !notedetected) {
                notedetected = true;
                timer.reset();
                timer.start();
            } 
            if (notedetected && timer.get() >= expectedTime){
                SmartDashboard.putString("auto", "stopped");
                timer.stop();
                return true;
            }
        return false;
    }

    private void setStopDriving(){
        driving = true;
    }

    public void setStartDriving(){
        driving = false;
    }

    public boolean getStopDriving(){
        return driving;
    }

    private boolean notePresent(){
        return pdp.getCurrent(6) > 15;
    }

    public void resetNoteDetected(){
        notedetected = false;
        LED.runDefault();
    }

    // private void stopintake(){
    //     intakeMotor.set(0);
    // }

    // public void setIntakeTrue() {
    //     intakeStatus = true;
    //     // set intake to true
    // }

    // public void setIntakeFalse() {
    //     intakeStatus = false;
    //     // set intake to false
    // }
    public void setRetractReady(boolean b){
        retractReady = b;
    }

    public boolean getRetractReady(){
        return retractReady;
    }

    public void startIntake(){
        startUp.reset();
        startUp.start();
        speed = 0.85;
        setRetractReady(false);
        notedetected = false;
        
    }

    public void startIntakeReverse(){
        speed = -0.85;
    }

    public void feed(){
        startUp.reset();
        startUp.start();
        speed = 1;
        setRetractReady(false);
        notedetected = false;    }

    public void stopintake(){
        speed = 0;
    }
}