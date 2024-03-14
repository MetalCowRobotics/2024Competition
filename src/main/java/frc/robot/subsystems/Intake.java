package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Intake {
    private CANSparkMax intakeMotor;   
    private double speed = 0;
    // private RelativeEncoder intakeEncoder;
    private PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    private Timer timer = new Timer();
    private Timer startUp = new Timer();
    private double expectedTime = .16; //.18
    private boolean notedetected = false;
    private boolean retractReady = false;
    private boolean autoMode = false;
    private boolean driving = false;
    //private DigitalInput noteDetector;
    //private boolean notePresent = false; 

    public Intake() {
        intakeMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        // intakeEncoder = intakeMotor.getEncoder();
        intakeMotor.setInverted(true);
        // noteDetector = new DigitalInput(0);
        // set up the intake motor and the note detector
    }

    // public void periodic() {
    //     if (intakeStatus) {
    //         intakeMotor.set(speed);
    //         // set speed of intake to 0.5
    //     }
    //     else {
    //          intakeMotor.set(0);
    //         }
    //         // set speed of intake to 0
    //     if (noteDetector.get()){
    //         intakeStatus = false;
    //         // if the note detector is pressed, then set the intake to false
    //     }    
    //     //noteDetector = new DigitalInput(0);
    //     SmartDashboard.putNumber("Velocity",intakeEncoder.getVelocity());
    // }
    //     SmartDashboard.putNumber("Velocity",intakeEncoder.getVelocity());
    // }

    public void periodic() {
        // if(!autoMode){
        SmartDashboard.putNumber("startUp", startUp.get());
        SmartDashboard.putBoolean("Note detected", notedetected);
        SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("Current",pdp.getCurrent(6));
        if(startUp.get() < .5){
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
        // } else System.out.println("speed < 0");

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

    // public void setspeed(double i){
    //        // if (speed == 0) {
    //             speed = i;
    //         // } else {
    //         //     speed = 0;
    //         // }
    //         //speed = i; 
    //         intakeMotor.set(speed);
    // }
    private void setStopDriving(){
        driving = true;
    }

    public void setStartDriving(){
        driving = false;
    }

    public boolean getStopDriving(){
        return driving;
    }
    
    public void setAutoMode(){
        autoMode = true;
    }

    private boolean notePresent(){
        return pdp.getCurrent(6) > 15;
    }

    public void setRetractReady(boolean b){
        retractReady = b;
    }

    public boolean getRetractReady(){
        return retractReady;
    }

    public void startIntake(){
        startUp.reset();
        startUp.start();
        speed = .9;
        setRetractReady(false);
        notedetected = false;
    }

    public void startIntakeReverse(){
        speed = -.8;
    }

    public void stopintake(){
        autoMode = false;
        speed = 0;
    }
}

//Travvypaddyepicgamer was here!!!!!