package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    private CANSparkMax intakeMotor;   
    private double speed = 0;
    private DigitalInput noteDetector;

    public Intake() {
        intakeMotor = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        noteDetector = new DigitalInput(0);
    }

    public void periodic() {
        intakeMotor.set(speed);
    }

    public void setIntakingSpeed() {
        speed = 0.85;
    }

    public void setStopSpeed() {
        speed = 0;
    }

    // public void runUntilNote() {
    //     if (!noteDetector.get()) {
    //         periodic();
    //     }
    // }
}