package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeJointSubsystem {
    private CANSparkMax intakeJointMotor;
    private RelativeEncoder encoder;

    private PIDController pidController;

    private double maxSetpoint;
    private double minSetpoint;
    private double targetAngle;

    private double nominalVoltage = 12.6;
    private double rampTime = 0.250;
    private CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kBrake;
    private int stallCurrentLimit = 30;
    private int freeCurrentLimit = 30;
    private double maxRPM = 6200; // 4000
    private double minRPM = 0; // 2000
    private double reduction = 100.0 * (60.0 / 18.0);
    private double kP = 0.04; // 0.015
    private double kI = 0.0;
    private double kD = 0.00;
    private double positionTolerance = 2;
    private double initialPosition = 0.0;

    public IntakeJointSubsystem() {
        intakeJointMotor = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);

        intakeJointMotor.enableVoltageCompensation(nominalVoltage);

        intakeJointMotor.setOpenLoopRampRate(rampTime);

        intakeJointMotor.setClosedLoopRampRate(rampTime);

        intakeJointMotor.setInverted(false);

        intakeJointMotor.setIdleMode(idleMode);

        intakeJointMotor.setSmartCurrentLimit(stallCurrentLimit, freeCurrentLimit);

        encoder = intakeJointMotor.getEncoder();
        
        maxSetpoint = maxRPM / 5820;
        minSetpoint = minRPM / 5820;

        pidController = new PIDController(kP, kI, kD);

        pidController.setIntegratorRange(-0.65, 0.65);
    }

    private boolean allowPositiveMotion(double angle) {
        return angle >= 0;
    }

    private boolean allowNegativeMotion(double angle) {
         return angle <= 180;
    }

    private double limitSpeed(double speed) {
        if (speed == 0) {
            return 0.0;
        }
        double absSpeed = Math.abs(speed);
		absSpeed = Math.max(absSpeed, minSetpoint);
		absSpeed = Math.min(absSpeed, maxSetpoint);
		return Math.copySign(absSpeed, speed);
    }

    public double getCurrentAngle() {
        return Units.rotationsToDegrees(encoder.getPosition() / reduction) + initialPosition;
    }

    public void resetEncoders(double angle) {
        encoder.setPosition(angle);
    }

    public void setTarget(double target) {
        this.targetAngle = target;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public boolean atTarget() {
        return Math.abs(targetAngle - getCurrentAngle()) < positionTolerance;
    }

    public boolean atAngle(double desiredAngle) {
        return Math.abs(desiredAngle - getCurrentAngle()) < positionTolerance;
    }

    private void writeStatus() {
        SmartDashboard.putNumber("Intake Angle", getCurrentAngle());
        // SmartDashboard.putNumber("Wrist Angular Velocity", Units.rotationsToDegrees(encoder.getVelocity() / reduction));
        // SmartDashboard.putNumber("Wrist Encoder Position", encoder.getPosition());
        // SmartDashboard.putNumber("Wrist encoder Velocity", encoder.getVelocity());
        // SmartDashboard.putNumber("Wrist Target Angle", targetAngle);
        // SmartDashboard.putNumber("Wrist Target Encoder Podition", setpoint);
    }

    public void periodic() {
        writeStatus();

        double speed = 0;

        pidController.setSetpoint(targetAngle);

        if (!atTarget()) {
            speed = pidController.calculate(getCurrentAngle());
        }

        speed = limitSpeed(speed);
        if (speed < 0) {
            if (!allowNegativeMotion(speed)) {
                speed = 0;
            }
        }

        if (speed > 0) {
            if (!allowPositiveMotion(speed)) {
                speed = 0;
            }
        }
        SmartDashboard.putNumber("Intake Joint Encoder Output", encoder.getPosition());
        SmartDashboard.putNumber("Intake Joint Motor Output", speed);

        if (atTarget()) {
            intakeJointMotor.set(0);
        } else {
            intakeJointMotor.set(speed);
        }
    }
}
