package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ShooterJointSubsystem {

    private static ShooterJointSubsystem instance = new ShooterJointSubsystem();
    private CANSparkMax shooterJointMotor;
    private RelativeEncoder encoder;

    private PIDController pidController;

    private double maxSetpoint;
    private double minSetpoint;
    private double targetAngle;

    private DigitalInput boreInput;
    private DutyCycleEncoder boreEncoder;
    private double boreRawValue, boreConvertedValue, boreConvertedOffsetValue;

    private double nominalVoltage = 12.6;
    private double rampTime = 0.250;
    private CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kBrake;
    private int stallCurrentLimit = 30;
    private int freeCurrentLimit = 30;
    private double maxRPM = 6200; // 4000
    private double minRPM = 0; // 2000
    private double reduction = 100.0 * (24.0 / 12.0);
    private double kP = 0.04; // 0.015
    private double kI = 0.0;
    private double kD = 0.00;
    private double positionTolerance = 2;
    private double initialPosition = 0.0;

    private ShooterJointSubsystem() {
        shooterJointMotor = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);

        shooterJointMotor.enableVoltageCompensation(nominalVoltage);

        shooterJointMotor.setOpenLoopRampRate(rampTime);

        shooterJointMotor.setClosedLoopRampRate(rampTime);

        shooterJointMotor.setInverted(false);

        shooterJointMotor.setIdleMode(idleMode);

        shooterJointMotor.setSmartCurrentLimit(stallCurrentLimit, freeCurrentLimit);

        encoder = shooterJointMotor.getEncoder();

        boreInput = new DigitalInput(2);
        boreEncoder = new DutyCycleEncoder(boreInput);
        
        maxSetpoint = maxRPM / 5820;
        minSetpoint = minRPM / 5820;

        pidController = new PIDController(kP, kI, kD);

        pidController.setIntegratorRange(-0.65, 0.65);

        SmartDashboard.putNumber("ShooterJointkp", kP);
        SmartDashboard.putNumber("ShooterJointkp", kI);
        SmartDashboard.putNumber("ShooterJointkp", kD);
    }

    public static ShooterJointSubsystem getInstance(){
        return instance;
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
        return -((boreEncoder.get() * 360) - 329);
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

    private void writeStatus() {
        SmartDashboard.putNumber("Shooter Angle", getCurrentAngle());
        // SmartDashboard.putNumber("Wrist Angular Velocity", Units.rotationsToDegrees(encoder.getVelocity() / reduction));
        // SmartDashboard.putNumber("Wrist Encoder Position", encoder.getPosition());
        // SmartDashboard.putNumber("Wrist encoder Velocity", encoder.getVelocity());
        // SmartDashboard.putNumber("Wrist Target Angle", targetAngle);
        // SmartDashboard.putNumber("Wrist Target Encoder Podition", setpoint);
    }

    public void periodic() {
        writeStatus();

        pidController.setPID(SmartDashboard.getNumber("ShooterJointkP", kP), SmartDashboard.getNumber("ShooterJointkI", kI), SmartDashboard.getNumber("ShooterJointkD", kD));
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
        // SmartDashboard.putNumber("Shooter Joint Encoder Output", encoder.getPosition());
        SmartDashboard.putNumber("Shooter Joint Motor Output", speed);

        if (atTarget()) {
            shooterJointMotor.set(0);
        } else {
            shooterJointMotor.set(speed);
        }
    }
}
