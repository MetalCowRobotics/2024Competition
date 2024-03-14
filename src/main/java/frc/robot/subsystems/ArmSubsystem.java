package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem {
    private CANSparkMax armMotor1;
    private CANSparkMax armMotor2;
    private RelativeEncoder encoder1;
    private RelativeEncoder encoder2;

    private PIDController pidController1;
    private PIDController pidController2;

    private double maxSetpoint;
    private double minSetpoint;
    private double setpoint;
    private double targetAngle;

    private double nominalVoltage = 12.6;
    private double rampTime = 0.125;
    private CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kBrake;
    private int stallCurrentLimit = 30;
    private int freeCurrentLimit = 30;
    private double maxRPM = 5500; 
    private double minRPM = 3000;
    private double reduction = 100 * (24.0 / 12.0);
    private double kP = 0.05; // 0.07
    private double kI = 0.0;
    private double kD = 0.0;
    private double positionTolerance = 4.0;
    private double initialPosition = 0.0;
    
    public ArmSubsystem() {
        armMotor1 = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        armMotor2 = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);

        armMotor1.enableVoltageCompensation(nominalVoltage);
        armMotor2.enableVoltageCompensation(nominalVoltage);

        armMotor1.setOpenLoopRampRate(rampTime);
        armMotor2.setOpenLoopRampRate(rampTime);

        armMotor1.setClosedLoopRampRate(rampTime);
        armMotor2.setClosedLoopRampRate(rampTime);

        armMotor1.setInverted(false);
        armMotor2.setInverted(true);

        armMotor1.setIdleMode(idleMode);
        armMotor2.setIdleMode(idleMode);

        armMotor1.setSmartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
        armMotor2.setSmartCurrentLimit(stallCurrentLimit, freeCurrentLimit);

        encoder1 = armMotor1.getEncoder();
        encoder2 = armMotor2.getEncoder();

        maxSetpoint = maxRPM / 5820;
        minSetpoint = minRPM / 5820;

        pidController1 = new PIDController(kP, kI, kD);
        pidController2 = new PIDController(kP, kI, kD);

        pidController1.setIntegratorRange(-0.6, 0.6);
        pidController2.setIntegratorRange(-0.6, 0.6);
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

    public double getEncoder1CurrentAngle() {
        return Units.rotationsToDegrees(encoder1.getPosition() / reduction) + initialPosition;
    }

    public double getEncoder2CurrentAngle() {
        return Units.rotationsToDegrees(encoder2.getPosition() / reduction) + initialPosition;
    }

    public double getAvgCurrentAngle(){
        return ((getEncoder1CurrentAngle()+getEncoder2CurrentAngle())/2);
    }

    public double getWristAngle(double angle) {
        return angle;
    }

    public void resetEncoders(double angle) {
        encoder1.setPosition(angle);
        encoder2.setPosition(angle);
    }

    public void setTarget(double target) {
        this.targetAngle = target;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public boolean atTarget() {
        if (Math.abs(targetAngle - getEncoder1CurrentAngle()) < positionTolerance
            &&
            Math.abs(targetAngle - getEncoder2CurrentAngle()) < positionTolerance) {    
                return true;
            }
        else {
            return false;
        }
    }

   public boolean atAngle(double desiredAngle) {
        if (Math.abs(desiredAngle - getEncoder1CurrentAngle()) < positionTolerance
            &&
            Math.abs(desiredAngle - getEncoder2CurrentAngle()) < positionTolerance) {
                return true;
            }
        else {
            return false;
        }
    }
    private void writeStatus() {
        SmartDashboard.putNumber("Arm 1 Angle", getEncoder1CurrentAngle());
        SmartDashboard.putNumber("Arm 2 Angle", getEncoder2CurrentAngle());

        // SmartDashboard.putNumber("Arm 1 Angular Velocity", Units.rotationsToDegrees(encoder1.getVelocity() / reduction));
        // SmartDashboard.putNumber("Arm 2 Angular Velocity", Units.rotationsToDegrees(encoder2.getVelocity() / reduction));

        // SmartDashboard.putNumber("Arm 1 Encoder Position", encoder1.getPosition());
        // SmartDashboard.putNumber("Arm 2 Encoder Position", encoder2.getPosition());

        // SmartDashboard.putNumber("Arm 1 encoder Velocity", encoder1.getVelocity());
        // SmartDashboard.putNumber("Arm 2 encoder Velocity", encoder2.getVelocity());

        // SmartDashboard.putNumber("Arm Target Angle", targetAngle);
        SmartDashboard.putNumber("Arm Target Encoder Podition", setpoint);
    }

    public void periodic() {
        writeStatus();

        double speed1 = 0;
        double speed2 = 0;

        pidController1.setSetpoint(targetAngle);
        pidController2.setSetpoint(targetAngle);

        if (!atTarget()) {
            speed1 = pidController1.calculate(getEncoder1CurrentAngle());
            speed2 = pidController2.calculate(getEncoder2CurrentAngle());
        }

        speed1 = limitSpeed(speed1);
        if (speed1 < 0) {
            if (!allowNegativeMotion(speed1)) {
                speed1 = 0;
            }
        }

        speed2 = limitSpeed(speed2);
        if (speed2 < 0) {
            if (!allowNegativeMotion(speed2)) {
                speed2 = 0;
            }
        }

        if (speed1 > 0) {
            if (!allowPositiveMotion(speed1)) {
                speed1 = 0;
            }
        }

        if (speed2 > 0) {
            if (!allowPositiveMotion(speed2)) {
                speed2 = 0;
            }
        }

        SmartDashboard.putNumber("Arm 1 Motor Output", speed1);
        SmartDashboard.putNumber("Arm 2 Motor Output", speed2);

        if (atTarget()) {
            armMotor1.set(0);
            armMotor2.set(0);
        } else {
            armMotor1.set(speed1);
            armMotor2.set(speed2);
        }
    }
}