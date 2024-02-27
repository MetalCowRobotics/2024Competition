package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmToAngles extends CommandBase {
    
    private WristSubsystem m_wrist;
    private ElbowSubsystem m_elbow;
    private ShoulderSubsystem m_shoulder;

    private double wristAngle;
    private double elbowAngle;
    private double shoulderAngle;

    public ArmToAngles(WristSubsystem wrist, ElbowSubsystem elbow, ShoulderSubsystem shoulder, double shoulderAngle, double elbowAngle, double wristAngle) {
        this.m_wrist = wrist;
        this.m_elbow = elbow;
        this.m_shoulder = shoulder;

        this.wristAngle = wristAngle;
        this.elbowAngle = elbowAngle;
        this.shoulderAngle = shoulderAngle;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        m_wrist.setTarget(wristAngle);
        m_elbow.setTarget(elbowAngle);
        m_shoulder.setTarget(shoulderAngle);
    }

    @Override
    public boolean isFinished() {
        return (m_wrist.atTarget() && m_shoulder.atTarget()) && m_elbow.atTarget();
    }

}