/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmToAngles extends Command{
    
    private WristSubsystem m_wrist;
    private ArmSubsystem a_arm;

    private double wristAngle;
    private double armAngle;

    public ArmToAngles(WristSubsystem wrist, ArmSubsystem arm, double wristAngle, double armAngle) {
        this.m_wrist = wrist;
        this.a_arm = arm;

        this.wristAngle = wristAngle;
        this.armAngle = armAngle;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        m_wrist.setTarget(wristAngle);
        a_arm.setTarget(armAngle);
    }

    @Override
    public boolean isFinished() {
        return (m_wrist.atTarget() && a_arm.atTarget());
    }

}
*/