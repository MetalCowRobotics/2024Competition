package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class FullArmSubsystem {
    private ArmSubsystem m_ArmSubsystem;
    private WristSubsystem m_WristSubsystem;
    private double armTarget;
    private double wristTarget;
    double[][] phase;
    int phaseNumber;

    public FullArmSubsystem(){
        m_ArmSubsystem = new ArmSubsystem();
        m_WristSubsystem =  new WristSubsystem();
        armTarget = Constants.FullArmConstants.armRest;
        wristTarget = Constants.FullArmConstants.wristRest;
    }

    public int getPhase(){
        return phaseNumber;
    }

    public void shortCircut(double targetPosArm, double targetPosWrist){
        if((targetPosArm == Constants.FullArmConstants.armRest) && (targetPosWrist == Constants.FullArmConstants.wristRest)){
            m_ArmSubsystem.setTarget(0);
            m_WristSubsystem.setTarget(-35);    
        }else{
            m_ArmSubsystem.setTarget(targetPosArm);
            m_WristSubsystem.setTarget(targetPosWrist);
        }
    }

    public void keepWristIn(){
        if((m_ArmSubsystem.getAvgCurrentAngle()>5)&&(m_ArmSubsystem.getAvgCurrentAngle()<100)){
            m_WristSubsystem.setTarget(-35);
        }
        else{
            m_WristSubsystem.setTarget(wristTarget);
        }
    }

    public void setPickupPosition(){
        armTarget = Constants.FullArmConstants.armPickup;
        wristTarget = Constants.FullArmConstants.wristPickup;
        shortCircut(armTarget, wristTarget);

    }

    public void setRestPosition(){
        armTarget = Constants.FullArmConstants.armRest;
        wristTarget = Constants.FullArmConstants.wristRest;
        shortCircut(armTarget, wristTarget);

    }

    public void setClimbVertPosition(){
        armTarget = Constants.FullArmConstants.armClimbVert;
        wristTarget = Constants.FullArmConstants.wristClimbVert;
        shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerPosition(){
        armTarget = Constants.FullArmConstants.armSpeaker;
        wristTarget = Constants.FullArmConstants.wristSpeaker;
        shortCircut(armTarget, wristTarget);
    }

    public void setClimbFinPosition(){
        armTarget = Constants.FullArmConstants.armClimbFin;
        wristTarget = Constants.FullArmConstants.wristClimbFin;
        shortCircut(armTarget, wristTarget);

    }
    
    public void periodic() {    
        keepWristIn();
        System.out.println("CurrentPos: " + m_WristSubsystem.getCurrentAngle());
        System.out.println(m_WristSubsystem.getTargetAngle());
        m_WristSubsystem.periodic();
        m_ArmSubsystem.periodic();

    }
}
