package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class NoteTransitSubsystem {
    private ArmSubsystem m_ArmSubsystem;
    private WristSubsystem m_WristSubsystem;
    private double armTarget;
    private double wristTarget;
    double[][] phase;
    int phaseNumber;

    public NoteTransitSubsystem(){
        m_ArmSubsystem = new ArmSubsystem();
        m_WristSubsystem =  new WristSubsystem();
        
        armTarget = Constants.FullArmConstants.armRest;
        wristTarget = Constants.FullArmConstants.wristRest;
    }



    // public void shortCircut(double targetPosArm, double targetPosWrist){
    //     if((targetPosArm == Constants.FullArmConstants.armRest) && (targetPosWrist == Constants.FullArmConstants.wristRest)){
    //         m_ArmSubsystem.setTarget(0);
    //         m_WristSubsystem.setTarget(-10);
    //         // if the arm and wrist are at rest, then set the arm to 0 and the wrist to -35    
    //     }else{
    //         m_ArmSubsystem.setTarget(targetPosArm);
    //         m_WristSubsystem.setTarget(targetPosWrist);
    //         // if the arm and wrist are not at rest, then set the arm to the target position and the wrist to the target position
    //     }
    // }


    public boolean atTarget(){
        return m_ArmSubsystem.atTarget() && m_WristSubsystem.atTarget();
    }

    public void setPickupPosition(){
        armTarget = Constants.FullArmConstants.armPickup;
        wristTarget = Constants.FullArmConstants.wristPickup;
        shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerPosition(){
        armTarget = Constants.FullArmConstants.armSpeaker;
        wristTarget = Constants.FullArmConstants.wristSpeaker;
        shortCircut(armTarget, wristTarget);
    }

    public void setStageShootingPosition(){
        armTarget = Constants.FullArmConstants.armStageShooting;
        wristTarget = Constants.FullArmConstants.wristStageShooting;
        shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerFromSpikeMark(){
        armTarget = Constants.FullArmConstants.armSpeakerFromNote;
        wristTarget = Constants.FullArmConstants.wristSpeakerFromNote;
        shortCircut(armTarget, wristTarget);
    }

    public void setRestPosition(){
        armTarget = Constants.FullArmConstants.armRest;
        wristTarget = Constants.FullArmConstants.wristRest;
        shortCircut(armTarget, wristTarget);
    }

    public void setAMPPosition(){
        armTarget = Constants.FullArmConstants.armAmp;
        wristTarget = Constants.FullArmConstants.wristAmp;
        shortCircut(armTarget, wristTarget);
    }

    public void periodic() {    
        SmartDashboard.putNumber("Arm Target", armTarget);
        SmartDashboard.putNumber("Wrist Target", wristTarget);

        System.out.println("CurrentPos: " + m_WristSubsystem.getCurrentAngle());
        System.out.println(m_WristSubsystem.getTargetAngle());
        
        m_WristSubsystem.periodic();
        m_ArmSubsystem.periodic();
    }
}