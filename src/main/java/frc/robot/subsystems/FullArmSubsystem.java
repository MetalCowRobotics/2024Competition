package frc.robot.subsystems;
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
            // if the arm and wrist are at rest, then set the arm to 0 and the wrist to -35    
        }else{
            m_ArmSubsystem.setTarget(targetPosArm);
            m_WristSubsystem.setTarget(targetPosWrist);
            // if the arm and wrist are not at rest, then set the arm to the target position and the wrist to the target position
        }
    }

    public void keepWristIn(){
        if(((armTarget > 20) && (m_ArmSubsystem.getAvgCurrentAngle() < 47)) || (armTarget < 30) && (m_ArmSubsystem.getAvgCurrentAngle() > 5)) {
            m_WristSubsystem.setTarget(-48);
            // bring wrist in when moving up
        } else {
            m_WristSubsystem.setTarget(wristTarget);
            // wrist go to position
        }
    }

    public void keepArmIn(){
        if((m_WristSubsystem.getCurrentAngle() > -33) && (armTarget > 10) && (m_ArmSubsystem.getAvgCurrentAngle() < 20)){
            m_ArmSubsystem.setTarget(m_ArmSubsystem.getAvgCurrentAngle());
            // stop arm until wrist is clear of hard stop
        } else if ((m_WristSubsystem.getCurrentAngle() < 2) && (armTarget > 220) ){
            m_ArmSubsystem.setTarget(225);
            // stop arm until wrist is clear of the bumpers
        } else if ((m_WristSubsystem.getCurrentAngle() < 13) && (armTarget > 230) ){
            m_ArmSubsystem.setTarget(240);
            // stop arm until wrist is clear of the bumpers
        } else {
            m_ArmSubsystem.setTarget(armTarget);
            // arm go to position
        }
    }

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

    public void setClimbVertPosition(){
        armTarget = Constants.FullArmConstants.armClimbVert;
        wristTarget = Constants.FullArmConstants.wristClimbVert;
        shortCircut(armTarget, wristTarget);
    }

    public void setClimbFinPosition(){
        armTarget = Constants.FullArmConstants.armClimbFin;
        wristTarget = Constants.FullArmConstants.wristClimbFin;
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
        keepWristIn();
        keepArmIn();
        System.out.println("CurrentPos: " + m_WristSubsystem.getCurrentAngle());
        System.out.println(m_WristSubsystem.getTargetAngle());
        
        m_WristSubsystem.periodic();
        m_ArmSubsystem.periodic();
    }
}