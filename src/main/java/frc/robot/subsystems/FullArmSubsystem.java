package frc.robot.subsystems;

import frc.robot.Constants;

public class FullArmSubsystem {
    private ArmSubsystem m_ArmSubsystem;
    private WristSubsystem m_WristSubsystem;
    private String positionIdentifier;

    public FullArmSubsystem(){
        m_ArmSubsystem = new ArmSubsystem();
        m_WristSubsystem =  new WristSubsystem();
        positionIdentifier = "Rest";
    }

    public String getPositionIdentifier(){
        return positionIdentifier;
    }

    public void setPositionIdentifier(String desiredPos){
        positionIdentifier = desiredPos;
    }

    public void move(){
        m_ArmSubsystem.setTarget(60);
    }

    public void no() {
        m_ArmSubsystem.setTarget(0);
    }

    public void periodic(){
        if(positionIdentifier.equalsIgnoreCase("PickUpToRest")){

        } else if(positionIdentifier.equalsIgnoreCase("RestToPickUp")){
            // m_ArmSubsystem.setTarget(Constants.FullArmConstants.armSubsystem_Rest);
            // m_WristSubsystem.setTarget(Constants.FullArmConstants.wristSubsystem_Rest);
            if(m_ArmSubsystem.atAngle(Constants.FullArmConstants.armSubsystem_Rest) && m_WristSubsystem.atAngle(Constants.FullArmConstants.wristSubsystem_Rest)){
                m_ArmSubsystem.setTarget(Constants.FullArmConstants.armSubsystem_Rest);
                m_WristSubsystem.setTarget(Constants.FullArmConstants.wristSubsystem_PickupRest);
                System.out.println("Phase1");
            }else if(m_ArmSubsystem.atAngle(Constants.FullArmConstants.armSubsystem_Rest) && m_WristSubsystem.atAngle(Constants.FullArmConstants.wristSubsystem_PickupRest)){
                m_ArmSubsystem.setTarget(Constants.FullArmConstants.armSubsystem_PickupRest);
                m_WristSubsystem.setTarget(Constants.FullArmConstants.wristSubsystem_PickupRest);
                System.out.println("Phase2");        
            }else if(m_ArmSubsystem.atAngle(Constants.FullArmConstants.armSubsystem_PickupRest) && m_WristSubsystem.atAngle(Constants.FullArmConstants.wristSubsystem_PickupRest)){
                m_ArmSubsystem.setTarget(Constants.FullArmConstants.armSubsystem_Pickup);
                m_WristSubsystem.setTarget(Constants.FullArmConstants.wristSubsystem_Pickup);       
                System.out.println("Phase3");  
            }
        } else if(positionIdentifier.equalsIgnoreCase("AmpToRest")){

        } else if(positionIdentifier.equalsIgnoreCase("RestToAmp")){

        } else if(positionIdentifier.equalsIgnoreCase("SpeakerToRest")){

        } else if(positionIdentifier.equalsIgnoreCase("RestToSpeaker")){

        } else if(positionIdentifier.equalsIgnoreCase("Rest")){
            m_ArmSubsystem.setTarget(Constants.FullArmConstants.armSubsystem_Rest);
            m_WristSubsystem.setTarget(Constants.FullArmConstants.wristSubsystem_Rest);
        } else{
            m_ArmSubsystem.setTarget(Constants.FullArmConstants.armSubsystem_Rest);
            m_WristSubsystem.setTarget(Constants.FullArmConstants.wristSubsystem_Rest);

        }
        m_ArmSubsystem.periodic();
        m_WristSubsystem.periodic();

    }



}
