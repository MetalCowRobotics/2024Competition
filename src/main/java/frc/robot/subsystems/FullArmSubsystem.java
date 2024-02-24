package frc.robot.subsystems;

import frc.robot.Constants;

public class FullArmSubsystem {
    private ArmSubsystem m_ArmSubsystem;
    private WristSubsystem m_WristSubsystem;
    double[][] phase;
    int phaseNumber;
    boolean reset;

    public FullArmSubsystem(){
        m_ArmSubsystem = new ArmSubsystem();
        m_WristSubsystem =  new WristSubsystem();
        phase = Constants.FullArmConstants.restPhases;
        // phaseNumber = Constants.FullArmConstants.restPhases[1].length-1;
        phaseNumber = 4;
        reset = true;
    }

    private void nextPhase(){
        phaseNumber++;
        if(phaseNumber<=4){
         m_ArmSubsystem.setTarget(phase[0][phaseNumber]);
         m_WristSubsystem.setTarget(phase[1][phaseNumber]);
        }
        // if (phaseNumber > 4) {
        //     reset = false;
        // }
    }

    public void periodic() {
        if (m_ArmSubsystem.atTarget() && m_WristSubsystem.atTarget()) {
            nextPhase();
        }
        m_ArmSubsystem.periodic();
        m_WristSubsystem.periodic();

    }

    public void setIntakePosition(){
        phaseNumber = 0;
        phase = Constants.FullArmConstants.pickupPhases;
    }

    public void setRestPosition(){
        phaseNumber = 0;
        phase = Constants.FullArmConstants.restPhases;
    }

    public void setClimb1Position(){
        phaseNumber = 0;
        phase = Constants.FullArmConstants.climbPhases1;
    }

    public void setClimb2Position(){
        phaseNumber = 0;
        phase = Constants.FullArmConstants.climbPhases2;
    }

    // public void setSpeakerPosition() {
    //     phase = 
    // }

    // public void setAmpPosition() {
    //     phase = 
    // }






}
