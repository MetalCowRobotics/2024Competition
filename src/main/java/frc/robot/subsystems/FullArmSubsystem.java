package frc.robot.subsystems;

import frc.robot.Constants;

public class FullArmSubsystem {
    private ArmSubsystem m_ArmSubsystem;
    private WristSubsystem m_WristSubsystem;
    double[][] phase;
    int phaseNumber;

    public FullArmSubsystem(){
        m_ArmSubsystem = new ArmSubsystem();
        m_WristSubsystem =  new WristSubsystem();
        phase = Constants.FullArmConstants.restPhases;
        phaseNumber = Constants.FullArmConstants.restPhases[1].length-1;
    }

    public int getPhase(){
        return phaseNumber;
    }

    private void nextPhase(){
        if(phaseNumber<=(phase[0].length - 1)){
            m_ArmSubsystem.setTarget(phase[0][phaseNumber]);
            m_WristSubsystem.setTarget(phase[1][phaseNumber]);
        }
        phaseNumber++;
    }

    public int shortCircut(double[][] targetPhase){
        System.out.println("Short Circuting");
        int closestPhase = 0;
        double minDiff = (Math.abs(phase[1][0] - m_WristSubsystem.getCurrentAngle()));
        if(targetPhase.equals(Constants.FullArmConstants.restPhases)){
            phaseNumber = 0;
        }
        for(int i = 0; i<phase[1].length;i++){
            if((Math.abs(phase[1][i] - m_WristSubsystem.getCurrentAngle())) < minDiff){
                minDiff = (Math.abs(phase[1][i] - m_WristSubsystem.getCurrentAngle()));
                closestPhase = i;
            }
        }
        System.out.print(closestPhase);
        return closestPhase;
    }

    public void setIntakePosition(){
        phaseNumber = shortCircut(Constants.FullArmConstants.pickupPhases);
        phase = Constants.FullArmConstants.pickupPhases;
        
    }

    public void setRestPosition(){
        phaseNumber = shortCircut(Constants.FullArmConstants.restPhases);
        phase = Constants.FullArmConstants.restPhases;
    }

    public void setClimb1Position(){
        phaseNumber = shortCircut(Constants.FullArmConstants.climbPhases1);
        phase = Constants.FullArmConstants.climbPhases1;
    }

    public void setClimb2Position(){
        phaseNumber = shortCircut(Constants.FullArmConstants.climbPhases2);
        phase = Constants.FullArmConstants.climbPhases2;
    }

    public void setAmpPosition(){
        phaseNumber = shortCircut(Constants.FullArmConstants.ampPhases);
        phase = Constants.FullArmConstants.ampPhases;
    }

    public void setSpeakerPosition(){
        phaseNumber = shortCircut(Constants.FullArmConstants.speakerPhases);
        phase = Constants.FullArmConstants.speakerPhases;
    }

    public void periodic() {
        if (m_ArmSubsystem.atTarget() && m_WristSubsystem.atTarget()) {
            nextPhase();
        }
        m_ArmSubsystem.periodic();
        m_WristSubsystem.periodic();
    }







}
