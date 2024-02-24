package frc.robot.subsystems;

import frc.robot.Constants;

public class FullArmSubsystem {
    private ArmSubsystem m_ArmSubsystem;
    private WristSubsystem m_WristSubsystem;
    int phaseNumber;

    public FullArmSubsystem(){
        m_ArmSubsystem = new ArmSubsystem();
        m_WristSubsystem =  new WristSubsystem();
        phaseNumber = 0;
    }

    private void nextPhase(){
        phaseNumber++;
        m_ArmSubsystem.setTarget(phase[phaseNumber][0]);
        m_WristSubsystem.setTarget(phase[phaseNumber][1]);
    }


    public void periodic(){

    }





}
