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
        // phase = Constants.FullArmConstants.restPhases;
        // phaseNumber = Constants.FullArmConstants.restPhases[1].length-1;
    }

    public int getPhase(){
        return phaseNumber;
    }

    // private void nextPhase(){
    //     if(phaseNumber<=(phase[0].length - 1)){
    //         m_ArmSubsystem.setTarget(phase[0][phaseNumber]);
    //         m_WristSubsystem.setTarget(phase[1][phaseNumber]);
    //     }
    //     phaseNumber++;
    // }
//In terms of WRIST angle
/* 
    public int shortCircut(double[][] targetPhase){
        //System.out.print("Short Circuting: ");
        int closestPhase = 0;
        double minDiff = (Math.abs(targetPhase[1][0] - m_WristSubsystem.getCurrentAngle()));
        if(targetPhase.equals(Constants.FullArmConstants.restPhases)){
            phaseNumber = 0;
        }
        else{
            for(int i = 0; i<targetPhase[1].length;i++){
                if((Math.abs(targetPhase[1][i] - m_WristSubsystem.getCurrentAngle())) < minDiff){
                    minDiff = (Math.abs(targetPhase[1][i] - m_WristSubsystem.getCurrentAngle()));
                    closestPhase = i;
                }
            }
        }
        m_ArmSubsystem.setTarget(phase[0][closestPhase]);
        m_WristSubsystem.setTarget(phase[1][closestPhase]);
        System.out.println("\nClosest Phase: " + closestPhase);
        System.out.println("Current Angle: " + m_WristSubsystem.getCurrentAngle());
        System.out.println("Phase Angle: " + phase[1][closestPhase]+"\n");
        return closestPhase;
    }
    */

//In terms of ARM angle
    // public int shortCircut(double[][] targetPhase){
    //     //System.out.print("Short Circuting: ");
    //     int closestPhase = 0;
    //     double minDiff = (Math.abs(targetPhase[0][0] - m_ArmSubsystem.getAvgCurrentAngle()));
    //     if(targetPhase.equals(Constants.FullArmConstants.restPhases)){
    //         phaseNumber = 0;
    //     }
    //     else{
    //         for(int i = 0; i<targetPhase[0].length;i++){
    //             if((Math.abs(targetPhase[0][i] - m_ArmSubsystem.getAvgCurrentAngle())) < minDiff){
    //                 minDiff = (Math.abs(targetPhase[0][i] - m_ArmSubsystem.getAvgCurrentAngle()));
    //                 closestPhase = i;
    //             }
    //         }
    //     }
    //     m_ArmSubsystem.setTarget(phase[0][closestPhase]);
    //     m_WristSubsystem.setTarget(phase[1][closestPhase]);
    //     System.out.println("\nClosest Phase: " + closestPhase);
    //     System.out.println("Current Angle: " + m_ArmSubsystem.getAvgCurrentAngle());
    //     System.out.println("Phase Angle: " + phase[0][closestPhase]+"\n");
    //     return closestPhase;
    // }

    public void shortCircut(double targetPosArm, double targetPosWrist){
        if((targetPosArm == Constants.FullArmConstants.armRest) && (targetPosWrist == Constants.FullArmConstants.wristRest)){
            m_ArmSubsystem.setTarget(0);
            m_WristSubsystem.setTarget(-30);    
        }else{
            m_ArmSubsystem.setTarget(targetPosArm);
            m_WristSubsystem.setTarget(targetPosWrist);
        }
    }

    public void keepWristIn(){
        if((m_ArmSubsystem.getAvgCurrentAngle()>5)&&(m_ArmSubsystem.getAvgCurrentAngle()<70)){
            m_WristSubsystem.setTarget(-30);
        }
    }

    // public void setIntakePosition(){
    //     phaseNumber = shortCircut(Constants.FullArmConstants.pickupPhases);
    //     phase = Constants.FullArmConstants.pickupPhases;
        
    //}
    public void setPickupPosition(){
        armTarget = Constants.FullArmConstants.armPickup;
        wristTarget = Constants.FullArmConstants.wristPickup;
        shortCircut(armTarget, wristTarget);

    }


    // public void setRestPosition(){
    //     phaseNumber = shortCircut(Constants.FullArmConstants.restPhases);
    //     phase = Constants.FullArmConstants.restPhases;
    // }
    public void setRestPosition(){
        armTarget = Constants.FullArmConstants.armRest;
        wristTarget = Constants.FullArmConstants.wristRest;
        shortCircut(armTarget, wristTarget);

    }

    // public void setClimb1Position(){
    //     phaseNumber = shortCircut(Constants.FullArmConstants.climbPhases1);
    //     phase = Constants.FullArmConstants.climbPhases1;
    // }
    public void setClimbVertPosition(){
        armTarget = Constants.FullArmConstants.armClimbVert;
        wristTarget = Constants.FullArmConstants.wristClimbVert;
        shortCircut(armTarget, wristTarget);

    }

    // public void setClimb2Position(){
    //     phaseNumber = shortCircut(Constants.FullArmConstants.climbPhases2);
    //     phase = Constants.FullArmConstants.climbPhases2;
    // }
    public void setClimbFinPosition(){
        armTarget = Constants.FullArmConstants.armClimbFin;
        wristTarget = Constants.FullArmConstants.wristClimbFin;
        shortCircut(armTarget, wristTarget);

    }

    // public void setAmpPosition(){
    //     phaseNumber = shortCircut(Constants.FullArmConstants.ampPhases);
    //     phase = Constants.FullArmConstants.ampPhases;
    // }

    // public void setSpeakerPosition(){
    //     phaseNumber = shortCircut(Constants.FullArmConstants.speakerPhases);
    //     phase = Constants.FullArmConstants.speakerPhases;
    // }

    public void periodic() {    
        SmartDashboard.putBoolean("Safe", !((m_ArmSubsystem.getAvgCurrentAngle()>5)&&(m_ArmSubsystem.getAvgCurrentAngle()<70)));
        keepWristIn();
        if(m_WristSubsystem.atTarget()){
            m_ArmSubsystem.setTarget(armTarget);
        }
        //
        System.out.println("CurrentPos: " + m_WristSubsystem.getCurrentAngle());
        System.out.println(m_WristSubsystem.getTargetAngle());
        m_WristSubsystem.periodic();
        m_ArmSubsystem.periodic();

    }
    // public void periodic() {
    //     if (m_ArmSubsystem.atTarget() && m_WristSubsystem.atTarget()) {
    //         nextPhase();
    //     }
    //     m_ArmSubsystem.periodic();
    //     m_WristSubsystem.periodic();
    // }







}
