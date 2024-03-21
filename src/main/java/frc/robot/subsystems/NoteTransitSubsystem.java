package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class NoteTransitSubsystem {

    private ConveyorBeltSubsystem m_ConveyorBeltSubsystem;
    private IntakeJointSubsystem m_IntakeJointSubsystem;
    private ShooterJointSubsystem m_ShooterJointSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    private double shooterTarget;
    private double intakeTarget;


    public NoteTransitSubsystem(){
        m_ConveyorBeltSubsystem = new ConveyorBeltSubsystem();
        m_IntakeJointSubsystem = new IntakeJointSubsystem();
        m_ShooterJointSubsystem = new ShooterJointSubsystem();
        m_IntakeSubsystem = new IntakeSubsystem();


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
        return m_IntakeJointSubsystem.atTarget() && m_ShooterJointSubsystem.atTarget();
    }

    public void setPickupPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeDeployed;
        //shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeDeployed;
        //shortCircut(armTarget, wristTarget);
    }

    public void setStageShootingPosition(){
        shooterTarget = Constants.JointConstants.shooterFar;
        intakeTarget = Constants.JointConstants.intakeDeployed;
        //shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerFromSpikeMark(){
        shooterTarget = Constants.JointConstants.shooterFar;
        intakeTarget = Constants.JointConstants.intakeDeployed;
       // shortCircut(armTarget, wristTarget);
    }

    public void setRestPosition(){
        shooterTarget = Constants.JointConstants.shooterStart;
        intakeTarget = Constants.JointConstants.intakeStart;
        //shortCircut(armTarget, wristTarget);
    }

    public void setAMPPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeAmp;
        //shortCircut(armTarget, wristTarget);
    }

    public void periodic() {    
        SmartDashboard.putNumber("Intake Target", intakeTarget);
        SmartDashboard.putNumber("Shooter Target", shooterTarget);
        
        m_ConveyorBeltSubsystem.periodic();
        m_IntakeSubsystem.periodic();
        m_IntakeJointSubsystem.periodic();
        m_ShooterJointSubsystem.periodic();
    }
}