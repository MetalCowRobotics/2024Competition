package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class NoteTransitSubsystem {

    private static NoteTransitSubsystem instance = new NoteTransitSubsystem();
    private IntakeJointSubsystem m_IntakeJointSubsystem;
    private ShooterJointSubsystem m_ShooterJointSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private Shooter m_Shooter;
    private boolean isShootingState;
    private boolean alreadyLiftedIntake;

    

    private double shooterTarget;
    private double intakeTarget;


    private NoteTransitSubsystem(){
        m_IntakeJointSubsystem = IntakeJointSubsystem.getInstance();
        m_ShooterJointSubsystem = ShooterJointSubsystem.getInstance();
        m_IntakeSubsystem = IntakeSubsystem.getInstance();
        m_Shooter = Shooter.getInstance();
        isShootingState = false;
        alreadyLiftedIntake = false;
    }

    public static NoteTransitSubsystem getInstance(){
        return instance;
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
        m_IntakeSubsystem.setPickupSpeed();
        isShootingState = false;
        //shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        isShootingState = true;
        //shortCircut(armTarget, wristTarget);
    }

    public void setStageShootingPosition(){
        shooterTarget = Constants.JointConstants.shooterFar;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        isShootingState = true;
        //shortCircut(armTarget, wristTarget);
    }

    public void setSpeakerFromSpikeMark(){
        shooterTarget = Constants.JointConstants.shooterFar;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        isShootingState = true;
       // shortCircut(armTarget, wristTarget);
    }

    public void setRestPosition(){
        shooterTarget = Constants.JointConstants.shooterStart;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.stopintake();
        isShootingState = false;
        //shortCircut(armTarget, wristTarget);
    }

    public void setAMPPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeAmp;
        m_IntakeSubsystem.setAmpSpeed();
        isShootingState = false;
        //shortCircut(armTarget, wristTarget);
    }

    public void toggleShooter(){
        m_Shooter.toggleShooter();
    }

    public void enableIntake(){
        if((!isShootingState) || ((isShootingState) && (m_Shooter.getShooterSpunUp()))){
            m_IntakeSubsystem.startIntake();
        }else{
            disableIntake();
        }
    }

    public void quickOuttake(){
        m_IntakeSubsystem.startIntakeReverse();
    }

    public void disableIntake(){
        m_IntakeSubsystem.stopintake();
    }



    public void periodic() {    
        SmartDashboard.putNumber("Intake Target", intakeTarget);
        SmartDashboard.putNumber("Shooter Target", shooterTarget);
        if(m_IntakeSubsystem.noteAcquired() && !alreadyLiftedIntake){
            setRestPosition();
            alreadyLiftedIntake = true;
        }
        m_IntakeSubsystem.periodic();
        m_IntakeJointSubsystem.periodic();
        m_ShooterJointSubsystem.periodic();
    }
}