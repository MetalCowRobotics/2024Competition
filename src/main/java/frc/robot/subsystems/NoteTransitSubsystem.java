package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class NoteTransitSubsystem {
    private static NoteTransitSubsystem instance = new NoteTransitSubsystem();
    private IntakeJointSubsystem m_IntakeJointSubsystem;
    private ShooterJointSubsystem m_ShooterJointSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private Shooter m_Shooter;
    private boolean isShootingState;
    private boolean alreadyLiftedIntake;
    private boolean isPickup;
    private double shooterTarget;
    private double intakeTarget;

    private NoteTransitSubsystem(){
        m_IntakeJointSubsystem = IntakeJointSubsystem.getInstance();
        m_ShooterJointSubsystem = ShooterJointSubsystem.getInstance();
        m_IntakeSubsystem = IntakeSubsystem.getInstance();
        m_Shooter = Shooter.getInstance();
        isShootingState = false;
        isPickup = false;
        alreadyLiftedIntake = false;
    }

    public static NoteTransitSubsystem getInstance(){
        return instance;
    } 

    public boolean atTarget(){
        SmartDashboard.putBoolean("atTarget", m_IntakeJointSubsystem.atTarget() && m_ShooterJointSubsystem.atTarget());
        return m_IntakeJointSubsystem.atTarget() && m_ShooterJointSubsystem.atTarget();
    }

    //Sets joints to pickup location, and sets the intake speed to the pickup speed for when it is enabled,
    public void setPickupPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeDeployed;
        m_IntakeSubsystem.setPickupSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = false;
    }

    //Sets joints to speaker location, and sets the intake speed to the feed speed for when it is enabled,
    public void setSpeakerPosition(){
        shooterTarget = Constants.JointConstants.shooterClose;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
    }

    //Sets joints to speaker location, and sets the intake speed to the feed speed for when it is enabled,
    public void setStageShootingPosition(){
        shooterTarget = Constants.JointConstants.shooterFar;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
    }

    //Sets joints to speaker from spike location, and sets the intake speed to the feed speed for when it is enabled,
    public void setSpeakerFromSpikeMark(){
        shooterTarget = SmartDashboard.getNumber("Shooter Far Target", Constants.JointConstants.shooterFar);
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
    }
   
    //Sets joints to rest location, and sets the intake speed to the off 
    public void setRestPosition(){
        shooterTarget = Constants.JointConstants.shooterStart;
        intakeTarget = Constants.JointConstants.intakeStart;
        m_IntakeSubsystem.stopintake();
        m_Shooter.setShootingSpeed();
        isShootingState = false;
    }

    //Sets joints to amp location, and sets the intake speed to the amp speed when it is enabled
    public void setAMPPosition(){
        shooterTarget = Constants.JointConstants.shooterAMP;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setAmpSpeed();
        isShootingState = true;
    }

    //Turns the shooter on or off
    public void toggleShooter(){
        m_Shooter.toggleShooter();
    }

    public void startShooter(){
        m_Shooter.startShooter();
    }

    public void stopShooter(){
        m_Shooter.stopShooter();
    }

    public boolean getShooterSpunUp(){
        return m_Shooter.getShooterSpunUp();
    }

    public boolean getShootingState(){
        return isShootingState;
    }

    //Turns on the intake to the speed that the state requires, except if you are in a state where you are shooting, then if you try, it does not enable the intake unless the shooter is at speed
    public void enableIntake(){
        if((!isShootingState) || ((isShootingState) && (m_Shooter.getShooterSpunUp()))){
            m_IntakeSubsystem.startIntake();
            if(isPickup){
                m_IntakeSubsystem.setAlreadyStopped(false);
                alreadyLiftedIntake = false;
            }
        }else if(((isShootingState) && (m_Shooter.getShooterSpunUp()))){
            m_IntakeSubsystem.toggleIntake();
        }else{
            disableIntake();
        }
    }

    //Spits out a piece regardless of position, ideal to be done in amp position
    public void quickOuttake(){
        m_IntakeSubsystem.startIntakeReverse();
    }

    //Turns off the intake
    public void disableIntake(){
        m_IntakeSubsystem.stopintake();
    }

    public void periodic() {
        m_IntakeJointSubsystem.setTarget(intakeTarget);
        m_ShooterJointSubsystem.setTarget(shooterTarget);
        SmartDashboard.putNumber("Intake Target", intakeTarget);
        SmartDashboard.putNumber("Shooter Target", shooterTarget);
        SmartDashboard.putBoolean("atTarget", m_IntakeJointSubsystem.atTarget() && m_ShooterJointSubsystem.atTarget());
        //Automatically lifts the intake once tehre is a note inside
        if(m_IntakeSubsystem.noteAcquired() && !alreadyLiftedIntake){
            setSpeakerPosition();
            alreadyLiftedIntake = true;
        }
        m_IntakeSubsystem.periodic();
        m_IntakeJointSubsystem.periodic();
        m_ShooterJointSubsystem.periodic();
        m_Shooter.periodic();
    }
}