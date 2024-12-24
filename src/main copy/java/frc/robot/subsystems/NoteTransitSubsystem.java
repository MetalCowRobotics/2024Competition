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
    private enum positions {rest, pickup, shortshot, longshot, amp};
    private positions curPosition = positions.rest;

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
        m_IntakeSubsystem.setAlreadyStopped(false);
        m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterClose);
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakeDeployed);
        m_IntakeSubsystem.setPickupSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = false;
        curPosition = positions.pickup;
    }

    //Sets joints to speaker location, and sets the intake speed to the feed speed for when it is enabled,
    public void setSpeakerPosition(){
        m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterClose);
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakeLoading);
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
        curPosition = positions.shortshot;
    }

    //Sets joints to speaker location, and sets the intake speed to the feed speed for when it is enabled,
    public void setStageShootingPosition(){
        m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterFar);
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakefarshot);
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
        curPosition = positions.longshot;
    }

    public void setStageShootingPositionmid(){
        shooterTarget = Constants.JointConstants.shooterFar;
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
    }

    //Sets joints to speaker from spike location, and sets the intake speed to the feed speed for when it is enabled,
    public void setSpeakerFromSpikeMark(){
        m_ShooterJointSubsystem.setTarget(SmartDashboard.getNumber("Shooter Far Target", Constants.JointConstants.shooterFar));
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakefarshot);
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
        curPosition = positions.longshot;
    }

     public void setStage(){
        m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterStage);
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakefarshot);
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        isShootingState = true;
        curPosition = positions.longshot;
    }

    public void setSpeakerMidPosition(){
        shooterTarget = SmartDashboard.getNumber("Shooter Mid Target", 
        Constants.JointConstants.shooterMid);
        intakeTarget = Constants.JointConstants.intakeLoading;
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();

        isShootingState = true;
    }
   
    public void setVariableAngle(double xDist){
        if(!(xDist == 0.0)){
            m_ShooterJointSubsystem.setVariableAngle2(xDist);
        }else{
            m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterFar);
        }
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakefarshot);
        m_IntakeSubsystem.setFeedSpeed();
        m_Shooter.setShootingSpeed();
        curPosition = positions.longshot;
    }

    //Sets joints to rest location, and sets the intake speed to the off 
    public void setRestPosition(){
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakeStart);
        m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterStart);
        m_IntakeSubsystem.stopintake();
        m_Shooter.setShootingSpeed();
        isShootingState = false;
        curPosition = positions.rest;
    }

    //Sets joints to amp location, and sets the intake speed to the amp speed when it is enabled
    public void setAMPPosition(){
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakeDeployed);
        m_ShooterJointSubsystem.setTarget(Constants.JointConstants.shooterAMP);
        m_IntakeSubsystem.setPickupSpeed();
        m_Shooter.setAmpSpeed();
        isShootingState = true;
        curPosition = positions.amp;
    }

    public void toggleIntake(){
        m_IntakeSubsystem.setReadyToLift(false);
        if((!isShootingState) || ((isShootingState) && (m_Shooter.getShooterSpunUp()))){
            m_IntakeSubsystem.toggleIntake();
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
        SmartDashboard.putNumber("Intake Target", intakeTarget);
        SmartDashboard.putNumber("Shooter Target", shooterTarget);
        SmartDashboard.putBoolean("atTarget", m_IntakeJointSubsystem.atTarget() && m_ShooterJointSubsystem.atTarget());
        //Automatically lifts the intake once tehre is a note inside
        // if(m_IntakeSubsystem.getReadyToLift() && !alreadyLiftedIntake){
        //     setSpeakerPosition();
        //     alreadyLiftedIntake = true;
        // }
        if (positions.pickup == curPosition && m_IntakeSubsystem.noteAcquired()) {
            setRestPosition();
        }
        m_IntakeSubsystem.periodic();
        m_IntakeJointSubsystem.periodic();
        m_ShooterJointSubsystem.periodic();
        m_Shooter.periodic();
    }
}