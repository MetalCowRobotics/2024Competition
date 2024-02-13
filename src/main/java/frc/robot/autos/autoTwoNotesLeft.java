package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class autoTwoNotesLeft {
  /* Subsystems */
    private Swerve m_swerve = new Swerve();
    private ShoulderSubsystem m_shoulderSubsystem;
    private ElbowSubsystem m_elbowSubsystem;
    private WristSubsystem m_wristSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private LEDSubsystem m_LEDSubsystem;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
   private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton moveToCenter = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton moveToLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton moveToRight = new JoystickButton(driver, XboxController.Button.kB.value);
    
    /* Operator Buttons */
    private final JoystickButton cubeSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton coneSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton cubeFloorIntakePosition = new JoystickButton(operator, XboxController.Axis.kLeftY);
    // private final JoystickButton coneFloorIntakePosition = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton lowScoringPosition = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton midScoringPosition = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton stow = new JoystickButton(operator, XboxController.Button.kA.value);

    Trigger crawl = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    Trigger sprint = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);

    Trigger cubeFloorIntakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftY.value) > 0.8);
    Trigger coneFloorIntakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightY.value) > 0.8);

    Trigger intakeForward = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.7);
    Trigger intakeReverse = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kBack.value));
    Trigger stopIntake = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kX.value));
    Trigger eject = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.7);

    Trigger substationRight = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kRightBumper.value));
    Trigger substationLeft = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kLeftBumper.value));

    Trigger wristUp = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftX.value) > 0.7);
    Trigger wristDown = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightX.value) > 0.7);

    Trigger shootHigh = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kStart.value));
    // Trigger toggleLED = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kY.value));
    // private final JoystickButton stopstow = new JoystickButton(operator, XboxController.Button.kB.value);

    Trigger balance = new Trigger(() -> driver.getRawButton(XboxController.Button.kStart.value));

    /* Subsystems */
    // private Swerve m_swerve = new Swerve();
    // private ShoulderSubsystem m_shoulderSubsystem;
    // private ElbowSubsystem m_elbowSubsystem;
    // private WristSubsystem m_wristSubsystem;
    // private IntakeSubsystem m_IntakeSubsystem;
    // private LEDSubsystem m_LEDSubsystem;

    Trigger drive = new Trigger(() -> 
        (Math.abs(driver.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.1 || Math.abs(driver.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.1) || 
        (Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1 || Math.abs(driver.getRawAxis(XboxController.Axis.kRightY.value)) > 0.1)
    );
    Trigger stopIntakeOnPickup = new Trigger(() -> m_IntakeSubsystem.coneInIntake() || m_IntakeSubsystem.cubeInIntake());
    // private final JoystickButton stopstow = new JoystickButton(operator, XboxController.Button.kB.value);
    
    /* Autos */
    private double armMovementTimeout = 3;
    private SendableChooser<Command> m_autoSelector;               
    
    private Command chargeStationScoreMobilityDock;
    private Command chargeStationScoreDock;
    private Command substationScoreMobilityDockBlue;
    private Command substationScoreMobilityDockRed;
    private Command substationScoreMobility;
    private Command cableRunScoreMobility;
    private Command armTest;
    private Command twoPieceAutoBlueMidCubeLowCube;
    private Command twoPieceAutoRedMidCubeLowCube;
    private Command twoPieceTwoCube;
    private Command twoPieceHighCubeHighCone;

    private Command twoPieceAutoBlueHighCubeLowCube;
    private Command twoPieceAutoRedHighCubeLowCube;


    private Command twoPieceAutoBlueMidConeLowCube;
    private Command twoPieceAutoBlueMidCubeMidCone;
    private Command twoPieceAutoBluePoofsTest;

    private Command threePiece;
    
    private Command alignToMiddle;
    private Command alignToLeft;
    private Command alignToRight;
    private Command alignToSubstationRight;
    private Command changeColor;
    private Command balanceCommand;
    private Command alignToSubstationLeft;

    private Command noAuto = new InstantCommand(() -> m_swerve.zeroGyro(180));

    /*change the color
    Trigger changeLightColor = new Trigger(() -> m_IntakeSubsystem.Identification());*/

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        m_swerve = new Swerve();
        m_LEDSubsystem = new LEDSubsystem(1);
        m_autoSelector = new SendableChooser<Command>();

        intakeSubsystem = new intakeSubsystem();
        armSubsystem = new armSubsystem();
        wristSubsystem = new wristSubsystem();
        
        Rotation2d leftSpeakerHeading = new Rotation2d(135);

        autoTwoNotesLeft = new SequentialCommandGroup( 
            new InstantCommand(() -> m_swerve.setHeading(leftSpeakerHeading)), // face the speaker from left
            new InstantCommand(() -> m_swerve.resetModulesToAbsolute()),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new InstantCommand(() -> m_swerve.zeroGyro()),
            new ParallelRaceGroup(
                new InstantCommand(() -> m_swerve.driveAuto()),
                new InstantCommand(() -> m_IntakeSubsystem.run())
            ),
            new DriveToPoint(m_swerve, -4, 0, 180),
            new EnableVision(m_swerve)
        );   
}
