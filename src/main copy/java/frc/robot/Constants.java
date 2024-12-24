package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 14;

        public static final COTSTalonFXSwerveConstants chosenModule =  //Tuned
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.50); //Tuned
        public static final double wheelBase = Units.inchesToMeters(21.5); //Tuned
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = 1;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 10;
        public static final int angleCurrentThreshold = 20;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 25;
        public static final int driveCurrentThreshold = 30;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 105;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.09; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //Tuned
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 6; //TODO: This must be tuned to specific robot
        public static final double maxAutoSpeed = 2.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = Math.PI*3; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //Tuned
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final int configNum = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); // 76.49
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, configNum);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //Tuned
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final int configNum = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); // -177.08
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, configNum);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //Tuned
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final int configNum = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); //-5.69
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, configNum);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //Tuned
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final int configNum = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); //-122.58
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, configNum);
        }
    }

    public static final class VisionConstants {
        /* In Meters (Camera 10 Inches In Front of Gyro, Camera 4 Inches To The Gyro's Left, Camera 9 Inches Above Gyro). */
        public static final Transform3d robotToCamTranslation = new Transform3d(0.254, 0.1016, 0.2286, new Rotation3d());

        public static final class RobotCoordsForEachID {
            public static final double[] redSpeakerCenterCoords = {15.15, 5.55, 180};
            public static final double[] blueSpeakerCeneterCoords = {1.35, 5.55, 0};

            public static final double[] redRightSpeakerCoords = {15.77, 6.66, 120};
            public static final double[] blueRightSpeakerCoords = {0.77, 4.44, 300};

            public static final double[] redLeftSpeakerCoords = {15.77, 4.44, 240};
            public static final double[] blueLeftSpeakerCoords = {0.77, 6.66, 60};

            public static final double[] redAmpCoords = {14.70, 7.60, 270};
            public static final double[] blueAmpCoords = {1.84, 7.60, 270};

            public static final double[] redWolfTrapCoords = {12.25, 4.90, 225};
            public static final double[] blueWolfTrapCoords = {4.35, 4.90, 315};

            public static final double[] redSourceTrapCoords = {12.20, 3.25, 135};
            public static final double[] blueSourceTrapCoords = {4.40, 3.25, 45};

            public static final double[] redOuterTrapCoords = {10.75, 4.10, 0};
            public static final double[] blueOuterTrapCoords = {5.85, 4.10, 180};
        }
    }

    public static final class AutoConstants { //Tuned
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    // Wrist and Intake angles for each position in degrees
    public static final class JointConstants{
        public static final double intakeStart = 0;
        public static final double intakeDeployed = 208;
        public static final double intakefarshot = 15;
        public static final double intakeLoading = 12;
        public static final double shooterStart = 0;
        public static final double shooterClose = -32;
        public static final double shooterFar = -52;
        public static final double shooterStage = -49;
        public static final double shooterAMP = -28;
        public static final double shooterMid = -44;

        public static final double[][] variableShootingConstants = {{0.0, -3.5, -6.9, -10.3, -13.62, -16.85, -19.97, -22.98, -25.85, -33.6},
                                                                    {-38.2, -33.67, -36.01, -38.22, -43.3, -42.26, -44.1, -45.84, -46.47, -49},
                                                                    {-50.46, -51.83, -53.1, -54.33, -55.48, -56.56, -57.6, -58.55, -59.47, -60.35},
                                                                    {-61.2, -61.96, -62.7, -63.4, -64.1, -64.75, -65.36, -65.95, -66.5, -67.06},
                                                                    {-67.6, -68.07, -68.54, -69, -69.43, -69.85, -70.26, -70.64, -71, -71.4},
                                                                    {-71.7, -72.06, -72.4, -72.7, -73, -73.3, -73.6, -73.85, -74.1, -74.37}};
        
    }
    public static double targetPositionTolerance = 0.1;
    public static double targetAngleTolerance = 3;
}