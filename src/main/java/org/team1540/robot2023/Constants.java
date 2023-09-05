package org.team1540.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.team1540.lib.math.Conversions;
import org.team1540.lib.util.COTSFalconSwerveConstants;
import org.team1540.lib.util.SwerveModuleConstants;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.GridScoreData;
import org.team1540.robot2023.utils.swerve.ModuleCorner;

public final class Constants {
    public static final boolean ENABLE_PNEUMATICS = false;
    public static final int PNEUMATIC_HUB = 2;
    public static final int PDH = 1;
    private static final boolean isNewRobot = true;
    
    // 22 in

//    public static final double poleOffsetMeters = 0.5;

    public static final class Auto {
        public static final double gridPoleOffsetMeters = Units.inchesToMeters(22);
        public static final double gridBackoffOffsetMeters = Units.inchesToMeters(32);
        public static final double hpOffsetX = Units.inchesToMeters(35);
        public static final double hpOffsetY = -Units.inchesToMeters(29); 

        public static final double autoDriveDistance = 1; //in meters


        public static final ArmState armUp = ArmState.fromRotationExtension(Rotation2d.fromDegrees(0),0);


        public static final ArmState armPrescore = ArmState.fromRotationExtension(Rotation2d.fromDegrees(-65),0);

        public static final ArmState armDown = ArmState.fromRotationExtension(Rotation2d.fromDegrees(-120),0);
        public static final ArmState armDownBackwards = ArmState.fromRotationExtension(Rotation2d.fromDegrees(121.5),37.8);
        public static final ArmState reverseCube = ArmState.fromRotationExtension(Rotation2d.fromDegrees(120),0);
        public static final ArmState armHumanPlayer = ArmState.fromRotationExtension(Rotation2d.fromDegrees(-60),55);
        public static final ArmState armHumanPlayerRetreat = ArmState.fromRotationExtension(Rotation2d.fromDegrees(-55),55);


        public static final GridScoreData midCube = new GridScoreData(ArmState.fromRotationExtension(Rotation2d.fromDegrees(-65), 35.18));
        public static final GridScoreData highCube = new GridScoreData(ArmState.fromRotationExtension(Rotation2d.fromDegrees(-55),59));
        public static final GridScoreData reverseHighCube = new GridScoreData(ArmState.fromRotationExtension(Rotation2d.fromDegrees(55),59));

        public static final GridScoreData midCone = new GridScoreData(
                ArmState.fromRotationExtension(Rotation2d.fromDegrees(-50),47.0), // Mid Cone Approach (46.0 originally)
                ArmState.fromRotationExtension(Rotation2d.fromDegrees(-70.7),47.0) // Mid Cone Score (46.0 originally)
        );
        public static final GridScoreData highCone = new GridScoreData(
                ArmState.fromRotationExtension(Rotation2d.fromDegrees(-53),67), // High Cone Approach
                ArmState.fromRotationExtension(Rotation2d.fromDegrees(-65.5),67), // High Cone Score
                ArmState.fromRotationExtension(Rotation2d.fromDegrees(-58),67) // High Cone Retreat
        );

        public static final GridScoreData hybridNode = new GridScoreData(ArmState.fromRotationExtension(Rotation2d.fromDegrees(-15), 0)).withAdditionalBackoff(Units.inchesToMeters(0));
        public static final GridScoreData middleHybridNode = new GridScoreData(ArmState.fromRotationExtension(Rotation2d.fromDegrees(-30),0)).withAdditionalBackoff(Units.inchesToMeters(0));



        public static final class PID {
            public static final double translationP = 5;
            public static final double translationI = 0;
            public static final double translationD = 0;
            public static final double rotationP = 5;
            public static final double rotationI = 0;
            public static final double rotationD = 0;
        }
    }
    public static final class Swerve {
        public static final String canbus = isNewRobot ? "swerve" : ""; // Set to "" to use RIO's can bus

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26);
        public static final double wheelBase = Units.inchesToMeters(23);
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
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.1289; //TODO: This must be tuned to specific robot
//        public static final double driveKP = 0.6;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.1942 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.3193 / 12);
        public static final double driveKA = (0.33558 / 12);

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        public static final double maxVelocity = 6380.0 / 60.0 *
                (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
                0.10033 * Math.PI;
        public static final double maxAngularSpeed = maxVelocity /
                Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;


        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            private static final int moduleID = 4;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleID, ModuleCorner.FRONT_LEFT);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            private static final int moduleID = 6 ;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleID, ModuleCorner.FRONT_RIGHT);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            private static final int moduleID = 8;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleID, ModuleCorner.REAR_LEFT);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            private static final int moduleID = 2;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleID, ModuleCorner.REAR_RIGHT);
        }
    }
    public static final class Vision {
        public static final double kP = -0.04;
        public static final double kI = 0;
        public static final double kD = -0.002;
    }

    public static final class ArmConstants {
        public static final int PIVOT1_ID = 11;
        public static final int PIVOT2_ID = 10;
        public static final int PIVOT_ENCODER_CHANNEL_A = 8;
        public static final int PIVOT_ENCODER_CHANNEL_B = 9;
        public static final int PIVOT_ENCODER_PULSES_PER_REV = 2048;
        public static final int TELESCOPE_ID = 12;
        public static final int PIGEON_ID = 13;

        public static final double PIVOT_FF = 0;
        public static final double PIVOT_KP = 0.2;
        public static final double PIVOT_KI = 0;
        public static final double PIVOT_KD = 1;
        public static final double PIVOT_MAX_ACCEL = 40_000;
        public static final double PIVOT_CRUISE_SPEED = 20_000;

        // The distance of the pivot from the ground
        public static final double PIVOT_HEIGHT = 21.5;
        // The distance of the pivot from the edge of the frame perimeter
        public static final double PIVOT_DISTANCE = 14;
        // Minimum pivot angle before arm collides with robot (radians) (should be negative)
        public static final double PIVOT_MIN_ANGLE = -2.260;
        public static final int PIVOT_FORWARD_LIMIT = 139_000;
        public static final int PIVOT_REVERSE_LIMIT = -134_500;
        // Pivot motor revolutions to pivot revolutions
        public static final double PIVOT_GEAR_RATIO = 196.875;
        // The base arm length
        public static final double ARM_BASE_LENGTH = 32.325;
        // The extended arm length
        public static final double ARM_LENGTH_EXT = 88; // TODO: 1/29/2023 figure this out
        public static final double EXT_ROTS_TO_INCHES = 2 * 1.504 * Math.PI; // TODO: 2/11/2023 figure this out so that position things work
        public static final double EXT_GEAR_RATIO = 16;

        public static final double TELESCOPE_KF = 0.0002;
        public static final double TELESCOPE_KP = 0.0001;
        public static final double TELESCOPE_KI = 0;
        public static final double TELESCOPE_KD = 0;
        public static final double TELESCOPE_CRUISE_SPEED = 5000;

        public static final double TELESCOPE_MAX_ACCEL = TELESCOPE_CRUISE_SPEED/(0.25);


        public static final float TELESCOPE_FORWARD_LIMIT = (float) ((67 - ARM_BASE_LENGTH) * EXT_GEAR_RATIO / EXT_ROTS_TO_INCHES);

        // The max height from the floor
        public static final double MAX_LEGAL_HEIGHT = 78 - PIVOT_HEIGHT;
        // The max distance extended from the frame perimeter
        public static final double MAX_LEGAL_DISTANCE = 48 + PIVOT_DISTANCE;
        // The maximum distance the arm will point to
        public static final double MAX_POINT_DISTANCE = MAX_LEGAL_DISTANCE + 100; // TODO: 1/30/2023 this should be the maximum limelight detection distance

        public static final double PIGEON_MNT_YAW = 0; // TODO: 2/15/2023 figure this out (depends on how the pigeon is mounted)
        public static final double PIGEON_MNT_PITCH = 90;
        public static final double PIGEON_MNT_ROLL = 0;
        public static final double PIGEON_OFFSET = 4.04;
    }

    public static final class WristConstants {
        public static final int CANCODER_ID = 18;
        public static final double CANCODER_OFFSET = 0; // TODO: 8/29/2023 figure out cancoder offset
        public static final double CANCODER_GEARING = 1; // TODO: 8/30/2023 find this gear ratio

        public static final int WRIST_ID = 17;
        public static final int WRIST_GEARING = 1; // TODO: 8/30/2023 find this gear ratio
        public static final double WRIST_FORWARD_LIMIT = Conversions.degreesToFalcon(80, WRIST_GEARING); // TODO: 8/29/2023 figure out wrist soft limits
        public static final double WRIST_REVERSE_LIMIT = Conversions.degreesToFalcon(-80, WRIST_GEARING);
        public static final double WRIST_KP = 1;
        public static final double WRIST_KI = 0;
        public static final double WRIST_KD = 0;
        public static final int WRIST_CURRENT = 30; // TODO: 8/29/2023 figure out wrist current limit
    }

    public static final class GrabberConstants {
        public static final int INTAKE_1_ID = 14;
        public static final int INTAKE_2_ID = 15;

        public static final double INTAKE_CURRENT_THRESH = 40;
    }

    public static final class RollerIntakeConstants {
        public static final int ROLLER_INTAKE_ID = 16;
        public static final int INTAKE_IDLE_CURRENT = 10; // TODO: 8/29/2023 find the intake idle current limit
        public static final int INTAKE_AGGRO_CURRENT = 30; // TODO: 8/29/2023 find the intake aggravating current limit
    }
}