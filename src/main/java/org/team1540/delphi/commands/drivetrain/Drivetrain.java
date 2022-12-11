package org.team1540.delphi.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.delphi.utils.swerve.ModuleOffset;
import org.team1540.delphi.utils.swerve.ModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

import static org.team1540.delphi.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static org.team1540.delphi.Constants.DRIVETRAIN_WHEELBASE_METERS;

public class Drivetrain extends SubsystemBase {


    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );


    private final ChickenSwerveModule moduleFrontLeft = new ChickenSwerveModule(4, ModuleOffset.MODULE4, ModulePosition.FRONT_LEFT);
    private final ChickenSwerveModule moduleFrontRight = new ChickenSwerveModule(1, ModuleOffset.MODULE1, ModulePosition.FRONT_RIGHT);
    private final ChickenSwerveModule moduleRearLeft = new ChickenSwerveModule(3, ModuleOffset.MODULE3, ModulePosition.REAR_LEFT);
    private final ChickenSwerveModule moduleRearRight = new ChickenSwerveModule(2, ModuleOffset.MODULE2, ModulePosition.REAR_RIGHT);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveModuleState[] states = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getYaw());

    public Drivetrain() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ChickenSwerveModule.MAX_VELOCITY_METERS_PER_SECOND);
        moduleFrontLeft.set(states[0]);
        moduleFrontRight.set(states[1]);
        moduleRearLeft.set(states[2]);
        moduleRearRight.set(states[3]);
    }

    /**
     * Adjusts all the wheels to achieve the desired movement
     *
     * @param xPercent      The forward and backward movement
     * @param yPercent      The left and right movement
     * @param rot           The amount to turn (in radians)
     * @param fieldRelative If the directions are relative to the field instead of the robot
     */
    public void drive(double xPercent, double yPercent, double rot, boolean fieldRelative) {
        double xSpeed = xPercent * ChickenSwerveModule.MAX_VELOCITY_METERS_PER_SECOND;
        double ySpeed = yPercent * ChickenSwerveModule.MAX_VELOCITY_METERS_PER_SECOND;
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
        setChassisSpeeds(chassisSpeeds);
    }

    private void setModuleStates(SwerveModuleState[] newStates) {
        this.states = newStates;
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        states = kinematics.toSwerveModuleStates(speeds);
    }


    protected Command getPathCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                this::getPose, // Pose supplier
                this.kinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                this // Requires this drive subsystem
        );
    }

    protected Command getResettingPathCommand(PathPlannerTrajectory trajectory) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> resetOdometry(trajectory.getInitialHolonomicPose())),
                getPathCommand(trajectory)
        );
    }


    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        if (gyro.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(gyro.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getYaw());
    }

}
