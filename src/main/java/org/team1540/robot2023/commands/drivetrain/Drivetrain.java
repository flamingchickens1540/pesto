package org.team1540.robot2023.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.LimelightManager;
import org.team1540.robot2023.utils.swerve.SwerveModule;

import static org.team1540.robot2023.Constants.Swerve;
import static org.team1540.robot2023.Globals.field2d;

public class Drivetrain extends SubsystemBase {

    private SwerveModuleState[] states = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private final SwerveModule[] modules = new SwerveModule[]{
            new SwerveModule(0, Swerve.Mod0.constants),
            new SwerveModule(1, Swerve.Mod1.constants),
            new SwerveModule(2, Swerve.Mod2.constants),
            new SwerveModule(3, Swerve.Mod3.constants)
    };

    private final AHRS gyro;
    private double fieldOrientationOffset = 0;
    // These PID controllers don't actually do anything, but their PID values are copied for PathPlanner commands
    private final PIDController dummyTranslationPID = new PIDController(Constants.Auto.PID.translationP,Constants.Auto.PID.translationI,Constants.Auto.PID.translationD);
    private final PIDController dummyRotationPID = new PIDController(Constants.Auto.PID.rotationP,Constants.Auto.PID.rotationI,Constants.Auto.PID.rotationD);
    // Whether to allow the wheels to park
    private boolean isParkMode = false;
    private boolean isRunningPath = false;
    private boolean isRunningAuto = false;

    // Odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    public Drivetrain(AHRS gyro) {
        this.gyro = gyro;
        poseEstimator = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        PPSwerveControllerCommand.setLoggingCallbacks(
                (trajectory) -> field2d.getObject("activetrajectory").setTrajectory(trajectory),
                (pose) -> field2d.getObject("targetpose").setPose(pose),
                null,
                (translation, rotation) -> {
                    SmartDashboard.putNumber("drivetrain/pathplanner/xErr", translation.getX());
                    SmartDashboard.putNumber("drivetrain/pathplanner/yErr", translation.getY());
                    SmartDashboard.putNumber("drivetrain/pathplanner/rotErr", rotation.getDegrees());
                });
        SmartDashboard.putData("drivetrain/translationPID", dummyTranslationPID);
        SmartDashboard.putData("drivetrain/rotationPID", dummyRotationPID);
        SmartDashboard.putNumberArray("drivetrain/swerveModuleStates/desired", new double[]{
                states[0].angle.getDegrees(), states[0].speedMetersPerSecond,
                states[1].angle.getDegrees(), states[1].speedMetersPerSecond,
                states[2].angle.getDegrees(), states[2].speedMetersPerSecond,
                states[3].angle.getDegrees(), states[3].speedMetersPerSecond
        });
        SmartDashboard.putNumberArray("drivetrain/swerveModuleStates/actual", new double[]{
                modules[0].getState().angle.getDegrees(), modules[0].getState().speedMetersPerSecond,
                modules[1].getState().angle.getDegrees(), modules[1].getState().speedMetersPerSecond,
                modules[2].getState().angle.getDegrees(), modules[2].getState().speedMetersPerSecond,
                modules[3].getState().angle.getDegrees(), modules[3].getState().speedMetersPerSecond
        });

        gyro.reset();
    }

    public void stopTags() {
        this.isRunningAuto = true;
    }

    public void startTags() {
        this.isRunningAuto = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro/yaw", gyro.getYaw());
        SmartDashboard.putNumber("gyro/pitch", gyro.getPitch());
        SmartDashboard.putNumber("gyro/roll", gyro.getRoll());
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.maxVelocity);
        modules[0].setDesiredState(states[0], true, isParkMode);
        modules[1].setDesiredState(states[1], true, isParkMode);
        modules[2].setDesiredState(states[2], true, isParkMode);
        modules[3].setDesiredState(states[3], true, isParkMode);
        poseEstimator.update(getYaw(), getModulePositions());
//        if (!isRunningPath && !isRunningAuto) {
//            LimelightManager.getInstance().applyEstimates(poseEstimator);
//        }
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public boolean updateWithApriltags() {
        return LimelightManager.getInstance().zeroFromLimelights(poseEstimator, getYaw(), getModulePositions());
    }
    public boolean updateWithScoringApriltags() {
        return LimelightManager.getInstance().applyFrontEstimates(poseEstimator, getYaw(), getModulePositions());
    }


    public void resetAllToAbsolute() {
        DataLogManager.log("Zeroing swerve module relative encoders");
        for (SwerveModule module: modules) {
            module.resetToAbsolute();
        }
    }

    /**
     * Adjusts all the wheels to achieve the desired movement
     *
     * @param xPercent      The forward and backward movement
     * @param yPercent      The left and right movement
     * @param rotPercent           The amount to turn
     * @param fieldRelative If the directions are relative to the field instead of the robot
     */
    public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {

        double xSpeed = xPercent * Swerve.maxVelocity;
        double ySpeed = yPercent * Swerve.maxVelocity;
        double rot = Math.toRadians(rotPercent*360);
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().minus(Rotation2d.fromDegrees(fieldOrientationOffset)))
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
        double deadzone = 0.02;
        double rotDeadzone = 0.1;
        if (Math.abs(xPercent) > 0 || Math.abs(yPercent) > deadzone || Math.abs(rot) > rotDeadzone) {
            isParkMode = false;
            setChassisSpeeds(chassisSpeeds);
        } else {
            stopLocked();
        }
    }

    /**
     * Stops the robot and forms an X with the wheels
     */
    public void stopLocked() {
        isParkMode = true;
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)), //Front Left
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), //Front Right
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), //Back Left
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)) //Back Right
        });
    }

    void setModuleStates(SwerveModuleState[] newStates) {
        this.states = newStates;
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        states = Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    }


    public Command getAutoPathCommand(PathPlannerTrajectory trajectory) {
        return this.getPathCommand(trajectory, dummyTranslationPID, dummyRotationPID);
    }

    public Command getPathCommand(PathPlannerTrajectory trajectory, PIDController dummyTranslation, PIDController dummmyRotation) {
        return Commands.sequence(
                new InstantCommand(() -> isRunningPath = true).withName("StartBlockingTags"),
                new PPSwerveControllerCommand(
                trajectory,
                this::getPose, // Pose supplier
                // TODO: Tune
                new PIDController(dummyTranslation.getP(), dummyTranslation.getI(), dummyTranslation.getD()),
                new PIDController(dummyTranslation.getP(), dummyTranslation.getI(), dummyTranslation.getD()),
                new PIDController(dummmyRotation.getP(), dummmyRotation.getI(), dummmyRotation.getD()),
                this::setChassisSpeeds, // Module states consumer
                this // Requires this drive subsystem
            ),
            new InstantCommand(() -> isRunningPath = false).withName("StopBlockingTags")
        );
    }

    protected Command getResettingPathCommand(PathPlannerTrajectory trajectory) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> resetOdometry(trajectory.getInitialHolonomicPose())).withName("ResetOdometry"),
                getAutoPathCommand(trajectory)
        );
    }


    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyro.zeroYaw();

    }

//    public void zeroFieldOrientation() {
//        fieldOrientationOffset = getYaw().getDegrees();
//    }
    public void zeroFieldOrientation() {
        fieldOrientationOffset = getYaw().getDegrees()-poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public void zeroFieldOrientationManual() {
        fieldOrientationOffset = getYaw().getDegrees();
    }

    public Rotation2d getYaw() {
        if (gyro.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(-gyro.getFusedHeading());
        }
        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees( gyro.getYaw());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        for (SwerveModule module : modules) {
            module.setNeutralMode(neutralMode);
        }
    }
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }



    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()
        };
    }

}
