package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.*;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;
import org.team1540.robot2023.utils.swerve.SwerveModule;

import static org.team1540.robot2023.Constants.Swerve;

public class Drivetrain extends SubsystemBase {


    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Swerve.trackWidth / 2.0, Swerve.wheelBase / 2.0),
            // Front right
            new Translation2d(Swerve.trackWidth / 2.0, -Swerve.wheelBase / 2.0),
            // Back left
            new Translation2d(-Swerve.trackWidth / 2.0, Swerve.wheelBase / 2.0),
            // Back right
            new Translation2d(-Swerve.trackWidth / 2.0, -Swerve.wheelBase / 2.0)
    );

    private final SwerveModule[] modules = new SwerveModule[]{
            new SwerveModule(0, Swerve.Mod0.constants),
            new SwerveModule(1, Swerve.Mod1.constants),
            new SwerveModule(2, Swerve.Mod2.constants),
            new SwerveModule(3, Swerve.Mod3.constants)
    };

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveModuleState[] states = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());

    public Drivetrain() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.maxVelocity);
        modules[0].setDesiredState(states[0], true);
        modules[1].setDesiredState(states[1], true);
        modules[2].setDesiredState(states[2], true);
        modules[3].setDesiredState(states[3], true);
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
        double deadzone = 0.02;
        double rotDeadzone = 0.1;
        if (Math.abs(xPercent) > 0 || Math.abs(yPercent) > deadzone || Math.abs(rot) > rotDeadzone) {
            setChassisSpeeds(chassisSpeeds);
        } else {
            stopLocked();
        }
    }

    public void stopLocked() {
        System.out.println("LOCKING YAY");
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)), //Front Left
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), //Front Right
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), //Back Left
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)) //Back Right
        });
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
        odometry.resetPosition(getYaw(), null, pose);
    }


    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : modules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

}
