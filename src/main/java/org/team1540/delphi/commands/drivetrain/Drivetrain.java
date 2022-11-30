package org.team1540.delphi.commands.drivetrain;
import org.team1540.delphi.utils.swerve.ModuleOffset;
import org.team1540.delphi.utils.swerve.ModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    // TODO: check measurements/ make more accurate
    private final Translation2d offsetFrontLeftLocation = new Translation2d(0.4826, 0.4826);
    private final Translation2d offsetFrontRightLocation = new Translation2d(0.4826, -0.4826);
    private final Translation2d offsetRearLeftLocation = new Translation2d(-0.4318, 0.4318);
    private final Translation2d offsetRearRightLocation = new Translation2d(-0.4318, -0.4318); 
    
    private final SwerveModule moduleFrontLeft = new SwerveModule(1, ModuleOffset.MODULE1, ModulePosition.FRONT_LEFT);
    private final SwerveModule moduleFrontRight = new SwerveModule(2, ModuleOffset.MODULE2, ModulePosition.FRONT_RIGHT);
    private final SwerveModule moduleRearLeft = new SwerveModule(3, ModuleOffset.MODULE3, ModulePosition.REAR_LEFT);
    private final SwerveModule moduleRearRight = new SwerveModule(4, ModuleOffset.MODULE4, ModulePosition.REAR_RIGHT);
    
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        offsetFrontLeftLocation, 
        offsetFrontRightLocation, 
        offsetRearLeftLocation, 
        offsetRearRightLocation
    );
    
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    

    public Drivetrain() {
        gyro.reset();
    }
    
    /**
    * Adjusts all of the wheels to achieve the desired movement
    * @param xSpeed The forward and backward movement
    * @param ySpeed The left and right movement
    * @param rot The amount to turn (in radians)
    * @param fieldRelative If the directions are relative to the field instead of the robot 
    */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        moduleFrontLeft.setDesiredState(swerveModuleStates[0]);
        moduleFrontRight.setDesiredState(swerveModuleStates[1]);
        moduleRearLeft.setDesiredState(swerveModuleStates[2]);
        moduleRearRight.setDesiredState(swerveModuleStates[3]);
    }
    
    /**
     * Updates the SwerveDriveOdometry with the current robot state
     */
    public void updateOdometry() {
        m_odometry.update(
            gyro.getRotation2d(),
            moduleFrontLeft.getPosition(),
            moduleFrontRight.getPosition(),
            moduleRearLeft.getPosition(),
            moduleRearRight.getPosition()
        );
    }
}
