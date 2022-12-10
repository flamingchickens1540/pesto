package org.team1540.delphi.commands.drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
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

import static org.team1540.delphi.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static org.team1540.delphi.Constants.DRIVETRAIN_WHEELBASE_METERS;

public class Drivetrain extends SubsystemBase{


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


    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);

    public Drivetrain() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ChickenSwerveModule.MAX_VELOCITY_METERS_PER_SECOND*2);
        moduleFrontLeft.set(states[0]);
        moduleFrontRight.set(states[1]);
        moduleRearLeft.set(states[2]);
        moduleRearRight.set(states[3]);
    }

    /**
    * Adjusts all the wheels to achieve the desired movement
    * @param xSpeed The forward and backward movement
    * @param ySpeed The left and right movement
    * @param rot The amount to turn (in radians)
    * @param fieldRelative If the directions are relative to the field instead of the robot 
    */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        chassisSpeeds =
            fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyro.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        if (gyro.isMagnetometerCalibrated()) {
          // We will only get valid fused headings if the magnetometer is calibrated
          return Rotation2d.fromDegrees(gyro.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

}
