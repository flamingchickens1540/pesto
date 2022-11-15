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
    private final Translation2d offsetFrontLeftLocation = new Translation2d(0.381, 0.381); //meters from center of robot CHANGEEE
    private final Translation2d offsetFrontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d offsetRearLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d offsetRearRightLocation = new Translation2d(-0.381, -0.381); 
    
    private final SwerveModule moduleFrontLeft = new SwerveModule(1, ModuleOffset.MODULE1, ModulePosition.FRONT_LEFT);
    private final SwerveModule moduleFrontRight = new SwerveModule(2, ModuleOffset.MODULE2, ModulePosition.FRONT_RIGHT);
    private final SwerveModule moduleRearLeft = new SwerveModule(3, ModuleOffset.MODULE3, ModulePosition.REAR_LEFT);
    private final SwerveModule moduleRearRight = new SwerveModule(4, ModuleOffset.MODULE4, ModulePosition.REAR_RIGHT);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);//change to navx 

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
    // Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        gyro.getRotation2d(),
        moduleFrontLeft.getPosition(),
        moduleFrontRight.getPosition(),
        moduleRearLeft.getPosition(),
        moduleRearRight.getPosition());
  }
}
  
/////////
    /*private final TalonFX leftMotorFront = new TalonFX(Constants.Motors.leftFront);
    private final TalonFX leftMotorRear = new TalonFX(Constants.Motors.leftRear);
    private final TalonFX rightMotorFront = new TalonFX(Constants.Motors.rightFront);
    private final TalonFX rightMotorRear = new TalonFX(Constants.Motors.rightRear);
    
    private final TalonFX leftMotorFrontSwerve = new TalonFX(Constants.Motors.leftFrontSwerve);
    private final TalonFX leftMotorRearSwerve = new TalonFX(Constants.Motors.leftRearSwerve);
    private final TalonFX rightMotorFrontSwerve = new TalonFX(Constants.Motors.rightFrontSwerve);
    private final TalonFX rightMotorRearSwerve = new TalonFX(Constants.Motors.rightRearSwerve);

    private final TalonFX driveMotors[] = { leftMotorFront, leftMotorRear, rightMotorFront, rightMotorRear };
    private final TalonFX leftDriveMotors[] = { leftMotorFront, leftMotorRear, leftMotorFrontSwerve, leftMotorRearSwerve};
    private final TalonFX rightDriveMotors[] = { rightMotorFront, rightMotorRear, rightMotorFrontSwerve, rightMotorRearSwerve};

    public Drivetrain() {
        initMotors(NeutralMode.Coast);
    }
    public Drivetrain(NeutralMode brakeType) {
        initMotors(brakeType); 
    }

    private void initMotors(NeutralMode brakeType){
        for (TalonFX motor : driveMotors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(brakeType);
        }
        // Set configuration for left motors
        for (TalonFX motor : leftDriveMotors) {
            motor.setInverted(false);
        }
        // Set configuration for right motors
        for (TalonFX motor : rightDriveMotors) {
            motor.setInverted(true);
        }
        // Make rear motors follow front motors
        leftMotorRear.follow(leftMotorFront);
        rightMotorRear.follow(rightMotorFront);
        leftMotorRearSwerve.follow(leftMotorFrontSwerve); 
        rightMotorRearSwerve.follow(rightMotorFrontSwerve); 

    }

    public void setPercent(double leftPercent, double rightPercent, double leftSwervePercent, double rightSwervePercent) {
        leftMotorFront.set(ControlMode.PercentOutput, leftPercent);
        rightMotorFront.set(ControlMode.PercentOutput, rightPercent);
        leftMotorFrontSwerve.set(ControlMode.PercentOutput, leftSwervePercent);
        rightMotorFrontSwerve.set(ControlMode.PercentOutput, rightSwervePercent);
    }
}
*/
