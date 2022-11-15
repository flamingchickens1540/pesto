package org.team1540.delphi.commands.drivetrain;
import org.team1540.delphi.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); //meters from center of robot CHANGEEE
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381); 
    
    private final SwerveModule m_frontLeft = new SwerveModule(1);
    private final SwerveModule m_frontRight = new SwerveModule(2);
    private final SwerveModule m_backLeft = new SwerveModule(3);
    private final SwerveModule m_backRight = new SwerveModule(4);

    private final AnalogGyro m_gyro = new AnalogGyro(0);//change to navx 

    private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    public Drivetrain() {
        m_gyro.reset();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
  }
    // Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition());
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
