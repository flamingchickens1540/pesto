package org.team1540.delphi.commands.drivetrain;
import org.team1540.delphi.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private final TalonFX leftMotorFront = new TalonFX(Constants.Motors.leftFront);
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
