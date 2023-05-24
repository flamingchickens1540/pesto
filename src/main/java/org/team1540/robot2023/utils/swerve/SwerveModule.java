package org.team1540.robot2023.utils.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.team1540.lib.math.Conversions;
import org.team1540.lib.util.CTREModuleState;
import org.team1540.lib.util.SwerveModuleConstants;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.Robot;

import static org.team1540.robot2023.Constants.Swerve.canbus;

public class SwerveModule {
    public int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final TalonFX mAngleMotor;
    private final TalonFX mDriveMotor;
    private final CANcoder angleEncoder;

    private final VelocityDutyCycle driveVelocityOut = new VelocityDutyCycle(0);
    private final PositionDutyCycle turnOut = new PositionDutyCycle(0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, canbus);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, canbus);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, canbus);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isParkMode){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState, isParkMode);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        mDriveMotor.getConfigurator().refresh(config);
        config.NeutralMode = neutralMode;
        mDriveMotor.getConfigurator().apply(config);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxVelocity;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.setControl(driveVelocityOut.withVelocity(velocity).withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean forceTurn){
        Rotation2d angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxVelocity * 0.01)) && !forceTurn) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        mAngleMotor.setControl(turnOut.withPosition(Conversions.degreesToRotations(angle.getDegrees(), Constants.Swerve.angleGearRatio)));
        lastAngle = angle;
    }


    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue()*Constants.Swerve.angleGearRatio);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToRotations(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder(){
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);

        mDriveMotor.setRotorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                Conversions.falconToMPS(mDriveMotor.getRotorVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.falconToMeters(mDriveMotor.getRotorPosition().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                getAngle()
        );
    }
}