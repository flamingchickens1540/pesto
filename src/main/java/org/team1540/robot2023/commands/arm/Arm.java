package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.lib.math.Conversions;
import org.team1540.robot2023.Constants.ArmConstants;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.ChickEncoder;

public class Arm extends SubsystemBase {
    private final TalonFX pivot1 = new TalonFX(ArmConstants.PIVOT1_ID);
    private final TalonFX pivot2 = new TalonFX(ArmConstants.PIVOT2_ID);
    private final ChickEncoder pivotEncoder = new ChickEncoder(
            ArmConstants.PIVOT_ENCODER_CHANNEL_A,
            ArmConstants.PIVOT_ENCODER_CHANNEL_B,
            ArmConstants.PIVOT_ENCODER_PULSES_PER_REV,
            true
    );
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(7);

    private final CANSparkMax telescope = new CANSparkMax(ArmConstants.TELESCOPE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder telescopeEncoder = telescope.getEncoder();
    private final SparkMaxPIDController telescopePID = telescope.getPIDController();
    private final SparkMaxLimitSwitch telescopeLimitSwitch = telescope.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    private final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(ArmConstants.PIGEON_ID);

    private double pivotAccel;

    public Arm() {
//        telescope.restoreFactoryDefaults();

        pivot1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0));
        pivot2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0));
        telescope.setSmartCurrentLimit(40);

        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot2.setNeutralMode(NeutralMode.Brake);
        telescope.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivot1.setInverted(false);
        pivot1.configForwardSoftLimitThreshold(ArmConstants.PIVOT_FORWARD_LIMIT);
        pivot1.configForwardSoftLimitEnable(true);
        pivot1.configReverseSoftLimitThreshold(ArmConstants.PIVOT_REVERSE_LIMIT);
        pivot1.configReverseSoftLimitEnable(true);
        pivot2.setInverted(true);
        pivot2.follow(pivot1);

        telescope.setInverted(true);
        telescope.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.TELESCOPE_FORWARD_LIMIT);
        telescope.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        pivot1.config_kP(0, ArmConstants.PIVOT_KP);
        pivot1.config_kI(0, ArmConstants.PIVOT_KI);
        pivot1.config_kD(0, ArmConstants.PIVOT_KD);
        pivot1.configMotionCruiseVelocity(ArmConstants.PIVOT_CRUISE_SPEED);
        pivot1.configMotionAcceleration(ArmConstants.PIVOT_MAX_ACCEL);

        telescopePID.setP(ArmConstants.TELESCOPE_KP);
        telescopePID.setI(ArmConstants.TELESCOPE_KI);
        telescopePID.setD(ArmConstants.TELESCOPE_KD);
        telescopePID.setFF(ArmConstants.TELESCOPE_KF);
        telescopePID.setSmartMotionMaxAccel(ArmConstants.TELESCOPE_MAX_ACCEL, 0);
        telescopePID.setSmartMotionMaxVelocity(ArmConstants.TELESCOPE_CRUISE_SPEED, 0);
        telescopePID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);

        pigeon2.configMountPose(ArmConstants.PIGEON_MNT_YAW, ArmConstants.PIGEON_MNT_PITCH, ArmConstants.PIGEON_MNT_ROLL);

        smashDartboardInit();
    }

    public double getMaxExtension() {
        return getMaxExtension(getRotation2d());
    }

    public double timeToRotation(Rotation2d rotation2d){
        double setpoint = Conversions.degreesToFalcon(rotation2d.getDegrees(), ArmConstants.PIVOT_GEAR_RATIO);
        double distance = Math.abs(setpoint - pivot1.getSelectedSensorPosition());
        double timeToAccelerate = ArmConstants.PIVOT_CRUISE_SPEED/(pivotAccel);
        boolean isAProfile = distance <=
                timeToAccelerate * ArmConstants.PIVOT_CRUISE_SPEED*10;
        if(!isAProfile){
            double value = (distance - (timeToAccelerate * ArmConstants.PIVOT_CRUISE_SPEED * 10))
                    / (ArmConstants.PIVOT_CRUISE_SPEED * 10) + (2 * timeToAccelerate);
            SmartDashboard.putNumber("arm/timeToRotation", 1000* value);
            return 1000* value;
        }
        else{
            SmartDashboard.putNumber("arm/timeToRotation", 1000*(2*Math.sqrt(distance/(pivotAccel*10))));
            return 1000*(2*Math.sqrt(distance/(pivotAccel*10)));
        }
    }

    public double timeToExtension(double extension){
        double setpoint = (extension - ArmConstants.ARM_BASE_LENGTH) * ArmConstants.EXT_GEAR_RATIO / ArmConstants.EXT_ROTS_TO_INCHES;
        double distance = Math.abs(setpoint - telescopeEncoder.getPosition());
        double timeToAccelerate = (ArmConstants.TELESCOPE_CRUISE_SPEED/60)/((ArmConstants.TELESCOPE_MAX_ACCEL/60));
        boolean isAProfile = distance <=
                timeToAccelerate * (ArmConstants.TELESCOPE_CRUISE_SPEED/60);
        if(!isAProfile){
            double value = 1000 * ((distance - timeToAccelerate * (ArmConstants.TELESCOPE_CRUISE_SPEED / 60))
                    / (ArmConstants.TELESCOPE_CRUISE_SPEED / 60) + 2 * timeToAccelerate);
            SmartDashboard.putNumber("arm/timeToExtension", value);
            return value;
        }
        else{
            double value = 1000 * 2 * Math.sqrt(distance / (ArmConstants.TELESCOPE_MAX_ACCEL / 60));
            SmartDashboard.putNumber("arm/timeToExtension", value);
            return value;
        }
    }

    public double timeToState(ArmState state){
        return Math.max(timeToRotation(state.getRotation2d()) , (timeToExtension(state.getExtension())));
    }

    public double getMaxExtension(Rotation2d rotation) {
//        double angle = Conversions.actualToCartesian(rotation).getRadians();
//        if (angle == Math.PI / 2) return ArmConstants.MAX_LEGAL_HEIGHT;
//        if (angle == 0 || angle == Math.PI) return ArmConstants.MAX_LEGAL_DISTANCE;
//        else if (angle > 0 && angle < Math.PI) {
//            return Math.min(Math.abs(ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(angle)), Math.abs(ArmConstants.MAX_LEGAL_HEIGHT / Math.sin(angle)));
//        }
//        else {
//            return Math.min(Math.abs(ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(angle)), Math.abs(ArmConstants.PIVOT_HEIGHT / Math.sin(angle)));
//        }
        double angle = Conversions.actualToCartesian(rotation).getDegrees();
        double vMax, hMax;
        if (angle == 0 || angle == 180) return ArmConstants.MAX_LEGAL_DISTANCE;
        if (angle == 90) return ArmConstants.MAX_LEGAL_HEIGHT;

        if (angle > 180 || angle < 0) vMax = ArmConstants.PIVOT_HEIGHT / Math.sin(Math.toRadians(angle));
        else vMax = ArmConstants.MAX_LEGAL_HEIGHT / Math.sin(Math.toRadians(angle));
        vMax = Math.abs(vMax);

        hMax = Math.abs(ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(Math.toRadians(angle)));
        return Math.min(hMax, vMax);
    }

    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(
                        (pivot1.getSelectedSensorPosition() + pivot2.getSelectedSensorPosition())/2,
                        ArmConstants.PIVOT_GEAR_RATIO)
        );
    }

    private double getExtension() {
        return telescopeEncoder.getPosition() * ArmConstants.EXT_ROTS_TO_INCHES / ArmConstants.EXT_GEAR_RATIO + ArmConstants.ARM_BASE_LENGTH;
    }

    public ArmState getArmState() {
        return ArmState.fromRotationExtension(getRotation2d(), getExtension());
    }

    public boolean getLimitSwitch() {
        return telescopeLimitSwitch.isPressed();
    }

    protected void setRotation(Rotation2d rotation) {
        setRotation(rotation, true);
    }

    protected void setRotation(Rotation2d rotation, boolean resetEncoders) {
        if (resetEncoders) resetToEncoder();
        double angle = rotation.getDegrees();
        pivot1.set(ControlMode.MotionMagic, Conversions.degreesToFalcon(angle, ArmConstants.PIVOT_GEAR_RATIO));
    }

    protected void setExtension(double extension) {
        telescopePID.setReference(
                (extension - ArmConstants.ARM_BASE_LENGTH) * ArmConstants.EXT_GEAR_RATIO / ArmConstants.EXT_ROTS_TO_INCHES,
                CANSparkMax.ControlType.kSmartMotion, 0
        );

    }

    public void stopAll() {
        pivot1.set(ControlMode.PercentOutput, 0);
        telescope.set(0);
    }

    public Rotation2d getGyroAngle() {
        short[] pigeonAccel = new short[3];
        pigeon2.getBiasedAccelerometer(pigeonAccel);
        double pigeonRoll;
        if (pigeonAccel[0] > 0) {
            pigeonRoll = pigeon2.getRoll() > 0 ? pigeon2.getRoll() - 180 : pigeon2.getRoll() + 180;
        } else pigeonRoll = pigeon2.getRoll();
        return Rotation2d.fromDegrees(pigeonRoll + ArmConstants.PIGEON_OFFSET);
    }

    public void resetToGyro() {
//        short[] pigeonAccel = new short[3];
//        pigeon2.getBiasedAccelerometer(pigeonAccel);
//        double pigeonRoll;
//        if (pigeonAccel[0] > 0) {
//            pigeonRoll = pigeon2.getRoll() > 0 ? pigeon2.getRoll() - 180 : pigeon2.getRoll() + 180;
//        } else pigeonRoll = pigeon2.getRoll();
//        pivot1.setSelectedSensorPosition(
//                Conversions.degreesToFalcon(
//                        pigeonRoll + ArmConstants.PIGEON_OFFSET,
//                        ArmConstants.PIVOT_GEAR_RATIO
//                )
//        );
        pivotEncoder.setPosition(getGyroAngle());
        pivot1.setSelectedSensorPosition(
                Conversions.degreesToFalcon(pivotEncoder.getDegrees(), ArmConstants.PIVOT_GEAR_RATIO)
        );
    }

    public void resetToEncoder() {
        pivot1.setSelectedSensorPosition(
                Conversions.degreesToFalcon(pivotEncoder.getDegrees(), ArmConstants.PIVOT_GEAR_RATIO)
        );
        pivot2.setSelectedSensorPosition(
                Conversions.degreesToFalcon(pivotEncoder.getDegrees(), ArmConstants.PIVOT_GEAR_RATIO)
        );
    }


    public void setExtendingSpeed(double speed){
        telescope.set(speed);
    }

    public double getExtendingSpeed() {
        return telescopeEncoder.getVelocity();
    }

    public void setRotatingSpeed(double speed) {
        pivot1.set(ControlMode.PercentOutput, speed);
    }

    public double getRotationSpeed() {
        return Conversions.falconToRPM(
                (pivot1.getSelectedSensorVelocity() + pivot2.getSelectedSensorVelocity()) / 2,
                1
        );
    }

    public void setRotationNeutralMode(NeutralMode mode){
        pivot1.setNeutralMode(mode);
        pivot2.setNeutralMode(mode);
    }
    public void setExtensionNeutralMode(CANSparkMax.IdleMode mode){
        telescope.setIdleMode(mode);
    }

    public void holdPivot() {
        setRotation(getRotation2d(), false);
    }

    public void setPivotAccel(double pivotAccel){
        pivot1.configMotionAcceleration(pivotAccel);
        this.pivotAccel = pivotAccel;
    }

    public void holdExtension() {
        setExtension(getExtension());
    }

    private void smashDartboardInit() {
    }


    private void smashDartboard() {
        SmartDashboard.putNumber("arm/pigeonRoll", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("arm/pivotAngleDegrees", getRotation2d().getDegrees());
        SmartDashboard.putNumber("arm/extension", getExtension());
        SmartDashboard.putNumber("arm/pivotEncoder", pivotEncoder.getDegrees());
        SmartDashboard.putBoolean("arm/limit", getLimitSwitch());
        SmartDashboard.putNumber("arm/Xpos", getArmState().getX());
        SmartDashboard.putNumber("arm/Ypos", getArmState().getY());
        SmartDashboard.putNumber("arm/extensionRots", telescopeEncoder.getPosition());
        SmartDashboard.putBoolean("arm/isLegal", getExtension() < getMaxExtension());
        SmartDashboard.putNumber("arm/maxExtension", getMaxExtension());
        SmartDashboard.putNumber("arm/cartesianAngle", Conversions.actualToCartesian(getRotation2d()).getDegrees());
        SmartDashboard.putNumber("arm/absoluteEncoder", absEncoder.getAbsolutePosition() * 360);
    }

    @Override
    public void periodic() {
//        if(!isManualControl) limitArmExtension();
        if (getLimitSwitch()) telescopeEncoder.setPosition(0);
        smashDartboard();
    }
}
