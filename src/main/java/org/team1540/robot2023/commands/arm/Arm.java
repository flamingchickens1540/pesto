package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.lib.math.Conversions;
import org.team1540.robot2023.Constants.ArmConstants;
import org.team1540.robot2023.utils.ArmState;

public class Arm extends SubsystemBase {
    private final TalonFX pivot1 = new TalonFX(ArmConstants.PIVOT1_ID);
    private final TalonFX pivot2 = new TalonFX(ArmConstants.PIVOT2_ID);

    private final CANSparkMax telescope = new CANSparkMax(ArmConstants.TELESCOPE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder telescopeEncoder = telescope.getEncoder();
    private final SparkMaxPIDController telescopePID = telescope.getPIDController();
    private final SparkMaxLimitSwitch telescopeLimitSwitch = telescope.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    private final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(ArmConstants.PIGEON_ID);

    public Arm() {
        pivot1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0));
        pivot2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0));
        telescope.setSmartCurrentLimit(40);

        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot2.setNeutralMode(NeutralMode.Brake);
        telescope.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivot1.setInverted(true);
        pivot1.configForwardSoftLimitThreshold(ArmConstants.PIVOT_FORWARD_LIMIT);
        pivot1.configForwardSoftLimitEnable(true);
        pivot1.configReverseSoftLimitThreshold(ArmConstants.PIVOT_REVERSE_LIMIT);
        pivot1.configReverseSoftLimitEnable(true);
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
        telescopePID.setSmartMotionMaxAccel(ArmConstants.TELESCOPE_MAX_ACCEL, 0);
        telescopePID.setSmartMotionMaxVelocity(ArmConstants.TELESCOPE_CRUISE_SPEED, 0);
        telescopePID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);

        pigeon2.configMountPose(ArmConstants.PIGEON_MNT_YAW, ArmConstants.PIGEON_MNT_PITCH, ArmConstants.PIGEON_MNT_ROLL);

        resetAngle();

        smashDartboardInit();
    }

    public double getMaxExtension() {
        return getMaxExtension(getRotation2d());
    }

    public double getMaxExtension(Rotation2d rotation) {
        double angle = Conversions.actualToCartesian(rotation).getRadians();
        if (angle == Math.PI / 2) return ArmConstants.MAX_LEGAL_HEIGHT;
        if (angle == 0 || angle == Math.PI) return ArmConstants.MAX_LEGAL_DISTANCE;
        else if (angle > 0 && angle < Math.PI){
            return Math.min(Math.abs(ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(angle)), Math.abs(ArmConstants.MAX_LEGAL_HEIGHT / Math.sin(angle)));
        }
        else {
            return Math.min(Math.abs(ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(angle)), Math.abs(ArmConstants.PIVOT_HEIGHT / Math.sin(angle)));
        }
    }

    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(pivot1.getSelectedSensorPosition(), ArmConstants.PIVOT_GEAR_RATIO)
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

    public void setRotation(Rotation2d rotation) {
        double angle = rotation.getDegrees();
         pivot1.set(ControlMode.MotionMagic, Conversions.degreesToFalcon(angle, ArmConstants.PIVOT_GEAR_RATIO));
    }

    protected void setExtension(double extension) {
        telescopePID.setReference(
                (extension - ArmConstants.ARM_BASE_LENGTH) * ArmConstants.EXT_GEAR_RATIO / ArmConstants.EXT_ROTS_TO_INCHES,
                CANSparkMax.ControlType.kPosition, 0
        );

    }

    public void stopAll() {
        pivot1.set(ControlMode.PercentOutput, 0);
        telescope.set(0);
    }

    public void resetAngle() {
        short[] pigeonAccel = new short[3];
        pigeon2.getBiasedAccelerometer(pigeonAccel);
        double pigeonRoll;
        if (pigeonAccel[0] > 0) {
            pigeonRoll = pigeon2.getRoll() > 0 ? pigeon2.getRoll() - 180 : pigeon2.getRoll() + 180;
        } else pigeonRoll = pigeon2.getRoll();
        pivot1.setSelectedSensorPosition(
                Conversions.degreesToFalcon(
                        pigeonRoll + ArmConstants.PIGEON_OFFSET,
                        ArmConstants.PIVOT_GEAR_RATIO
                )
        );
    }


    public void setExtendingSpeed(double speed){
        telescope.set(speed);
    }

    public void setRotatingSpeed(double speed){
        pivot1.set(ControlMode.PercentOutput, speed);
    }
    public void setRotationNeutralMode(NeutralMode mode){
        pivot1.setNeutralMode(mode);
        pivot2.setNeutralMode(mode);
    }

    private void smashDartboardInit() {
    }

    private void smashDartboard() {
        SmartDashboard.putNumber("arm/pigeonRoll", pigeon2.getRoll());
        SmartDashboard.putNumber("arm/pivotAngleDegrees", getRotation2d().getDegrees());
        SmartDashboard.putNumber("arm/extension", getExtension());
        SmartDashboard.putNumber("arm/pivotEncoder", pivot1.getSelectedSensorPosition());
        SmartDashboard.putBoolean("arm/limit", getLimitSwitch());
        SmartDashboard.putNumber("arm/Xpos", getArmState().getX());
        SmartDashboard.putNumber("arm/Ypos", getArmState().getY());
        SmartDashboard.putNumber("arm/extensionRots", telescopeEncoder.getPosition());
        SmartDashboard.putBoolean("arm/isLegal", getExtension() > getMaxExtension());
    }

    @Override
    public void periodic() {
//        if(!isManualControl) limitArmExtension();
        if (getLimitSwitch()) telescopeEncoder.setPosition(0);
        smashDartboard();
    }
}
