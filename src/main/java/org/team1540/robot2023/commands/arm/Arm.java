package org.team1540.robot2023.commands.arm;


import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private final MotionMagicDutyCycle positionControl = new MotionMagicDutyCycle(0);
    private final DutyCycleOut percentControl = new DutyCycleOut(0);

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

    private final Pigeon2 pigeon2 = new Pigeon2(ArmConstants.PIGEON_ID);

    private double pivotAccel;

    private static double armToFalcon(Rotation2d distance) {
        return distance.getRotations() * ArmConstants.PIVOT_GEAR_RATIO;
    }
    private static double armDegreesToFalcon(double distance) {
        return (distance/360) * ArmConstants.PIVOT_GEAR_RATIO;
    }

    private static double falconToArmDegrees(double rotations) {
        return rotations*360/ArmConstants.PIVOT_GEAR_RATIO;
    }

    private TalonFXConfiguration getTalonConfig() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.CurrentLimits.StatorCurrentLimit = 60;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.PIVOT_FORWARD_LIMIT;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.PIVOT_REVERSE_LIMIT;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.Slot0.kP = ArmConstants.PIVOT_KP;
        configs.Slot0.kI = ArmConstants.PIVOT_KI;
        configs.Slot0.kD = ArmConstants.PIVOT_KD;
        configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.PIVOT_CRUISE_SPEED;
        configs.MotionMagic.MotionMagicAcceleration = ArmConstants.PIVOT_MAX_ACCEL;
        return configs;
    }

    private Pigeon2Configuration getPigeonConfig() {
        Pigeon2Configuration configs = new Pigeon2Configuration();
        configs.MountPose.MountPosePitch = ArmConstants.PIGEON_MNT_PITCH;
        configs.MountPose.MountPoseRoll = ArmConstants.PIGEON_MNT_ROLL;
        configs.MountPose.MountPoseYaw = ArmConstants.PIGEON_MNT_YAW;
        return configs;
    }
    public Arm() {
//        telescope.restoreFactoryDefaults();
        TalonFXConfiguration configs = getTalonConfig();
        pivot2.getConfigurator().apply(configs);
        pivot1.getConfigurator().apply(configs);
        pigeon2.getConfigurator().apply(getPigeonConfig());

        telescope.setSmartCurrentLimit(40);


        telescope.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivot1.setInverted(false);
        pivot2.setControl(new Follower(pivot1.getDeviceID(), true));

        telescope.setInverted(true);
        telescope.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.TELESCOPE_FORWARD_LIMIT);
        telescope.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);


        telescopePID.setP(ArmConstants.TELESCOPE_KP);
        telescopePID.setI(ArmConstants.TELESCOPE_KI);
        telescopePID.setD(ArmConstants.TELESCOPE_KD);
        telescopePID.setFF(ArmConstants.TELESCOPE_KF);
        telescopePID.setSmartMotionMaxAccel(ArmConstants.TELESCOPE_MAX_ACCEL, 0);
        telescopePID.setSmartMotionMaxVelocity(ArmConstants.TELESCOPE_CRUISE_SPEED, 0);
        telescopePID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);



        smashDartboardInit();
    }

    public double getMaxExtension() {
        return getMaxExtension(getRotation2d());
    }

    public double timeToRotation(Rotation2d rotation2d){
        double setpoint = armToFalcon(rotation2d);
        double distance = Math.abs(setpoint - pivot1.getRotorPosition().getValue());
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
                falconToArmDegrees((pivot1.getRotorPosition().getValue() + pivot2.getRotorPosition().getValue())/2)
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
        pivot1.setControl(positionControl.withPosition(armToFalcon(rotation)));
    }

    protected void setExtension(double extension) {
        telescopePID.setReference(
                (extension - ArmConstants.ARM_BASE_LENGTH) * ArmConstants.EXT_GEAR_RATIO / ArmConstants.EXT_ROTS_TO_INCHES,
                CANSparkMax.ControlType.kSmartMotion, 0
        );

    }

    public void stopAll() {
        pivot1.stopMotor();
        telescope.set(0);
    }

    public Rotation2d getGyroAngle() {
        short[] pigeonAccel = new short[3];
        ;
        double pigeonRoll;
        if (pigeon2.getAccelerationX().getValue() > 0) {
            double raw = pigeon2.getRoll().getValue();
            pigeonRoll = raw > 0 ? raw - 180 : raw + 180;
        } else pigeonRoll = pigeon2.getRoll().getValue();
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
        pivot1.setRotorPosition(pivotEncoder.getRotation2d().getRotations()*ArmConstants.PIVOT_GEAR_RATIO);
    }

    public void resetToEncoder() {
        pivot1.setRotorPosition(pivotEncoder.getRotation2d().getRotations()*ArmConstants.PIVOT_GEAR_RATIO);
        pivot2.setRotorPosition(pivotEncoder.getRotation2d().getRotations()*ArmConstants.PIVOT_GEAR_RATIO);
    }


    public void setExtendingSpeed(double speed){
        telescope.set(speed);
    }

    public double getExtendingSpeed() {
        return telescopeEncoder.getVelocity();
    }

    public void setRotatingSpeed(double speed) {
        pivot1.set(speed);
    }

    public double getRotationSpeed() {
        return (pivot1.getRotorVelocity().getValue() + pivot2.getRotorVelocity().getValue()) / 2/60;
    }

    public void setRotationNeutralMode(NeutralModeValue mode){
        MotorOutputConfigs config = new MotorOutputConfigs();
        pivot1.getConfigurator().refresh(config);
        config.NeutralMode = mode;
        pivot1.getConfigurator().apply(config);
        pivot2.getConfigurator().apply(config);
    }
    public void setExtensionNeutralMode(CANSparkMax.IdleMode mode){
        telescope.setIdleMode(mode);
    }

    public void holdPivot() {
        setRotation(getRotation2d(), false);
    }

    public void setPivotAccel(double pivotAccel){
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = pivotAccel;
        pivot1.getConfigurator().apply(config);
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
        SmartDashboard.putNumber("arm/pigeonAccelX", pigeon2.getAccelerationX().getValue());
    }

    @Override
    public void periodic() {
//        if(!isManualControl) limitArmExtension();
        if (getLimitSwitch()) telescopeEncoder.setPosition(0);
        smashDartboard();
    }
}
