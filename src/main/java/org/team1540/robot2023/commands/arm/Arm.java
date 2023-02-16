package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.lib.math.Conversions;
import org.team1540.robot2023.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX pivot1 = new TalonFX(ArmConstants.PIVOT1_ID);
    private final TalonFX pivot2 = new TalonFX(ArmConstants.PIVOT2_ID);

    private final CANSparkMax telescope = new CANSparkMax(ArmConstants.TELESCOPE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder telescopeEncoder = telescope.getEncoder();
    private final SparkMaxPIDController telescopePID = telescope.getPIDController();
    private final SparkMaxLimitSwitch telescopeLimitSwitch = telescope.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    private final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(ArmConstants.PIGEON_ID);

    private double extensionSetPoint = 0;
    private boolean notSet = false;

    private boolean isManualControl = false;


    public Arm() {
        pivot1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0));
        pivot2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0));
        telescope.setSmartCurrentLimit(40);

        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot2.setNeutralMode(NeutralMode.Brake);
        telescope.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivot1.setInverted(true);
        pivot2.follow(pivot1);
        telescope.setInverted(true);

        pivot1.config_kP(0, ArmConstants.PIVOT_KP);
        pivot1.config_kI(0, ArmConstants.PIVOT_KI);
        pivot1.config_kD(0, ArmConstants.PIVOT_KD);


        telescopePID.setP(ArmConstants.TELESCOPE_KP);
        telescopePID.setI(ArmConstants.TELESCOPE_KI);
        telescopePID.setD(ArmConstants.TELESCOPE_KD);

        pigeon2.configMountPose(ArmConstants.PIGEON_MNT_YAW, ArmConstants.PIGEON_MNT_PITCH, ArmConstants.PIGEON_MNT_ROLL);

        pivot1.setSelectedSensorPosition(
                Conversions.degreesToFalcon(
                        Conversions.cartesianToActual(Rotation2d.fromDegrees(pigeon2.getRoll())).getDegrees(),
                        ArmConstants.PIVOT_GEAR_RATIO
                )
        );

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
        // TODO: figure this out
        return telescopeEncoder.getPosition() * ArmConstants.EXT_ROTS_TO_INCHES / ArmConstants.EXT_GEAR_RATIO + ArmConstants.ARM_BASE_LENGTH;
    }

    public ArmState getArmState() {
        return ArmState.fromRotationExtension(getRotation2d(), getExtension());
    }

    public boolean getLimitSwitch() {
        return telescopeLimitSwitch.isPressed();
    }

    public void setRotation(Rotation2d rotation) {
        // TODO: something
        //Feedforward needs to incorporate how extended the arm is
        //We should also try calculating kF instead of arbitrary
        double angle = rotation.getDegrees();
        double feedforward = Math.cos(getRotation2d().getRadians())*ArmConstants.PIVOT_FF;
        pivot1.set(
                ControlMode.MotionMagic, Conversions.degreesToFalcon(angle, ArmConstants.PIVOT_GEAR_RATIO),
                DemandType.ArbitraryFeedForward,
                feedforward
        );
    }

    public void setExtensionSetPoint(double extensionSetPoint) {
        this.extensionSetPoint = extensionSetPoint;
        setExtension(extensionSetPoint);
    }

    private void setExtension(double extension) {
        // TODO: magic
        //Talk to Kevin about a feedforward for this
        //Might need to be something similar to an arm but with max at straight up not straight out
        //Or in other words, sin
        double feedforward = Math.sin(Conversions.cartesianToActual(getRotation2d()).getRadians()*ArmConstants.TELESCOPE_FF);
        telescopePID.setReference(extension, CANSparkMax.ControlType.kPosition, 0, feedforward);
//        telescope.set(ControlMode.Position,extension, DemandType.ArbitraryFeedForward, feedforward);
//        telescope.set(ControlMode.Position,extension);

    }

    public void stopAll() {
        pivot1.set(ControlMode.PercentOutput, 0);
        telescope.set(0);
    }

    private void limitArmExtension(){
        // TODO: 1/28/2023 Keep an eye on this if problems arise
        if(getMaxExtension() < getExtension()){
            setExtension(getMaxExtension());
            notSet = true;
        }
        else if(isExtending()){
            if(getMaxExtension() < extensionSetPoint){
                setExtension(getMaxExtension());
                notSet = true;
            }
            else if(notSet){
                setExtension(extensionSetPoint);
                notSet = false;
            }
        }
    }

    public boolean isRotating(){
        return pivot1.getSelectedSensorVelocity() > 0.1;
    }

    public boolean isExtending(){
        return telescopeEncoder.getVelocity() > 0.1;
    }

    public void setExtendingSpeed(double speed){
        telescope.set(speed);
    }

    public void setRotatingSpeed(double speed){
        pivot1.set(ControlMode.PercentOutput, speed);
    }

    public void setManualControl(boolean manualControl) {
        isManualControl = manualControl;
    }

    private void smashDartboardInit() {
        SmartDashboard.setDefaultNumber("arm/pigeonRoll", 0);
        SmartDashboard.setDefaultNumber("arm/pivotAngleDegrees", 0);
        SmartDashboard.setDefaultNumber("arm/extension", 0);
        SmartDashboard.setDefaultBoolean("arm/limit", false);
    }

    private void smashDartboard() {
        SmartDashboard.putNumber("arm/pigeonRoll", pigeon2.getRoll());
        SmartDashboard.putNumber("arm/pivotAngleDegrees", getRotation2d().getDegrees());
        SmartDashboard.putNumber("arm/extension", getExtension());
        SmartDashboard.putBoolean("arm/limit", getLimitSwitch());
    }

    @Override
    public void periodic() {
        if(!isManualControl) limitArmExtension();
        if (getLimitSwitch()) telescopeEncoder.setPosition(0);
        smashDartboard();
    }
}
