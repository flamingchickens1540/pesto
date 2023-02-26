package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.lib.math.Conversions;
import org.team1540.robot2023.Constants;

public class Pivot extends SubsystemBase {

    private final TalonFX pivot1 = new TalonFX(Constants.ArmConstants.PIVOT1_ID);
    private final TalonFX pivot2 = new TalonFX(Constants.ArmConstants.PIVOT2_ID);
    private final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(Constants.ArmConstants.PIGEON_ID);

    public Pivot(){
        pivot1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0));
        pivot2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0));

        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot2.setNeutralMode(NeutralMode.Brake);

        pivot1.setInverted(true);
        pivot1.configForwardSoftLimitThreshold(Constants.ArmConstants.PIVOT_FORWARD_LIMIT);
        pivot1.configForwardSoftLimitEnable(true);
        pivot1.configReverseSoftLimitThreshold(Constants.ArmConstants.PIVOT_REVERSE_LIMIT);
        pivot1.configReverseSoftLimitEnable(true);
        pivot2.follow(pivot1);

        pivot1.config_kP(0, Constants.ArmConstants.PIVOT_KP);
        pivot1.config_kI(0, Constants.ArmConstants.PIVOT_KI);
        pivot1.config_kD(0, Constants.ArmConstants.PIVOT_KD);
        pivot1.configMotionCruiseVelocity(Constants.ArmConstants.PIVOT_CRUISE_SPEED);
        pivot1.configMotionAcceleration(Constants.ArmConstants.PIVOT_MAX_ACCEL);

        pigeon2.configMountPose(Constants.ArmConstants.PIGEON_MNT_YAW, Constants.ArmConstants.PIGEON_MNT_PITCH, Constants.ArmConstants.PIGEON_MNT_ROLL);

        resetAngle();


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm/pigeonRoll", pigeon2.getRoll());
        SmartDashboard.putNumber("arm/pivotAngleDegrees", getRotation2d().getDegrees());
        SmartDashboard.putNumber("arm/pivotEncoder", pivot1.getSelectedSensorPosition());
    }

    Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(pivot1.getSelectedSensorPosition(), Constants.ArmConstants.PIVOT_GEAR_RATIO)
        );
    }

    public void setRotation(Rotation2d rotation) {
        double angle = rotation.getDegrees();
        pivot1.set(ControlMode.MotionMagic, Conversions.degreesToFalcon(angle, Constants.ArmConstants.PIVOT_GEAR_RATIO));
    }

    public void stop(){
        pivot1.set(ControlMode.PercentOutput, 0);
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
                        pigeonRoll + Constants.ArmConstants.PIGEON_OFFSET,
                        Constants.ArmConstants.PIVOT_GEAR_RATIO
                )
        );
    }

    public void setRotatingSpeed(double speed){
        pivot1.set(ControlMode.PercentOutput, speed);
    }

    public void setNeutralMode(NeutralMode mode){
        pivot1.setNeutralMode(mode);
        pivot2.setNeutralMode(mode);
    }
}
