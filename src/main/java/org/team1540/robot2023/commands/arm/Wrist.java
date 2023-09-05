package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.lib.math.Conversions;

import static org.team1540.robot2023.Constants.WristConstants.*;

public class Wrist extends SubsystemBase {
    private final TalonFX wrist = new TalonFX(WRIST_ID);
    private final CANCoder encoder = new CANCoder(CANCODER_ID);

    public Wrist() {
        encoder.configSensorDirection(false); // TODO: 8/29/2023 figure out inversion
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.configMagnetOffset(CANCODER_OFFSET);
        encoder.configFeedbackCoefficient((360 * CANCODER_GEARING)/4096, "deg", SensorTimeBase.PerSecond);

        wrist.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, WRIST_CURRENT, WRIST_CURRENT, 0)
        );
        wrist.setNeutralMode(NeutralMode.Brake);

        wrist.configForwardSoftLimitThreshold(WRIST_FORWARD_LIMIT);
        wrist.configForwardSoftLimitEnable(true);
        wrist.configReverseSoftLimitThreshold(WRIST_REVERSE_LIMIT);
        wrist.configReverseSoftLimitEnable(true);
        wrist.setInverted(false); // TODO: 8/29/2023 figure out inversion

        wrist.config_kP(0, WRIST_KP);
        wrist.config_kI(0, WRIST_KI);
        wrist.config_kD(0, WRIST_KD);
    }

    public void resetToAbsolute() {
        wrist.setSelectedSensorPosition(Conversions.degreesToFalcon(encoder.getAbsolutePosition(), WRIST_GEARING));
    }

    public void setWristRotation(Rotation2d rotation, boolean reset) {
        if (reset) resetToAbsolute();
        double degrees = rotation.getDegrees();
        wrist.set(ControlMode.Position, Conversions.degreesToFalcon(degrees, WRIST_GEARING));
    }

    public void setWristRotation(Rotation2d rotation) {
        setWristRotation(rotation, true);
    }

    public void smashDartboard() {
        SmartDashboard.putNumber(
                "arm/wrist/motorAngle", Conversions.falconToDegrees(wrist.getSelectedSensorPosition(), WRIST_GEARING)
        );
        SmartDashboard.putNumber("arm/wrist/encoderAngle", encoder.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        smashDartboard();
    }
}
