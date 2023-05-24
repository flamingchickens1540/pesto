package org.team1540.robot2023.utils.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.team1540.robot2023.Constants;

public class CANCoderConfig {
    public static void applyConfig(CANcoder ...cancoders) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        for (CANcoder cancoder : cancoders) {
            cancoder.getConfigurator().apply(config);
        }
    }
}
