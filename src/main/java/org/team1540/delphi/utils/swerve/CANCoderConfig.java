package org.team1540.delphi.utils.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class CANCoderConfig {
    public static void applyConfig(double magnetOffsetDegrees, CANCoder ...cancoders) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = magnetOffsetDegrees;
        
        for (CANCoder cancoder : cancoders) {
            cancoder.configAllSettings(config);
        }
    }
}
