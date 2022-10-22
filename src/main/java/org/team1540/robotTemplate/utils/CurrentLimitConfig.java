package org.team1540.robotTemplate.utils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class CurrentLimitConfig {
    public final double statorLimit, statorThreshCurrent, statorThreshTime;
    public final double supplyLimit, supplyThreshCurrent, supplyThreshTime;

    /**
     * CurrentLimitConfig stores stator and supply constants that can be applied to TalonFX controller.
     *
     * @param statorLimit         stator current limit amps
     * @param statorThreshCurrent stator threshold trigger current amps
     * @param statorThreshTime    stator threshold limit time seconds
     * @param supplyLimit         supply current limit amps
     * @param supplyThreshCurrent supply threshold trigger current amps
     * @param supplyThreshTime    supply threshold limit time seconds
     */
    public CurrentLimitConfig(double statorLimit, double statorThreshCurrent, double statorThreshTime,
                              double supplyLimit, double supplyThreshCurrent, double supplyThreshTime) {
        this.statorLimit = statorLimit;
        this.statorThreshCurrent = statorThreshCurrent;
        this.statorThreshTime = statorThreshTime;

        this.supplyLimit = supplyLimit;
        this.supplyThreshCurrent = supplyThreshCurrent;
        this.supplyThreshTime = supplyThreshTime;
    }

    /**
     * Reset a TalonFX controller to default settings
     * @param motor TalonFX controller to reset
     */
    public void reset(TalonFX motor) {
        TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
        defaultConfig.voltageCompSaturation = 12;
        defaultConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.openloopRamp = 0;

        motor.configFactoryDefault();
        motor.configAllSettings(defaultConfig);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.enableVoltageCompensation(true);
    }


    /**
     * Apply this current limit to one or more TalonFX controllers
     *
     * @param motors TalonFX controllers to apply the current limit to
     */
    public void applyTo(TalonFX... motors) {
        for (TalonFX motor : motors) {
            reset(motor);
            motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, this.statorLimit, this.statorThreshCurrent, this.statorThreshTime));
            motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, this.supplyLimit, this.supplyThreshCurrent, this.supplyThreshTime));
        }
    }
}
