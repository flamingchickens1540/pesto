package org.team1540.delphi.utils;

public class MathUtils {
    /**
     * Returns 0 if the input is within the deadzone, else the value
     *
     * @param value The joystick input
     * @return The value after deadzone is checked
     */
    public static double deadzone(double value, double deadzone) {
        if (java.lang.Math.abs(value) <= deadzone)
            return 0;
        else
            return value;
    }

    /**
     * Scale a value between range [outMin, outMax]. All thresholds are inclusive.
     *
     * @param input  value to scale
     * @param inMin  minimum input value
     * @param inMax  maximum input value
     * @param outMin minimum output value
     * @param outMax maximum output value
     * @return normalized value
     */
    public static double normalize(double input, double inMin, double inMax, double outMin, double outMax) {
        return (outMax - outMin) * ((input - inMin) / (inMax - inMin)) + outMin;
    }
}
