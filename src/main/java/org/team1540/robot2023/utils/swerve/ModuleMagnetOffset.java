package org.team1540.robot2023.utils.swerve;


/**
 * Represents a swerve module's magnet offset. Measured by aligning the wheel so that it would be facing left if the module was in the top left corner and finding the value that would make it zero.
 */
public class ModuleMagnetOffset {
    private static final double[] offsets = new double[]{
            150.469, // Module 1
            91.318,  // Module 2
            256.729, // Module 3
            9.0,     // Module 4
            239.766, // Module 5
            32.08,  // Module 6
            27.861, // Module 7
            202.061  // Module 8
    };

    /**
     * Returns the cancoder offset for this module as oriented in the front left corner
     * @param moduleID The ID of the module as printed on the label, 1-8 inclusive;
     */
    public static double get(int moduleID) {
        return offsets[moduleID-1];
    }
}
