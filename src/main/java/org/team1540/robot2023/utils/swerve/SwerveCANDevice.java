package org.team1540.robot2023.utils.swerve;

public class SwerveCANDevice {

    /**
     * Derives the CAN ID of a CANCoder from its module ID
     * @param moduleID the ID of the swerve module
     * @return the CANCoder's CAN ID
     */
    public static int getCancoderID(int moduleID) {
        return 11+moduleID;
    }
    
    /**
     * Derives the CAN ID of a swerve turning motor from its module ID
     * @param moduleID the ID of the swerve module
     * @return the motor's CAN ID
     */
    public static int getTurningMotorID(int moduleID) {
        return 21+moduleID;
    }

    /**
     * Derives the CAN ID of a swerve driving motor from its module ID
     * @param moduleID the ID of the swerve module
     * @return the motor's CAN ID
     */
    public static int getDrivingMotorID(int moduleID) {
        return 31+moduleID;
    }
}
