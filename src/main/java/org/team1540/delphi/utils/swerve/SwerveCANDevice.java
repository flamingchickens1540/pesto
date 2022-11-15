package org.team1540.delphi.utils.swerve;

public enum SwerveCANDevice {
    CANCODER(10),
    TURNING_MOTOR(20),
    DRIVING_MOTOR(30);
    
    private int prefix;
    private SwerveCANDevice(int prefix) {
        this.prefix = prefix;
    }
    
    public int getDeviceID(int moduleID) {
        return prefix+moduleID;
    }
    
}
