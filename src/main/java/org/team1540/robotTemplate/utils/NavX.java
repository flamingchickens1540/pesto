package org.team1540.robotTemplate.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;

public class NavX extends AHRS {

    public NavX(Port port) {
        super(port);
    }

    /**
     * @return NavX yaw counter-clockwise in radians, from -pi to pi. This method
     *         does NOT continue past pi or -pi and is thus the one you probably
     *         want to use most of the time.
     */
    public double getYawRadians() {
        return -Math.toRadians(this.getYaw());
    }

    /**
     * @return NavX angle counter-clockwise in radians. This method continues past
     *         pi and -pi and is thus the one you don't want to use (most of the
     *         time).
     */
    public double getAngleRadians() {
        return -Math.toRadians(this.getAngle());
    }

    public double getRateRadians() {
        return -Math.toRadians(this.getRate());
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getYawRadians());
    }

}
