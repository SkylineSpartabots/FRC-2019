package frc.robot.util;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;


/**
 * @author NeilHazra
 */

public class RPS {
	public AHRS ahrs;
	public RPS() {
		ahrs = new AHRS(SPI.Port.kMXP);
	}
	public void reset() {
	 ahrs.reset();
	}
	// Will return degrees
	public double getAngle() {
		return ahrs.getAngle();
	}
}