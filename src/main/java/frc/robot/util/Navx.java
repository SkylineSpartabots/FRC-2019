package frc.robot.util;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class Navx {


	private AHRS ahrs;

	public Navx() {
		ahrs = new AHRS(SPI.Port.kMXP);
	}

	public void reset() {
		ahrs.reset();
	}
	
	public double getAngle()	{
		return ahrs.getAngle();
	}

	

}