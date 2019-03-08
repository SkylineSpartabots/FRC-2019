package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * @author NeilHazra
 */
public class RPS {

	private AHRS ahrs;
	private NetworkTableEntry XDisp;
	private NetworkTableEntry YDisp;
	private NetworkTableEntry ZDisp;
	private NetworkTableEntry Yaw;
	public RPS() {
		XDisp = Robot.JetsonTable.getEntry("X Displacement");
		YDisp = Robot.JetsonTable.getEntry("Y Displacement");
		ZDisp = Robot.JetsonTable.getEntry("Z Displacement");
		Yaw = Robot.JetsonTable.getEntry("Yaw");
		ahrs = new AHRS(SPI.Port.kMXP);
	}

	public void reset() {
		ahrs.reset();
	}
	public double getXDisplacementToVisionTarget()	{
		return ((double) XDisp.getNumber(-1));
	}
	public double getYDisplacementToVisionTarget()	{
		return (double) YDisp.getNumber(-1);
	}
	public double getZDisplacementToVisionTarget()	{
		return (double) ZDisp.getNumber(-1);
	}
	//will return degrees
	public double getYawToVisionTarget()	{
		return (double) Yaw.getNumber(-1);
	}

	// Will return degrees
	public double getNavxAngle() {
		return ahrs.getAngle();
	}
}