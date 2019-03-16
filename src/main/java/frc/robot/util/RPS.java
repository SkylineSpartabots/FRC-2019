package frc.robot.util;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author NeilHazra
 */
public class RPS {
	private double z_axis_offset = 0.3;
	private double z_scale = 1;
	private double positive_x_fudgefactor = 1;
	private double negative_x_fudgefactor = 1;
	private double positive_x_fudge_offset = 0.;
	private double negative_x_fudge_offset = 0.;

	private double lastUpdateTimeLimit = 2000000;

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

	//todo check for old data
	public double getXDisplacementToVisionTargetRawInches()	{
		if(!isTargetSeenRecently()) return Double.NaN;
		return ((double) XDisp.getNumber(Double.NaN));
	}
	public double getYDisplacementToVisionTargetRawInches()	{
		if(!isTargetSeenRecently()) return Double.NaN;
		return (double) YDisp.getNumber(Double.NaN);
	}
	public double getZDisplacementToVisionTargetRawInches()	{
		if(!isTargetSeenRecently()) return Double.NaN;
		return (double) ZDisp.getNumber(Double.NaN);
	}

	public double getXDisplacementToVisionTargetRawMeters()	{
		return getXDisplacementToVisionTargetRawInches()*0.0254;
	}
	public double getYDisplacementToVisionTargetRawMeters()	{
		return getYDisplacementToVisionTargetRawInches()*0.0254;
	}
	public double getZDisplacementToVisionTargetRawMeters()	{
		return getZDisplacementToVisionTargetRawInches()*0.0254;
	}

	public double getZDisplacementEditedForCameraPositionMeters()	{
		return getZDisplacementToVisionTargetRawMeters()*z_scale - z_axis_offset; 
	} 
	public double getXDisplacementEditedForCameraPositionMeters()	{
		double x_dist;		
		if(Robot.rps.getXDisplacementToVisionTargetRawMeters() > 0)	{
			x_dist = getXDisplacementToVisionTargetRawMeters()*positive_x_fudgefactor+positive_x_fudge_offset;
		}	else {
			x_dist = getXDisplacementToVisionTargetRawMeters()*negative_x_fudgefactor+negative_x_fudge_offset;
		}
		return x_dist;
	} 

	//will return degrees
	public double getYawToVisionTargetRawDegrees()	{
		if(!isTargetSeenRecently()) return Double.NaN;
		return (double) Yaw.getNumber(0);
	}
	public boolean isVisionAlive() {
		return Timer.getFPGATimestamp()*1000000 - Robot.JetsonTable.getEntry("IsAliveCounter").getLastChange() < 2000000;
	}
	public boolean isTargetSeenRecently()	{
		return Timer.getFPGATimestamp()*1000000 - ZDisp.getLastChange() < lastUpdateTimeLimit;
	}


	// Will return degrees
	public double getNavxAngle() {
		return ahrs.getAngle();
	}
}