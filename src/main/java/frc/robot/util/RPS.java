package frc.robot.util;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author NeilHazra
 */
public class RPS {
	private double z_axis_offset = 0.45;
	private double positive_x_fudgefactor = 1;
	private double negative_x_fudgefactor = 1;


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
		return ((double) XDisp.getNumber(-3000));
	}
	public double getYDisplacementToVisionTargetRawInches()	{
		return (double) YDisp.getNumber(-3000);
	}
	public double getZDisplacementToVisionTargetRawInches()	{
		SmartDashboard.putNumber("LastVisionUpdate", ZDisp.getLastChange());
		return (double) ZDisp.getNumber(-3000);
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
		return getZDisplacementToVisionTargetRawMeters() - z_axis_offset; 
	} 
	public double getXDisplacementEditedForCameraPositionMeters()	{
		double x_dist;		
		if(Robot.rps.getXDisplacementToVisionTargetRawMeters() > 0)	{
			x_dist = getXDisplacementToVisionTargetRawMeters()*positive_x_fudgefactor;
		}	else {
			x_dist = getXDisplacementToVisionTargetRawMeters()*negative_x_fudgefactor;
		}
		return x_dist;
	} 

	//will return degrees
	public double getYawToVisionTargetRawDegrees()	{
		return (double) Yaw.getNumber(0);
	}


	// Will return degrees
	public double getNavxAngle() {
		return ahrs.getAngle();
	}
}