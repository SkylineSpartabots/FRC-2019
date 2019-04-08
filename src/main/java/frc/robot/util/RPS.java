package frc.robot.util;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * @author NeilHazra
 */

public class RPS {
	private double z_axis_offset = 0.43;
	private double z_scale = 1;
	private double positive_x_fudgefactor = 1;
	private double negative_x_fudgefactor = 1;
	private double positive_x_fudge_offset = -0.03;
	private double negative_x_fudge_offset = -0.03;

	private double lastUpdateTimeLimit = 2000000;

	private AHRS ahrs;
	
	public double angleOffset = 0; 


	
	private NetworkTableEntry XDisp;
	private NetworkTableEntry YDisp;
	private NetworkTableEntry ZDisp;
	private NetworkTableEntry Yaw;

	//private ShuffleboardTab visionTab;
	//private NetworkTableEntry Z_AXIS_OFFSET, Z_SCALE, POSITIVE_X_FUDGE_FACTOR, NEGATIVE_X_FUDGE_FACTOR, 
	//POSITIVE_X_FUDGE_OFFSET, NEGATIVE_X_FUDGE_OFFSET;

	
	public RPS() {
		XDisp = Robot.JetsonTable.getEntry("X Displacement");
		YDisp = Robot.JetsonTable.getEntry("Y Displacement");
		ZDisp = Robot.JetsonTable.getEntry("Z Displacement");
		Yaw = Robot.JetsonTable.getEntry("Yaw");
		ahrs = new AHRS(SPI.Port.kMXP);

		/*visionTab = Shuffleboard.getTab("VisionConstants");

		Z_AXIS_OFFSET = visionTab.add("Z Axis Offset", 0.5).withWidget(BuiltInWidgets.kTextView)
			.withProperties(Map.of("Min", -5, "Max", 5)).getEntry();
		
		Z_SCALE = visionTab.add("Z Scale", 1).withWidget(BuiltInWidgets.kTextView)
			.withProperties(Map.of("Min", -5, "Max", 5)).getEntry();

		POSITIVE_X_FUDGE_FACTOR = visionTab.add("Positive X Fudge Factor", 0.85).withWidget(BuiltInWidgets.kTextView)
			.withProperties(Map.of("Min", -5., "Max", 5)).getEntry();
		
		NEGATIVE_X_FUDGE_FACTOR = visionTab.add("Negative X Fudge Factor", 1).withWidget(BuiltInWidgets.kTextView)
			.withProperties(Map.of("Min", -5, "Max", 5)).getEntry();
	
		POSITIVE_X_FUDGE_OFFSET = visionTab.add("Positive X Fudge Offset", 1.0/13.0).withWidget(BuiltInWidgets.kTextView)
			.withProperties(Map.of("Min", -5, "Max", 5)).getEntry();
		
		NEGATIVE_X_FUDGE_OFFSET = visionTab.add("Negative X Fudge offset", 2.0/13.0).withWidget(BuiltInWidgets.kTextView)
			.withProperties(Map.of("Min", -5, "Max", 5)).getEntry();*/
		
	}

	public void reset() {
		angleOffset += ahrs.getAngle();
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
		return getZDisplacementToVisionTargetRawMeters()*z_scale - 
			z_axis_offset; 
	}

	public double getXDisplacementEditedForCameraPositionMeters()	{
		double x_dist;		
		if(Robot.rps.getXDisplacementToVisionTargetRawMeters() > 0)	{
			x_dist = getXDisplacementToVisionTargetRawMeters()*positive_x_fudgefactor +
				positive_x_fudge_offset;
		}	else {
			x_dist = getXDisplacementToVisionTargetRawMeters()*negative_x_fudgefactor +
				negative_x_fudge_offset;
		}
		return x_dist;
	} 

	//will return degrees
	public double getYawToVisionTargetRawDegrees()	{
		if(!isTargetSeenRecently()) return Double.NaN;
		return (double) Yaw.getNumber(0);
	}
	public boolean isVisionAlive() {
		return Timer.getFPGATimestamp()*1000000 - Robot.JetsonTable.getEntry("IsAliveCounter").getLastChange() < 4000000;
	}
	public boolean isTargetSeenRecently()	{
		return Timer.getFPGATimestamp()*1000000 - ZDisp.getLastChange() < lastUpdateTimeLimit;
	}


	public double getAbsoluteAngle()	{
		return ahrs.getAngle()+angleOffset;
	}
	public double getAbsoluteAngleHalfDegrees()	{
		double newAngle = getAbsoluteAngle();
		while (newAngle <= -180) newAngle += 360;
		while (newAngle > 180) newAngle -= 360;
		return newAngle; //-179, 180
	}
	public double getAngleToDepot()	{
		double angle = getAbsoluteAngleHalfDegrees();
		if(angle >= 0) {
			return (180-angle);
		}	else	{
			return (-180-angle);
		}
	}
	public double getNavxAngle()	{
		return ahrs.getAngle();
	}
}