/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.vision;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Limelight {

    private ShuffleboardTab VISION_TAB;
    private NetworkTableEntry drivekP, drivekI, drivekD, steer_k, maxDrive, maxSteer;
    public static final double DESIRED_TARGET_AREA = 21;

    public Limelight() {
        VISION_TAB = Shuffleboard.getTab("Vision");
        drivekP = VISION_TAB.add("Drive kP", 0.001).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
        drivekI = VISION_TAB.add("Drive kI", 0.0).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
        drivekD = VISION_TAB.add("Drive kD", 0.0).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
        steer_k = VISION_TAB.add("Steer Constant", 0.009).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", -5, "Max", 5)).getEntry();
        maxDrive = VISION_TAB.add("Max Drive Power", 0.3).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
        maxSteer = VISION_TAB.add("Max Steer Power", 0.3).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
        setLEDState(LEDState.BLINK);
    };

    public double[] getDriveConstants() {
        double[] constants = {drivekP.getDouble(0), drivekI.getDouble(0), drivekD.getDouble(0)};
        return constants;
    }

    public double getSteerConstant() {
        return steer_k.getDouble(0);
    }

    public double getMaxDrivePower() {
        return maxDrive.getDouble(0);
    }

    public double getMaxSteerPower() {
        return maxSteer.getDouble(0);
    }


    /**
     * Limelight newtork table returns 0 or 1 if the target is visible or not, ternary operator converts it
     * to a boolean method.
     * @return if the target is visible in the limelight
     */
    public static boolean isTargetVisible() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) < 1 ? false : true;
    }

    
    /**
     * @return the percentage of the image that it taken up by the target
     */
    public static double getTargetArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

    /**
     * 
     * @return horizontal offset from crosshair to target
     */
    public static double getHorizontalOffset() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    /**
     * 
     * @return vertical offset from crosshair to target
     */
    public static double getVerticalOffset() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    
    /**
     * 
     * @return the rotation from 0 to -90 degrees
     */
    public static double getSkew() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    }

    /**
     * 
     * @return the latency of pipline. Add about 11 msec for capture latency
     */
    public static double getLatency() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    }

    /**
     * @return the current pipline that the limelight is running
     */
    public static double getPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0);
    }

    /**
     * 
     * @param pipeline number from 0-9 specifying pipeline used
     */
    public static void setPipeline(int pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }


    public enum LEDState {
        ON(3),
        OFF(1), 
        BLINK(2), 
        PIPELINE(0);

        private int number;
    
        private LEDState(int number){
            this.number = number;
        }

        public int getNumber(){
            return number;
        }
    };
    /**
     * @return sets the led state on the limelight as specified by the enum in this class
     * @param ledState
     */
    public static void setLEDState(LEDState ledState) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState.getNumber());
    }

    public static LEDState getLEDState() {
        double state = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(0);
        switch((int)state) {
            case 1:
                return LEDState.OFF;
            case 2:
                return LEDState.BLINK;
            case 3:
                return LEDState.ON;
            default:
                return LEDState.PIPELINE;
        }
    }

    public static boolean hasReachedTarget() {
        return getTargetArea() > DESIRED_TARGET_AREA;
    }

    public double getSteerCorrection() {
        double steerCorrection = getHorizontalOffset() * getSteerConstant();
        double maxDrivePower = getMaxDrivePower();
        if(steerCorrection > maxDrivePower) {
            steerCorrection = maxDrivePower;
        }
        return steerCorrection;
    }

    public void setDataOnDisplay() {
        SmartDashboard.putBoolean("Is Target Visible:", isTargetVisible());
        SmartDashboard.putNumber("Steer Correction: ", getSteerCorrection());
        SmartDashboard.putNumber("Target Area:", getTargetArea());
        SmartDashboard.putNumber("Horizontal Offset:", getHorizontalOffset());
        SmartDashboard.putNumber("Vertical Offset:", getVerticalOffset());
        SmartDashboard.putNumber("Skew:", getSkew());
        SmartDashboard.putNumber("LED State:", getLEDState().getNumber());
    }


}
