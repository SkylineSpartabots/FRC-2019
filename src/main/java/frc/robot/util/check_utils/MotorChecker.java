/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.check_utils;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.util.TelemetryUtil;
import frc.robot.util.TelemetryUtil.PrintStyle;


/**
 * This class simplifies and groups motors into a test routine which varifies 
 * current and optional rpm of each motor
 */
public class MotorChecker {
    
   
    /**
     * Class for storing basic information about the motor controller and casts it appropriatley
     * to the correct controller type
     */
    public static class ControllerConfig {
       
        public SpeedController motorController;
        public DoubleSupplier currentSupplier, rpmSupplier;
        public String name;

        public ControllerConfig(SpeedController motorController, String name, DoubleSupplier currentSupplier, DoubleSupplier rpmSupplier) {
            this.motorController = motorController;
            this.name = name;
            this.currentSupplier = currentSupplier;
            this.rpmSupplier = rpmSupplier;
        }
    }

    /**
     * Class of information that the motor checker is going to check the motors against
     */
    public static class CheckerConfig {
        public double minCurrent = 3;
        public double currentBuffer = 3;

        public double minRPM = 2000;
        public double rpmBuffer = 500;

        public double runSecs = 3.0;
        public double waitSecs = 1.0;
        public double outputValue = 0.5;
    }

    /**
     * 
     * @param subsystem
     * @param controllersToCheck
     * @param checkerConfig
     * @return returns boolean of success or failure by checking all motors bu running them and comparing to specified values in 
     * checker config
     */
    public static boolean checkMotors(Subsystem subsystem, ArrayList<ControllerConfig> controllersToCheck, CheckerConfig checkerConfig) {
        TelemetryUtil.print("Checking " + controllersToCheck.size() + " talons in subsystem " + subsystem.getName(), PrintStyle.INFO);
        boolean failure = false;


        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();

        for(ControllerConfig config : controllersToCheck){
            TelemetryUtil.print("Checking: " + config.name, PrintStyle.NONE);
            config.motorController.set(checkerConfig.outputValue);

            Timer.delay(checkerConfig.runSecs);

            double current = Double.NaN;
            if(config.currentSupplier != null){
                current = config.currentSupplier.getAsDouble();
                currents.add(current);
                TelemetryUtil.print("Current: " + current, PrintStyle.NONE);

                if(current < checkerConfig.minCurrent) {
                    TelemetryUtil.print("Failed to reach min current threshold", PrintStyle.WARNING);
                    failure = true;
                }
            }
            
            
            double rpm = Double.NaN;
            if(config.rpmSupplier != null) {
                rpm = config.rpmSupplier.getAsDouble();
                rpms.add(rpm);
                TelemetryUtil.print("RPM: " + rpm, PrintStyle.NONE);

                if(rpm < checkerConfig.minRPM) { 
                    TelemetryUtil.print("Failed to reach min rpm threshold", PrintStyle.WARNING);
                    failure = true;
                }
            }

            config.motorController.set(0.0);

            Timer.delay(checkerConfig.waitSecs);
        }

        if(currents.size() > 0) {
            double currentsAverage = currents.stream().mapToDouble(num -> num).average().getAsDouble();
            if(!Equivalency.allAboutEqualTo(currents, currentsAverage, checkerConfig.currentBuffer)){
                TelemetryUtil.print("Currents varied beyond parameters", PrintStyle.WARNING);
                failure = true;
            }
        }   

        if(rpms.size() > 0){
            double rpmsAverage = rpms.stream().mapToDouble(num -> num).average().getAsDouble();
            if(!Equivalency.allAboutEqualTo(rpms, rpmsAverage, checkerConfig.rpmBuffer)){
                TelemetryUtil.print("RPMs varied beyond parameters", PrintStyle.WARNING);
                failure = true;
            }
        }

        return !failure;

    }

    /**
     * @return telemetry for Spark Max controller
     * @param sparkMax
     * @param name
     */
    public void getSparkTelemetry(CANSparkMax sparkMax, String name) {
        TelemetryUtil.print("Telemetry for spark controller: " + name, PrintStyle.INFO);
        TelemetryUtil.print("\tBus Voltage: " + sparkMax.getBusVoltage(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tOutput Current: " + sparkMax.getOutputCurrent(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tWritten Value: " + sparkMax.get(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tIs Inverted: " + sparkMax.getInverted(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tIs Follower: " + sparkMax.isFollower(), PrintStyle.SENSOR_VALUE);

        String motorType = sparkMax.getMotorType().equals(MotorType.kBrushed) ? "\tMotor Type: Brushed" : "\tMotor Type: Brushless";
        TelemetryUtil.print(motorType, PrintStyle.SENSOR_VALUE);
            
        TelemetryUtil.print("\tDevice ID: " + sparkMax.getDeviceId(), PrintStyle.SENSOR_VALUE);
    }

    
    /**
     * @return telemtry for talon controller
     * @param talon
     * @param name
     */
    public void getTalonTelemetry(WPI_TalonSRX talon, String name){
        TelemetryUtil.print("Telemetry for talon controller: " + name, PrintStyle.INFO);
        TelemetryUtil.print("\tBus Voltage: " + talon.getBusVoltage(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tApplied Output: " + talon.get(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tOutput Current: " + talon.getOutputCurrent(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tIs Inverted: " + talon.getInverted(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tDevice ID: " + talon.getDeviceID(), PrintStyle.SENSOR_VALUE);
    }

    /**
     * @return telemetry for victor controller
     * @param victor
     * @param name
     */
    public void getVictorTelemetry(WPI_VictorSPX victor, String name) {
        TelemetryUtil.print("Telemetry for victor controller: " + name, PrintStyle.INFO);
        TelemetryUtil.print("\tBus Voltage: " + victor.getBusVoltage(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tApplied Output: " + victor.get(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tIs Inverted: " + victor.getInverted(), PrintStyle.SENSOR_VALUE);
        TelemetryUtil.print("\tDevice ID: " + victor.getDeviceID(), PrintStyle.SENSOR_VALUE);
    }
}
