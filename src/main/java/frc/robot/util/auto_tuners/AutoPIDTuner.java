/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.auto_tuners;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SimplePID;
import frc.robot.util.TelemetryUtil;
import frc.robot.util.TelemetryUtil.PrintStyle;




/**
 * Add your docs here.
 */
public class AutoPIDTuner {

    public static class AutoPIDTunerConfig {
        public double runTimeout = 5.0;
        public double attempts = 20;

        public double minOutputLimit = -1.0;
        public double maxOutputLimit = 1.0;

        public double errorThreshold = 3.0;
        public double kP = 0.0001;
        public double kI = 0;
        public double kD = 0;

        public double[] setPointsToCheck = null;
    }

    private AutoTuneInterface autoTuneInterface;
    private AutoPIDTunerConfig config;
    private SimplePID pid;
    private double attemptCounter = 0;
    private Timer timer;

    private PrintWriter writer;

    private String fileDirectory = "//home//lvuser//deploy//AutoPIDTuneData.txt";

    
    public AutoPIDTuner(AutoTuneInterface autoTuneInterface, AutoPIDTunerConfig config){
        this.autoTuneInterface = autoTuneInterface;
        this.config = config;

        File dataFile = new File(fileDirectory);
        dataFile.setWritable(true);

        try {
            writer = new PrintWriter(dataFile);
        } catch (FileNotFoundException e) {
            TelemetryUtil.print("Auto PID file not found", PrintStyle.ERROR);
            e.printStackTrace();
        }


        DoubleSupplier pidSource = () -> autoTuneInterface.pidSource();

        pid = new SimplePID(pidSource, this.config.setPointsToCheck[0], this.config.kP, this.config.kI, this.config.kD);
        timer = new Timer();
    }

   

    public double[] getPID(double p_kP, double p_kI, double p_kD, double p_setPoint) {

        double[] newPID = {p_kP, p_kI, p_kD}; 

        if(!autoTuneInterface.resetRoutine()){
            TelemetryUtil.print("Auto PID Tuner Aborted: UNABLE TO RESET", PrintStyle.ERROR);
            return null;
        }

        autoTuneInterface.runSystem(0);

        pid.setOutputLimits(config.minOutputLimit, config.maxOutputLimit);
        pid.setkP(p_kP);
        pid.setkI(p_kI);
        pid.setkD(p_kD);
        pid.setSetpoint(p_setPoint);

        TelemetryUtil.print("Testing at values of kP = " + p_kP + "\tkI = " + p_kI + "\tkD = " + p_kD, PrintStyle.INFO);

        timer.reset();
        timer.start();
        
        boolean timedOut = false;
    
        
        while(pid.getError() > config.errorThreshold) {
            if(autoTuneInterface.abortCondition()) {
                autoTuneInterface.runSystem(0);
                TelemetryUtil.print("Auto PID Tuner aborted", PrintStyle.WARNING);
                return null;
            }

            if(timer.get() > config.runTimeout) {
                TelemetryUtil.print("PID Tuning attempt timed out", PrintStyle.INFO);
                timedOut = true;
                break;
            }

            double output = pid.compute();
            autoTuneInterface.runSystem(output);
            SmartDashboard.putNumber("Auto Tune - Output: ", output);
            SmartDashboard.putNumber("Auto Tune - Input: ", autoTuneInterface.pidSource());
            writer.println("c(t):" + autoTuneInterface.pidSource() + "t:" + timer.get()); 
        }

        autoTuneInterface.runSystem(0);

        if(timedOut) {
            if(hasOscillations()) {
                newPID = zieglerOscillationTuning();
            } else {
                double proportionDistCovered = 1/(pid.getError()/p_setPoint);
                newPID[0] *= proportionDistCovered;
            }
        } else {
            if(hasOscillations()) {
                newPID = zieglerOscillationTuning();
            } else {
                newPID = zieglerCurveTuning(p_setPoint);
            }  
        }

        if(attemptCounter > config.attempts) {
            TelemetryUtil.print("Auto PID Tuning Finished: kP = " + newPID[0] + "\tkI = " + newPID[1] + "\tkD = " + newPID[2], PrintStyle.INFO);
            return newPID;
        } else {
            return getPID(newPID[0], newPID[1], newPID[2], p_setPoint);
        }
    }



    private boolean hasOscillations(){
        return false;
    }

    private double[] zieglerOscillationTuning() {
        
        double[] returnArray = {0, 0, 0};
        return returnArray;
        
    }

    private double[] readLine(){

        BufferedReader reader;
        String line = null;
        String[] parts;

        try {
            reader = new BufferedReader(new FileReader(fileDirectory));
            line = reader.readLine();
            reader.close();
        } catch (FileNotFoundException e) {
            TelemetryUtil.print("Auto PID file not found", PrintStyle.ERROR);
            e.printStackTrace();
        } catch (IOException e) {
            TelemetryUtil.print("IO Exception in Auto PID tune", PrintStyle.ERROR);
            e.printStackTrace();
        }
        
        if(line != null) {
            line = line.split("c(t):")[1];
            parts = line.split("t:");

            double[] data =  {Double.parseDouble(parts[0]), Double.parseDouble(parts[1])};
            return data;
        } else {
            return null;
        }

       
    }

    private double[] zieglerCurveTuning(double p_setPoint){
        double[] data;
        double time, position;
        double prevTime, prevPosition;
        double inflection = 0, inflectionTime = 0, inflectionPosition = 0;
        double slope;

        data = readLine();
        prevPosition = data[0];
        prevTime = data[1];
        
        do {
            data = readLine();
            position = data[0];
            time = data[1];
            
            slope = ((position-prevPosition)/(time-prevTime));

            if(slope > inflection) { 
                inflection = slope;
                inflectionTime = time;
                inflectionPosition = position;
            } 
        } while(data != null && position < p_setPoint);

        double t = p_setPoint/inflection;
        double l = (inflectionPosition/inflection) - inflectionTime;

        double pid[] = {1.2 * (t/l), 2*l, 0.5*l};

        return pid;
    }



    public interface AutoTuneInterface {
        /**
         * Routine for resetting the mechanism before another trial
         * @return Whether the reset was successful
         */
        public boolean resetRoutine(); 
        
        /**
         * 
         * @return sensor input for pid loop
         */
        public double pidSource();

        /**
         * Method for running whatever system given power input computed
         */
        public void runSystem(double power);

        /**
         * 
         * @return Boolean state at which to abort Auto tuner for safety reasons
         */
        public boolean abortCondition();

    }
}
