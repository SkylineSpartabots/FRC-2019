package frc.robot.util;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.Date;

/**
 * PID Controller Object
 * <p>
 * Starts a runnable class PIDCompute that executes the PID algorithm
 * <p>
 * Method Signature: public PIDMain(Object pidsource,int setpoint, int
 * sampleTime, double kp, double ki, double kd)

 * @author NeilHazra
 * @author Mustafa 
 * @author Vibhav Peri 
 */

// The heart of PID
public class SimplePID {
	private double outputMax = 1;
	private double outputMin = -1;
	private PIDSource pidsource;
	private double output; // value to send to the motor
	/** The process error */
	private double error;
	/** The input of the PIDController */
	private double input; // what the value actually is
	private double setpoint; // desired target value
	private double prevInput;
	private double proportional; // P term
	private double integral; // I term
	private double derivative;// D term
	public double kp, ki, kd; // tuning parameters, the hardest part of PID
	private double prevTime = 0;
	private PrintWriter writer;
	private boolean log;
	/**
	 * @param pidsource
	 *            Object implementing PIDSource, contains method returning input
	 * @param setpoint
	 *            target value for PID controller
	 * @param sampleTime
	 *            time between successive calculations
	 * @param kp
	 *            proportional gain
	 * @param ki
	 *            integral gain
	 * @param kd
	 *            derivative gain
	 */
	public SimplePID(Object pidsource, double setpoint, double kp, double ki, double kd, boolean log, String filename) {
		this.pidsource = (PIDSource) pidsource;
		this.setpoint = setpoint;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.log = log;			
		try{
			File f = new File("//home//lvuser//deploy//" + filename + ".txt");
			f.createNewFile();
			f.setWritable(true);
			writer = new PrintWriter(f);
		}	catch (IOException | SecurityException  | NullPointerException ex){
			System.out.println("SimplePID File Exception: " + ex.getMessage());
			this.log = false; 
		}
		if(log)	{
			writer.println("Vernier Format 2");
			writer.println("Untiled.clmb 5/5/2019 9:37:43 .");
			writer.println("Data Set");
			writer.println("Time	Input	Output	Error");
			writer.println("x	y	z	w\n");
			writer.flush();
		}
	}


	public double getOutput() {
		return output;
	}
	public static double map(double oldValue, double oldMin, double oldMax, double newMin, double newMax) {
		return (newMax - newMin) * (oldValue - oldMin) / (oldMax - oldMin) + newMin;
	}
	public double getInput() {
		return pidsource.getInput();
	}

	public double getError() {
		return error;
	}
	/**
	 * @return the desired target value
	 */
	public double getSetpoint() {
		return setpoint;
	}
	/**
	 * Sets the desired target value
	 * 
	 * @param setpoint
	 *            desired target value
	 */
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		resetPID();
	}
	public void setOutputLimits(double min, double max) {
		outputMax = max;
		outputMin = min;
	}
	/**
	 * PID Algorithm calculates in this TimerTask created by PIDMain
	 * 
	 * @author NeilHazra
	 *
	 */
	public void writeNewData(double seconds, double input, double output, double error) {
		writer.println("" + seconds + "\t" + input + "\t" + output + "\t" + error);
	}
	public void flushLogData(){
		if(this.log = true)	writer.flush();	
	}
	 public void resetPID() {
		proportional = 0;
		integral = 0;
		derivative = 0;
		flushLogData();
	}

	public double compute()	{
		long currentTime = System.currentTimeMillis();
		double dt = (currentTime - prevTime)/1000.0;
		input = pidsource.getInput();
		error = input - setpoint;
		proportional = kp * error;
		integral += ki * error;
		// constrains integral in between outputMin and outputMax
		if (integral > outputMax) {
			integral = outputMax;
		}
		if (integral < outputMin) {
			integral = outputMin;
		}
		derivative = kd * (input - prevInput) / dt;
		output = proportional + integral + derivative;
		// constrains output in between outputMin and outputMax
		if (output > outputMax) {
			output = outputMax;
		}
		if (output < outputMin) {
			output = outputMin;
		}
		prevInput = input;
		prevTime = currentTime;
		if(log)	{
			writeNewData(currentTime, input, output, error);
		}
		return output;
	}	
}