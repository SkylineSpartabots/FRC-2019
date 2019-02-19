package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * PID Controller Object
 * <p>
 * Starts a runnable class PIDCompute that executes the PID algorithm
 * <p>
 * Method Signature: public PIDMain(Object pidsource,int setpoint, int
 * sampleTime, double kp, double ki, double kd)
 * 
 * @author NeilHazra
 * @author Mustafa
 * @author Vibhav Peri
 */
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
	private double derivative; // D term
	public double kp, ki, kd; // tuning parameters, the hardest part of PID
	private double prevTime = 0;
	private Logger PIDlog;
	private boolean log;

	/**
	 * 
	 * @param pidsource  Object implementing PIDSource, contains method returning
	 *                   input
	 * @param setpoint   target value for PID controller
	 * @param sampleTime time between successive calculations
	 * @param kp         proportional gain
	 * @param ki         integral gain
	 * @param kd         derivative gain
	 */
	public SimplePID(Object pidsource, double setpoint, double kp, double ki, double kd, boolean b) {
		this.pidsource = (PIDSource) pidsource;
		this.setpoint = setpoint;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.log = b;

		if (b) {
			try {
				PIDlog = new Logger(b + ".txt");
			} catch (NullPointerException ex) {
				System.out.println("SimplePID File Exception: Log Enabled without Valid FilePath" + ex.getMessage());
				this.log = false;
			}
		}

		if (this.log) {
			PIDlog.writeNewData("Vernier Format 2");
			PIDlog.writeNewData("Vernier Format 2");
			PIDlog.writeNewData("Untiled.clmb 5/5/2019 9:37:43 .");
			PIDlog.writeNewData("Data Set");
			PIDlog.writeNewData("Time	Input	Output	Error");
			PIDlog.writeNewData("x	y	z	w\n");
			PIDlog.flushLogData();
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
	 * @param setpoint desired target value
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
	 * PID Algorithm calculates
	 * 
	 * @author NeilHazra
	 *
	 */
	public void writeNewData(double seconds, double input, double output, double error) {
		PIDlog.writeNewData("" + seconds + "\t" + input + "\t" + output + "\t" + error);
	}

	public void flushLogData() {
		if (this.log)
			PIDlog.flushLogData();
	}

	public void resetPID() {
		proportional = 0;
		integral = 0;
		derivative = 0;

		if (log) {
			flushLogData();
		}
	}

	public double compute() {
		if (pidsource == null) {
			return -216;
		}

		double currentTime = Timer.getFPGATimestamp();
		double dt = (currentTime - prevTime);

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

		if (log) {
			writeNewData(currentTime, input, output, error);
		}

		return output;
	}
}