package frc.robot.util;

import java.util.function.DoubleSupplier;

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
 */
public class SimplePID {

	private double outputMax = 1;
	private double outputMin = -1;
	private double output; // value to send to the motor
	/** The process error */
	private double error;
	/** The input of the PIDController */
	private DoubleSupplier pidsource;
	private double input; // what the value actually is
	private double setpoint; // desired target value
	private double prevInput;
	private double proportional; // P term
	private double integral; // I term
	private double derivative; // D term
	public double kp, ki, kd; // tuning parameters, the hardest part of PID
	private double prevTime = 0;
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
	public SimplePID(DoubleSupplier pidsource, double setpoint, double kp, double ki, double kd) {
		this.pidsource = pidsource;
		this.setpoint = setpoint;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
	}

	public double getOutput() {
		return output;
	}

	public void setkP(double kp) {
		this.kp = kp;
		resetPID();
	}

	public void setkI(double ki) {
		this.ki = ki;
		resetPID();
	}

	public void setkD(double kd){
		this.kd = kd;
		resetPID();
	}

	public static double map(double oldValue, double oldMin, double oldMax, double newMin, double newMax) {
		return (newMax - newMin) * (oldValue - oldMin) / (oldMax - oldMin) + newMin;
	}

	public double getInput() {
		return pidsource.getAsDouble();
	}

	public double getError() {
		return error;
	}


	public double getSetpoint() {
		return setpoint;
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		resetPID();
	}
	public void setSetpointRamp(double setpoint) {
		this.setpoint = setpoint;
	}
	public void setOutputLimits(double min, double max) {
		outputMax = max;
		outputMin = min;
	}



	public void resetPID() {
		proportional = 0;
		integral = 0;
		derivative = 0;
	}

	public double compute() {
		if (pidsource == null) {
			return -216;
		}

		double currentTime = Timer.getFPGATimestamp();
		double dt = (currentTime - prevTime);

		input = pidsource.getAsDouble();
		error = setpoint - input; 
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
		output = proportional + integral - derivative;

		// constrains output in between outputMin and outputMax
		if (output > outputMax) {
			output = outputMax;
		}
		if (output < outputMin) {
			output = outputMin;
		}

		prevInput = input;
		prevTime = currentTime;

		return output;
	}
}
