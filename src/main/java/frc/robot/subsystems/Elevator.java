package frc.robot.subsystems;



import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive_controls.ElevatorControl;

/**
 * Subsystem for the elevator.
 */
public class Elevator extends Subsystem {

	private WPI_TalonSRX rightElevatorMotor, leftElevatorMotor;
	public Encoder elevatorEncoder;
	private DigitalInput elevatorLimitSwitch;
	

	public final static int MAX_ENCODER_LIMIT = 1180;
	public final static int MIN_ENCODER_LIMIT = -5;
	public final static int amps = 15;
	public final static int timeoutMs = 5000;


	private NetworkTableEntry kP, kI, kD, stallPowerMin, stallPowerMax;
	private double slope, stallPowerMinVal, stallPowerMaxVal;
	private static final ShuffleboardTab TAB = Shuffleboard.getTab("ElevatorConstants");



	public Elevator() {
		rightElevatorMotor = new WPI_TalonSRX(RobotMap.RIGHT_ELEVATOR);
		leftElevatorMotor = new WPI_TalonSRX(RobotMap.LEFT_ELEVATOR);

		rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
		leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
		rightElevatorMotor.configContinuousCurrentLimit(amps, timeoutMs);
		leftElevatorMotor.configContinuousCurrentLimit(amps, timeoutMs);

		rightElevatorMotor.setInverted(false);
		leftElevatorMotor.setInverted(false);


		elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_PORT_A, RobotMap.ELEVATOR_ENCODER_PORT_B);
		elevatorLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);

		kP = TAB.add("Elevator kP", 0.016).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		kI = TAB.add("Elevator kI", 0.004).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		kD = TAB.add("Elevator kD", 0.002).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		stallPowerMin = TAB.add("Min Stall Power", 0.1).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 1)).getEntry();
		stallPowerMax = TAB.add("Max Stall Power", 0.15).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 1)).getEntry();
	}

	/**
	 * 
	 * @return kp, ki, kd
	 */
	public double[] getElevatorConstants(){
		double[] constants = {kP.getDouble(0.001), kI.getDouble(0.0001), kD.getDouble(0.0001)};
		return constants;
	}
	
	public int getElevatorEncoderOutput()	{
		return elevatorEncoder.get();
	}

	
	/**
	 * 
	 * @return generates stall power using through a linear relationship
	 */
	public double getStallPower(){
		stallPowerMinVal = stallPowerMin.getDouble(0.001);
		stallPowerMaxVal = stallPowerMax.getDouble(0.001);
		slope = (stallPowerMaxVal - stallPowerMinVal)/MAX_ENCODER_LIMIT;
		if(getElevatorEncoderOutput() < 50){
			return -0.1;	
		} else {
			return stallPowerMinVal + (slope * getElevatorEncoderOutput());
		}
		
	}


	
	public void setStallPower()	{
		setPower(getStallPower());
	}
	private void setRawPower(double power) {
		rightElevatorMotor.set(power);
		leftElevatorMotor.set(power);
	}

	/**
	 * Sets the power to the motor. Takes in consideration of the current elevator
	 * position and resets the encoders when the limit switch is active
	 * 
	 * @param power power <= 0
	 */
	public void setPower(double power) {
		boolean maxReached = getElevatorEncoderOutput() >= MAX_ENCODER_LIMIT && power > 0;
		boolean minReached = (getElevatorEncoderOutput() <= MIN_ENCODER_LIMIT || getLimitSwitchState()) && power < 0;
		
		if(getLimitSwitchState()) {
			elevatorEncoder.reset();
		}
		
		if (minReached) {
			setRawPower(0);
		} else if(maxReached)	{
			setRawPower(getStallPower());
		} 	else {
			setRawPower(power);
		}

	}

	/**
	 * Returns the state of the limit switch
	 * @return true is limit switch is active, else false
	 */
	public boolean getLimitSwitchState() {
		return !elevatorLimitSwitch.get();
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ElevatorControl());
	}

	public void setElevatorDataOnDisplay() {
		SmartDashboard.putNumber("Elevator Encoder Count", getElevatorEncoderOutput());
		SmartDashboard.putBoolean("Elevator Limit Switch", getLimitSwitchState());

		SmartDashboard.putNumber("Left Elevator Motor Output Current", leftElevatorMotor.getOutputCurrent());
		SmartDashboard.putNumber("Right Elevator Motor Output Current", rightElevatorMotor.getOutputCurrent());

		SmartDashboard.putNumber("Left Elevator Motor Output Voltage", leftElevatorMotor.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Elevator Motor Output Voltage", rightElevatorMotor.getMotorOutputVoltage());
		SmartDashboard.putNumber("Left Elevator Set Power", leftElevatorMotor.get());
		SmartDashboard.putNumber("Stall Power", getStallPower());
	}

	public boolean checkSubsystem() {
		System.out.println("\n\n\n\nTesting Elevator..........................");
		elevatorEncoder.reset();
		boolean motorFailure = false;
		boolean encoderFailure = false;
		boolean limitSwitchFailure = false;
		boolean stallFailure = false;
		boolean beamBreakElevatorFailure = false;

		System.out.println("\n\nTesting elevator motors and encoder.......................");
		double kCurrentThreshold = 0.5; //TODO Add correct threshold
		int minEncoderVal = 300; // TODO ADD correct value
		int kElevatorStallThreshold = 20;
		double preStallEncoderValue;
		
		setRawPower(0.3);
		Timer.delay(0.5);
		double leftMotorCurrent = leftElevatorMotor.getOutputCurrent();
		double rightMotorCurrent = rightElevatorMotor.getOutputCurrent();

		if(leftMotorCurrent < kCurrentThreshold){
			motorFailure = true;
			System.out.println("!!!!!!!! FAILURE: LEFT ELEVATOR MOTOR CURRENT IS LOW !!!!!!!!");
		}

		if(rightMotorCurrent < kCurrentThreshold){
			motorFailure = true;
			System.out.println("!!!!!!!! FAILURE: RIGHT ELEVATOR MOTOR CURRENT IS LOW !!!!!!!!");
		}

		if(Math.abs(leftMotorCurrent - rightMotorCurrent) > kCurrentThreshold){
			motorFailure = true;
			System.out.println("!!!!!!!! FAILURE: ELEVATOR MOTORS AT DIFFERENT CURRENT !!!!!!!!");
		}

		if(!motorFailure){
			System.out.println("######## SUCCESSFUL: GO FOR ELEVATOR MOTORS ########");
		}


		if(getElevatorEncoderOutput() < minEncoderVal){
			encoderFailure = true;
			System.out.println("!!!!!!!! FAILURE: ELEVATOR ENCODER IS NOT COUNTING !!!!!!!!");
		} else {
			System.out.println("######## SUCCESSFUL: GO FOR ELEVATOR ENCODER ########");
			System.out.println("\n\nTesting elevator stall.................");
			setStallPower();
			Timer.delay(0.5);
			preStallEncoderValue = getElevatorEncoderOutput();
			Timer.delay(2);
			if(Math.abs(preStallEncoderValue - getElevatorEncoderOutput()) > kElevatorStallThreshold){
				System.out.println("!!!!!!!! FAILURE: UNABLE TO STALL AND MAINTAIN POSITION !!!!!!!!");
				stallFailure = true;
			} else {
				System.out.println("######## SUCCESSFUL: GO FOR ELEVATOR STALL ########");
			}
		}

		System.out.println("\n\nTesting elevator limit switch...............");
		if(getLimitSwitchState()){
			limitSwitchFailure = true;
			System.out.println("!!!!!!!! FAILURE: ELEVATOR LIMIT SWITCH THINKS ITS ACTIVE WHEN ELEVATOR IS UP !!!!!!!!");
		}

		setPower(-0.08);	
		if(!limitSwitchFailure){
			for(int i = 0; i < 6; i++){
				Timer.delay(1);
				if(getLimitSwitchState()){
					System.out.println("######## SUCCESSFUL: GO FOR ELEVATOR LIMIT SWITCH ########");
					limitSwitchFailure = false;
					break;
				} else {
					limitSwitchFailure = true;
				}
			}

			if(limitSwitchFailure){
				System.out.println("!!!!!!!! FAILURE: ELEVATOR LIMIT SWITCH FAILED TO ACTIVATE !!!!!!!!");
			}
			
		} else {
			Timer.delay(6);
		}

		System.out.println("\n\nTesting beam break with elevator down...............");
		if(Robot.intake.getRawCargoPosition()){
			beamBreakElevatorFailure = true;
			System.out.println("!!!!!!!! FAILURE: POSSIBLE THAT ELEVATOR IS IN THE WAY OF THE BEAM BREAK !!!!!!!!");
		} else {
			System.out.println("######## SUCCESSFUL: GO FOR ELEVATOR WITH BEAM BREAK ########");
		}

		return !motorFailure && !encoderFailure && !stallFailure && !limitSwitchFailure && !beamBreakElevatorFailure;

	}


	/**
	 * Enum for storing and getting the values for the encoder values for the 8
	 * different elevator positions
	 * TODO: NEED TO ADD ACTUAL VALUES
	 */
	public enum ElevatorPosition {
		DOWN(50, 50), CARGO_SHIP(557, 557), ROCKET_FIRST(360, 360), ROCKET_SECOND(750, 465), ROCKET_THIRD(1135, 885);

		public final int cargoPosition;
		public final int hatchPosition;

		private ElevatorPosition(int cargoPosition, int hatchPosition) {
			this.cargoPosition = cargoPosition;
			this.hatchPosition = hatchPosition;
		}

		/**
		 * Returns the cargo position if cargo is loaded into the intake, else the hatch position
		 * @return position to put the elevator at
		 */
		public int getPosition() {
			return Robot.intake.isCargo() ? cargoPosition : hatchPosition;
		}
	}

}