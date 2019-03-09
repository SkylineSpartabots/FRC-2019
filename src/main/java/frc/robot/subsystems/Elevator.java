package frc.robot.subsystems;



import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive_controls.*;

/**
 * Subsystem for the elevator.
 */
public class Elevator extends Subsystem {

	private WPI_TalonSRX rightElevatorMotor, leftElevatorMotor;
	public Encoder elevatorEncoder;
	private DigitalInput elevatorLimitSwitch;
	

	public final static int MAX_ENCODER_LIMIT = 1132; // TODO: Add limit
	public final static int MIN_ENCODER_LIMIT = 5;
	public final static int amps = 15;
	public final static int timeoutMs = 5000;

	public final static double MAX_STALL_POWER = 0.07;

	private NetworkTableEntry kP, kI, kD, stallPower;
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

		kP = TAB.add("Elevator kP", 0.028).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		kI = TAB.add("Elevator kI", 0.002).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		kD = TAB.add("Elevator kD", 0.001).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		stallPower = TAB.add("Elevator Stall Power", 0.07).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
	}

	/**
	 * 
	 * @return kp, ki, kd, stallPower
	 */
	public double[] getElevatorConstants(){
		double[] constants = {kP.getDouble(0.001), kI.getDouble(0.0001), kD.getDouble(0.0001), stallPower.getDouble(0.001)};
		return constants;
	}
	
	public int getElevatorEncoderOutput()	{
		return elevatorEncoder.get();
	}



	/**
	 * Sets the power to the motor. Takes in consideration of the current elevator
	 * position and resets the encoders when the limit switch is active
	 * 
	 * @param power power <= 0
	 */
	public void setStallPower(double stallPower)	{
		setRawPower(stallPower);
	}
	private void setRawPower(double power) {
		rightElevatorMotor.set(power);
		leftElevatorMotor.set(power);
	}

	public void setPower(double power) {
		System.out.println(power);
		boolean maxReached = getElevatorEncoderOutput() >= MAX_ENCODER_LIMIT && power > 0;
		boolean minReached = getElevatorEncoderOutput() <= MIN_ENCODER_LIMIT && power < 0;
		if (maxReached) {
			setStallPower(getElevatorConstants()[3]);
		} else if(minReached)	{
			setRawPower(0);
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
		//setDefaultCommand(new ElevatorControl());
	}

	public void setElevatorDataOnDisplay() {
		SmartDashboard.putNumber("Elevator Encoder Count", getElevatorEncoderOutput());
		SmartDashboard.putBoolean("Elevator Limit Switch", getLimitSwitchState());

		SmartDashboard.putNumber("Left Elevator Motor Output Current", leftElevatorMotor.getOutputCurrent());
		SmartDashboard.putNumber("Right Elevator Motor Output Current", rightElevatorMotor.getOutputCurrent());

		SmartDashboard.putNumber("Left Elevator Motor Output Voltage", leftElevatorMotor.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Elevator Motor Output Voltage", rightElevatorMotor.getMotorOutputVoltage());
	}

	/**
	 * Enum for storing and getting the values for the encoder values for the 8
	 * different elevator positions
	 * TODO: NEED TO ADD ACTUAL VALUES
	 */
	public enum ElevatorPosition {
		DOWN(0,0), CARGO_SHIP(557, 557), ROCKET_FIRST(360, 360), ROCKET_SECOND(790, 480), ROCKET_THIRD(1125, 900);

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