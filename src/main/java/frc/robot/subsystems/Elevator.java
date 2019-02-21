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
import frc.robot.commands.ElevatorControl;

/**
 * Subsystem for the elevator.
 */
public class Elevator extends Subsystem {

	private WPI_TalonSRX rightElevatorMotor, leftElevatorMotor;
	public Encoder elevatorEncoder;
	private DigitalInput elevatorLimitSwitch;

	public final static int MAX_ENCODER_LIMIT = 1000;
	public final static int MIN_ENCODER_LIMIT = 10;
	public final static int amps = 15;
	public final static int timeoutMs = 5000;

	private ShuffleboardTab TAB = Shuffleboard.getTab("SmartDashboard");
	private static NetworkTableEntry kP, kI, kD;

	public Elevator() {
		rightElevatorMotor = new WPI_TalonSRX(RobotMap.RIGHT_ELEVATOR);
		leftElevatorMotor = new WPI_TalonSRX(RobotMap.LEFT_ELEVATOR);

		rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
		leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
		rightElevatorMotor.configContinuousCurrentLimit(amps, timeoutMs);
		leftElevatorMotor.configContinuousCurrentLimit(amps, timeoutMs);

		rightElevatorMotor.setInverted(true);
		leftElevatorMotor.setInverted(true);

		elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_PORT_A, RobotMap.ELEVATOR_ENCODER_PORT_B);
		elevatorLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);

		kP = TAB.add("Elevator Proportional Constant", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		
		kI = TAB.add("Elevator Proportional Constant", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();

		kD = TAB.add("Elevator Proportional Constant", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();

	}

	public double[] getPIDElevatorConstants(){
		double[] pid = {kP.getDouble(0.001), kI.getDouble(0.001), kD.getDouble(0.001)};
		return pid;
	}
	public int getElevatorEncoderOutput() {
		return elevatorEncoder.get();
	}

	/**
	 * Sets the power to the motor. Takes in consideration of the current elevator
	 * position and resets the encoders when the limit switch is active
	 * 
	 * @param power power <= 0
	 */
	public void setStallPower() {
		setRawPower(0.10);
	}

	private void setRawPower(double power) {
		rightElevatorMotor.set(power);
		leftElevatorMotor.set(power);
	}

	public void setPower(double power) {
		boolean maxReached = getElevatorEncoderOutput() >= MAX_ENCODER_LIMIT && power > 0;
		boolean minReached = getElevatorEncoderOutput() <= MIN_ENCODER_LIMIT && power < 0;

		if (maxReached) {
			setStallPower();
		} else if (minReached) {
			setRawPower(0);
		} else {
			setRawPower(power);
		}

		if (getLimitSwitchState()) {
			elevatorEncoder.reset();
		}
	}

	/**
	 * Returns the state of the limit switch
	 * 
	 * @return true is limit switch is active, else false
	 */
	public boolean getLimitSwitchState() {
		return !elevatorLimitSwitch.get();
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ElevatorControl());
	}

	/**
	 * Enum for storing and getting the values for the encoder values for the 8
	 * different elevator positions TODO: NEED TO ADD ACTUAL VALUES
	 */
	public enum ElevatorPosition {
		CARGO_SHIP(0, 0), ROCKET_FIRST(0, 0), ROCKET_SECOND(480, 480), ROCKET_THIRD(900, 900);

		private final int cargoPosition;
		private final int hatchPosition;

		private ElevatorPosition(int cargoPosition, int hatchPosition) {
			this.cargoPosition = cargoPosition;
			this.hatchPosition = hatchPosition;
		}

		/**
		 * Returns the cargo position if cargo is loaded into the intake, else the hatch
		 * position
		 * 
		 * @return position to put the elevator at
		 */
		public int getPosition() {
			return (Robot.intake.getCargoState() ? cargoPosition : hatchPosition);
		}
	}

	public void setElevatorDataOnDisplay() {
		SmartDashboard.putNumber("Elevator Encoder Count", getElevatorEncoderOutput());
		SmartDashboard.putBoolean("Elevator Limit Switch", getLimitSwitchState());

		SmartDashboard.putNumber("Left Elevator Motor Output Current", leftElevatorMotor.getOutputCurrent());
		SmartDashboard.putNumber("Right Elevator Motor Output Current", rightElevatorMotor.getOutputCurrent());

		SmartDashboard.putNumber("Left Elevator Motor Output Voltage", leftElevatorMotor.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Elevator Motor Output Voltage", rightElevatorMotor.getMotorOutputVoltage());
	}
}
