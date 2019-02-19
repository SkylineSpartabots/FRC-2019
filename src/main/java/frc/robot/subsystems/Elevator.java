package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorControl;

/**
 * Subsystem for the elevator.
 */
public class Elevator extends Subsystem {

	private WPI_TalonSRX elevatorMaster, elevatorSlave;
	public Encoder elevatorEncoder;
	private DigitalInput elevatorLimitSwitch;

	public final static int MAX_ENCODER_LIMIT = 1400; // TODO: Add limit
	public final static int MIN_ENCODER_LIMIT = 10;

	public Elevator() {
		elevatorMaster = new WPI_TalonSRX(RobotMap.RIGHT_ELEVATOR);
		elevatorSlave = new WPI_TalonSRX(RobotMap.LEFT_ELEVATOR);

		elevatorMaster.setNeutralMode(NeutralMode.Brake);
		elevatorSlave.setNeutralMode(NeutralMode.Brake);

		elevatorMaster.setInverted(false);
		elevatorSlave.setInverted(false);

		elevatorSlave.follow(elevatorMaster);

		elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_PORT_A, RobotMap.ELEVATOR_ENCODER_PORT_B);
		elevatorLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);
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
	public void setRawPower(double power) {
		
		elevatorMaster.set(power);

		// reset encoder to 0 if elevator limit switch is pressed
		if (getLimitSwitchState()) {
			elevatorEncoder.reset();
		}
	}

	public void setPower(double power) {
		boolean maxReached = elevatorEncoder.getDistance() >= MAX_ENCODER_LIMIT && power > 0;
		boolean minReached = elevatorEncoder.getDistance() <= MIN_ENCODER_LIMIT && power < 0;

		if (maxReached || minReached) {
			elevatorMaster.set(0);
		} else {
			elevatorMaster.set(power);
		}

		if (getLimitSwitchState()) {
			elevatorEncoder.reset();
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

	/**
	 * Enum for storing and getting the values for the encoder values for the 8
	 * different elevator positions
	 * TODO: NEED TO ADD ACTUAL VALUES
	 */
	public enum ElevatorPosition {
		CARGO_SHIP(0, 0), ROCKET_FIRST(0, 0), ROCKET_SECOND(0, 0), ROCKET_THIRD(0, 0);

		private final int cargoPosition;
		private final int hatchPosition;

		private ElevatorPosition(int cargoPosition, int hatchPosition) {
			this.cargoPosition = cargoPosition;
			this.hatchPosition = hatchPosition;
		}

		/**
		 * Returns the cargo position if cargo is loaded into the intake, else the hatch position
		 * @return position to put the elevator at
		 */
		public int getPosition() {
			return (Robot.intake.isCargo() ? cargoPosition : hatchPosition);
		}
	}

}