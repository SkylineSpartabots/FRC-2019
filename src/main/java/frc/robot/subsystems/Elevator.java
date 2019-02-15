package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorControl;

public class Elevator extends Subsystem {

	private WPI_TalonSRX elevatorMaster, elevatorSlave;
	public Encoder elevatorEncoder;
	private DigitalInput elevatorLimitSwitch;

	public final static int MAX_ENCODER_LIMIT = 1000; // TODO: Add limit
	public final static int MIN_ENCODER_LIMIT = 0;

	public Elevator() {
		elevatorMaster = new WPI_TalonSRX(RobotMap.rightElevatorPort);
		elevatorSlave = new WPI_TalonSRX(RobotMap.leftElevatorPort);

		elevatorMaster.setNeutralMode(NeutralMode.Brake);
		elevatorSlave.setNeutralMode(NeutralMode.Brake);

		elevatorMaster.setInverted(false);
		elevatorSlave.setInverted(false);

		elevatorSlave.follow(elevatorMaster);

		elevatorEncoder = new Encoder(RobotMap.elevatorEncoderPorts[0], RobotMap.elevatorEncoderPorts[1]);
		elevatorLimitSwitch = new DigitalInput(RobotMap.elevatorLimitSwitch);
	}

	/**
	 * Sets the power to the motor. Takes in consideration of the current elevator
	 * position and resets the encoders when the limit switch is active
	 */
	public void setPower(double power) {
		boolean maxReached = elevatorEncoder.getDistance() >= MAX_ENCODER_LIMIT && power > 0;
		boolean minReached = elevatorEncoder.getDistance() <= MIN_ENCODER_LIMIT && power < 0;

		if (!maxReached || !minReached) {
			elevatorMaster.set(power);
		} else {
			elevatorMaster.set(0);
		}

		if (getLimitSwitchState()) {
			elevatorEncoder.reset();
		}
	}

	/**
	 * @return returns the state of limit switch. True is active.
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
	public enum ElevatorPositions {
		CARGO_SHIP(0, 0), ROCKET_FIRST(0, 0), ROCKET_SECOND(0, 0), ROCKET_THIRD(0, 0);

		private final int cargoPosition;
		private final int hatchPosition;

		private ElevatorPositions(int cargoPosition, int hatchPosition) {
			this.cargoPosition = cargoPosition;
			this.hatchPosition = hatchPosition;
		}

		public int getPosition() {
			if (Robot.intake.isCargo()) {
				return this.cargoPosition;
			} else {
				return this.hatchPosition;
			}
		}
	}

}