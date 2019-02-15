package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeControl;

public class Intake extends Subsystem {

	WPI_TalonSRX masterIntakeMotor, slaveIntakeMotor, innerIntakeMotor;
	Solenoid masterIntakeSolenoid, slaveIntakeSolenoid;
	AnalogInput distanceSensor;

	public Intake() {
		masterIntakeMotor = new WPI_TalonSRX(RobotMap.rightIntakeMotor);
		slaveIntakeMotor = new WPI_TalonSRX(RobotMap.leftIntakeMotor);

		masterIntakeMotor.setNeutralMode(NeutralMode.Brake);
		slaveIntakeMotor.setNeutralMode(NeutralMode.Brake);

		masterIntakeMotor.setInverted(false); // TODO: set directions
		slaveIntakeMotor.setInverted(false);

		slaveIntakeMotor.follow(masterIntakeMotor);

		innerIntakeMotor = new WPI_TalonSRX(RobotMap.innerIntakeMotor);
		innerIntakeMotor.setInverted(false);

		distanceSensor = new AnalogInput(RobotMap.intakeSensorPort);

		masterIntakeSolenoid = new Solenoid(RobotMap.rightIntakeSolenoid);
		slaveIntakeSolenoid = new Solenoid(RobotMap.leftIntakeSolenoid);
	}

	public void extendIntake() {
		masterIntakeSolenoid.set(true);
		slaveIntakeSolenoid.set(true);
	}

	public void retractIntake() {
		masterIntakeSolenoid.set(false);
		slaveIntakeSolenoid.set(false);
	}

	public boolean getIntakeSolenoidState() {
		return masterIntakeSolenoid.get();
	}

	/**
	 * @return sets power to all three intake motors, looking at conditions of
	 *         whether there is cargo and elevator position
	 * @param power
	 */
	public void setIntakePower(double power) {
		// Stop intake if there is cargo
		if (isCargo() && power > 0) {
			power = 0;
		}

		// Stop outer intake if elevator is up
		if (Robot.elevator.elevatorEncoder.getDistance() > Elevator.MIN_ENCODER_LIMIT) {
			masterIntakeMotor.set(0);
		} else {
			masterIntakeMotor.set(power);
		}

		innerIntakeMotor.set(power);
	}

	/**
	 * @return if cargo is in the intake it returns true, if false there is a hatch.
	 */
	public boolean isCargo() {
		return distanceSensor.getValue() > RobotMap.intakeSensorThreshold;
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new IntakeControl());
	}
}