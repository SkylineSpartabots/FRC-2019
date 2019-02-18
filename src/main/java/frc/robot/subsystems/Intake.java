package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeControl;

/**
 * Subsystem for the kebab intakes and the inner intake.
 * Only intakes cargo balls.
 */
public class Intake extends Subsystem {

	private WPI_TalonSRX masterIntakeMotor, slaveIntakeMotor, innerIntakeMotor;
	private Solenoid intakeSolenoid, slaveIntakeSolenoid;
	private AnalogInput distanceSensor;

	public Intake() {
		masterIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);
		slaveIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);

		masterIntakeMotor.setNeutralMode(NeutralMode.Brake);
		slaveIntakeMotor.setNeutralMode(NeutralMode.Brake);

		masterIntakeMotor.setInverted(false); // TODO: set directions
		slaveIntakeMotor.setInverted(false);

		slaveIntakeMotor.follow(masterIntakeMotor);

		innerIntakeMotor = new WPI_TalonSRX(RobotMap.INNER_INTAKE_MOTOR);
		innerIntakeMotor.setInverted(false);

		distanceSensor = new AnalogInput(RobotMap.INTAKE_SENSOR);

		intakeSolenoid = new Solenoid(RobotMap.INTAKE_SOLENOID);
	}

	/**
	 * Places intake kebabs down, ready to intake a cargo ball
	 */
	public void extendIntake() {
		intakeSolenoid.set(true);
	}

	/**
	 * Moves intake kebabs up, out of the way of the elevator
	 */
	public void retractIntake() {
		intakeSolenoid.set(false);
	}

	/**
	 * Returns whether the intake kebabs are down (extended) or up (retracted)
	 * 
	 * @return true if intake kebabs are down, else false
	 */
	public boolean areIntakeKebabsExtended() {
		return intakeSolenoid.get();
	}

	/**
	 * Sets power to all three intake motors, looking at conditions of whether there
	 * is cargo and elevator position
	 * 
	 * @param power power to run the motors at, power > 0
	 */
	public void setIntakePower(double power) {
		// Stop intake if there is cargo
		if (containsCargo() && power > 0) {
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
	 * Returns whether a cargo ball is inside the inner intake
	 * 
	 * @return true if cargo is inside, else false
	 */
	public boolean containsCargo() {
		return distanceSensor.getValue() > RobotMap.INTAKE_SENSOR_THRESHOLD;
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new IntakeControl());
	}
}