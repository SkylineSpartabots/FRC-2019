package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive_controls.*;
import frc.robot.util.Debouncer;


public class Intake extends Subsystem {

	WPI_TalonSRX masterIntakeMotor, slaveIntakeMotor, innerIntakeMotor;
	Solenoid intakeSolenoid, slaveIntakeSolenoid;
	AnalogInput beamBreak;
	BooleanSupplier beamBreakInput;
	Debouncer beamBreakDebouncer;

	public Intake() {
		masterIntakeMotor = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);
		slaveIntakeMotor = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);

		masterIntakeMotor.setNeutralMode(NeutralMode.Brake);
		slaveIntakeMotor.setNeutralMode(NeutralMode.Brake);

		masterIntakeMotor.setInverted(false);
		slaveIntakeMotor.setInverted(true);

		slaveIntakeMotor.follow(masterIntakeMotor);

		innerIntakeMotor = new WPI_TalonSRX(RobotMap.INNER_INTAKE_MOTOR);
		innerIntakeMotor.setInverted(false);
		innerIntakeMotor.setNeutralMode(NeutralMode.Brake);

		beamBreak = new AnalogInput(RobotMap.INTAKE_SENSOR);

		beamBreakInput = () -> getRawCargoPosition();

		beamBreakDebouncer = new Debouncer(beamBreakInput, 13);

		intakeSolenoid = new Solenoid(RobotMap.INTAKE_SOLENOID);

		

	}

	public void extendIntake() {
		intakeSolenoid.set(true);
	}

	public void retractIntake() {
		intakeSolenoid.set(false);
	}

	public boolean getIntakeSolenoidState() {
		return intakeSolenoid.get();
	}

	/**
	 * @return sets power to all three intake motors, looking at conditions of
	 *         whether there is cargo and elevator position
	 * @param power
	 */
	public void setIntakePower(double power) {

		boolean isCargo = isCargo();
		
		if (isCargo || Robot.elevator.getElevatorEncoderOutput() > 60 || !getIntakeSolenoidState()) {
			masterIntakeMotor.set(0);
		} else {
			masterIntakeMotor.set(power);
		}

		if (power != 0) {
			if (isCargo) {
				retractIntake();
				if (power < 0) {
					if(Robot.elevator.getElevatorEncoderOutput() > 60){
						innerIntakeMotor.set(power*0.75);
					} else {
						innerIntakeMotor.set(0);
					}
				} else {
					innerIntakeMotor.set(power);
				}
			} else {
				innerIntakeMotor.set(power);
			}
		} else {
			innerIntakeMotor.set(0);
		}

	}

	public void setRawIntakePower(double power){
		if(getIntakeSolenoidState()){
			masterIntakeMotor.set(power);
		} else {
			masterIntakeMotor.set(0);
		}

		innerIntakeMotor.set(power);
	}

	/**
	 * @return if cargo is in the intake it returns true, if false there is a hatch.
	 */
	public boolean getRawCargoPosition() {
		return beamBreak.getValue() < RobotMap.INTAKE_SENSOR_THRESHOLD;
	}


	public boolean isCargo(){
		return beamBreakDebouncer.getDebouncedValue();
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new IntakeControl());
	}

	public void setIntakeDataOnDisplay(){
		SmartDashboard.putBoolean("Is Cargo Inside", isCargo());
		SmartDashboard.putBoolean("Are Intake Kebabs Extended", getIntakeSolenoidState());
		SmartDashboard.putNumber("Intake Kebabs Power", masterIntakeMotor.get());
		SmartDashboard.putNumber("Inner Intake Power", innerIntakeMotor.get());
		SmartDashboard.putNumber("Beam Break Value", beamBreak.getValue());

	}

	public boolean checkSubsystem(){
		return true;
	}

		
}