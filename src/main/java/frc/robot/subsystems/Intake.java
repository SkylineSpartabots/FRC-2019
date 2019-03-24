package frc.robot.subsystems;

import java.util.Arrays;
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
import frc.robot.util.Equivalency;

public class Intake extends Subsystem {

	WPI_TalonSRX masterIntakeMotor, slaveIntakeMotor, innerIntakeMotor;
	Solenoid intakeSolenoid, slaveIntakeSolenoid;
	AnalogInput beamBreak;
	Debouncer.RawInput beamBreakInput;
	Debouncer beamBreakDebouncer;
	private final static double CARGO_CURRENT_THRESHOLD = 1;
	public boolean cargoOverride = false;

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
		
		if (isCargo || Robot.elevator.getElevatorEncoderOutput() > Elevator.MIN_ENCODER_LIMIT || !getIntakeSolenoidState()) {
			masterIntakeMotor.set(0);
		} else {
			masterIntakeMotor.set(power);
		}

		if (power != 0) {
			if (isCargo) {
				retractIntake();
				if (power < 0) {
					if(Robot.elevator.getElevatorEncoderOutput() > Elevator.MIN_ENCODER_LIMIT){
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

	public boolean getRawCargoPositionCurrent(){
		return innerIntakeMotor.getOutputCurrent() > CARGO_CURRENT_THRESHOLD;
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
		boolean motorFailure = false;
		boolean solenoidFailure = false;
		boolean sensorFailure = false;

		System.out.println("Testing Intake........................");


		System.out.println("\n\nTesting Intake Motors.................");
		final double kCurrentThreshold = 0.5;

		masterIntakeMotor.set(1);
		innerIntakeMotor.set(1);
		Timer.delay(3);
		double masterIntakeCurrent = masterIntakeMotor.getOutputCurrent();
		double slaveIntakeCurrent = slaveIntakeMotor.getOutputCurrent();
		double innerIntakeCurrent = innerIntakeMotor.getOutputCurrent();
		masterIntakeMotor.set(0);
		innerIntakeMotor.set(0);

		System.out.println("Master Intake Current:\t" + masterIntakeCurrent);
		System.out.println("Slave Intake Current:\t" + slaveIntakeCurrent);
		System.out.println("Inner Intake Current:\t" + innerIntakeCurrent);

		if(masterIntakeCurrent < kCurrentThreshold){
			System.out.println("!!!!!!!! FAILURE: RIGHT INTAKE MOTOR CURRENT LOW !!!!!!!!");
			motorFailure = true;
		}

		if(slaveIntakeCurrent < kCurrentThreshold){
			System.out.println("!!!!!!!! FAILURE: LEFT INTAKE MOTOR CURRENT LOW !!!!!!!!");
			motorFailure = true;
		}

		if(slaveIntakeCurrent < kCurrentThreshold){
			System.out.println("!!!!!!!! FAILURE: INNER INTAKE MOTOR CURRENT LOW !!!!!!!!");
			motorFailure = true;
		}

		if(!Equivalency.allAboutEqualTo(Arrays.asList(masterIntakeCurrent, slaveIntakeCurrent), masterIntakeCurrent, kCurrentThreshold)){
			System.out.println("!!!!!!!! FAILURE: INTAKE KEBABS OPERATING AT VARYING CURRENT !!!!!!!!");
			motorFailure = true;
		}

		if(!motorFailure){
			System.out.println("######## SUCCESSFUL: GO FOR INTAKE MOTORS ########");
		}


		System.out.println("\n\nTesting Intake Solenoids...............");

		extendIntake();
		Timer.delay(2);
		if(!getIntakeSolenoidState()){
			System.out.println("!!!!!!!! FAILURE: INTAKE SOLENOIDS FAILED TO EXTEND !!!!!!!!");
			solenoidFailure = true;
		}

		retractIntake();
		Timer.delay(2);
		if(getIntakeSolenoidState()){
			System.out.println("!!!!!!!! FAILURE: INTAKE SOLENOIDS FAILED TO RETRACT !!!!!!!!");
			solenoidFailure = true;
		}
		
		if(!solenoidFailure){
			System.out.println("######## SUCCESSFUL: GO FOR INTAKE SOLENOIDS ########");
		}

		System.out.println("\n\nTesting Beam Break.....................");
		if(getRawCargoPosition()) {
			sensorFailure = true;
			System.out.println("!!!!!!!! FAILURE: BEAM BREAK STARTED IN A BROKEN STATE !!!!!!!!");
		} else {
			for(int i = 0; i <= 10; i++) {
				System.out.println("PLEASE MANUALLY BREAK BEAM");
				if(getRawCargoPosition()) {
					System.out.println("BEAM SUCCESSFULLY BROKEN");
					sensorFailure = false;
					break;
				} else {
					sensorFailure = true;
				}
				Timer.delay(0.5);
			}

			if(sensorFailure) {
				System.out.println("!!!!!!!! FAILURE: BEAM WAS NOT BROKEN !!!!!!!!");
			} else {
				System.out.println("######## SUCCESSFUL: GO FOR BEAM BREAK ########");
			}

		}

		return !motorFailure && !solenoidFailure && !sensorFailure;
	}
}