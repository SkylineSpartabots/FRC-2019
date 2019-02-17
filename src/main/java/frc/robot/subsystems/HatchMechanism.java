package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class HatchMechanism extends Subsystem {

	private Solenoid hatchSolenoid, hatchSliderSolenoid;

	public HatchMechanism() {
		hatchSolenoid = new Solenoid(RobotMap.HATCH_SOLENOID);
		hatchSliderSolenoid = new Solenoid(RobotMap.HATCH_SLIDER_SOLENOID);

		hatchSolenoid.set(false);
		hatchSliderSolenoid.set(false);
	}

	public void release() {
		hatchSolenoid.set(false);
	}

	public void grasp() {
		hatchSolenoid.set(true);
	}

	public boolean getState() {
		return hatchSolenoid.get();
	}

	public void slideHatchOut() {
		hatchSliderSolenoid.set(true);
	}

	public void slideHatchIn() {
		hatchSliderSolenoid.set(false);
	}

	public boolean getSliderState() {
		return hatchSliderSolenoid.get();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}