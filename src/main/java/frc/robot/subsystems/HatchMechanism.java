package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class HatchMechanism extends Subsystem {

	private Solenoid hatchSolenoid;

	public HatchMechanism() {
		hatchSolenoid = new Solenoid(RobotMap.HATCH_SOLENOID);
		hatchSolenoid.set(false);
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

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}