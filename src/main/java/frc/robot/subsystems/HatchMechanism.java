package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Subsystem for the lotus mechanism. Controls hatch panels.
 */
public class HatchMechanism extends Subsystem {

	private Solenoid lotusSolenoid; // true when open (holding hatch in place), false when closed
	private Solenoid sliderSolenoid; // true when pushed out, false when pushed in

	public HatchMechanism() {
		lotusSolenoid = new Solenoid(RobotMap.HATCH_SOLENOID);
		sliderSolenoid = new Solenoid(RobotMap.HATCH_SLIDER_SOLENOID);

		// ensures neutral position of closed lotus in back position
	}

	/**
	 * Closes the lotus. This is the default position: no hatch panel inside.
	 */
	public void releaseLotus() {
		lotusSolenoid.set(true);
	}

	/**
	 * Opens the lotus to secure a hatch panel.
	 */
	public void graspLotus() {
		lotusSolenoid.set(false);
	}

	/**
	 * Returns whether the lotus is grasping something.
	 * 
	 * @return true if lotus is open, else false
	 */
	public boolean isLotusOpen() {
		return lotusSolenoid.get();
	}

	/**
	 * Slides the hatch mechanism out.
	 */
	public void slideOut() {
		sliderSolenoid.set(true);
	}

	/**
	 * Slides the hatch mechanism in.
	 */
	public void slideIn() {
		sliderSolenoid.set(false);
	}

	/**
	 * Returns whether the hatch mechanism is slid out.
	 * 
	 * @return true if mechanism is out, else true
	 */
	public boolean isSliderOut() {
		return sliderSolenoid.get();
	}

	@Override
	public void initDefaultCommand() {
	}
}