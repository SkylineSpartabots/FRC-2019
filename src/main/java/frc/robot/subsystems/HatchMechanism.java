package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Subsystem for the lotus mechanism. Controls hatch panels.
 */
public class HatchMechanism extends Subsystem {

	private Solenoid lotusSolenoid; // true when open (holding hatch in place), false when closed
	private Solenoid sliderSolenoid; // true when pushed out, false when pushed in

	private AnalogInput hatchLimitSwitch;

	public HatchMechanism() {
		lotusSolenoid = new Solenoid(RobotMap.HATCH_SOLENOID);
		sliderSolenoid = new Solenoid(RobotMap.HATCH_SLIDER_SOLENOID);

		hatchLimitSwitch = new AnalogInput(2);

		// ensures neutral position of closed lotus in back position
	}

	/**
	 * Closes the lotus. This is the default position: no hatch panel inside.
	 */
	public void releaseHatch() {
		lotusSolenoid.set(true);
	}

	/**
	 * Opens the lotus to secure a hatch panel.
	 */
	public void graspHatch() {
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

	/**
	 * 
	 * @return true if a hatch is in the hatch mechanism, false if it is not
	 */
	public boolean getHatchState(){
		return hatchLimitSwitch.getValue() < RobotMap.HATCH_LIMIT_SWTICH_THRESHOLD;
	}

	@Override
	public void initDefaultCommand() {
		//setDefaultCommand(new HatchMechanismControl());
	}


	public void setHatchMechanismDataOnDisplay(){
		SmartDashboard.putBoolean("Is the Lotus Open", isLotusOpen());
		SmartDashboard.putBoolean("Is the Slider Extended", isSliderOut());
		SmartDashboard.putBoolean("Is a Hatch Obtained", getHatchState());
		SmartDashboard.putNumber("Limit Switch", hatchLimitSwitch.getValue());
	}
}