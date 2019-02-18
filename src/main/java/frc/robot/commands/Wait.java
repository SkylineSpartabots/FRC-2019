package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Wait extends Command {

	private double waitDuration; // in milliseconds
	private Timer timer;

	/**
	 * Creates a command that finishes after the specified number of milliseconds.
	 * 
	 * @param waitDuration duration of timer in millseconds
	 */
	public Wait(double waitDuration) {
		this.waitDuration = waitDuration;
		timer = new Timer();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		timer.reset();
		timer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return timer.get() > (waitDuration / 1000);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
	}
}