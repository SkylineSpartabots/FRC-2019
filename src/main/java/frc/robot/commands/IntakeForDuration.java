package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeForDuration extends Command {

	private double power;
	private double duration;
	private Timer timer;

	/**
	 * Runs the intake at a specified power for a specified duration.
	 * 
	 * @param power power to set the intake motor at, (0, 1]
	 * @param duration number of milliseconds to run the intake at
	 */
	public IntakeForDuration(double power, double duration) {
		requires(Robot.intake);
		
		this.power = power;
		this.duration = duration;
		timer = new Timer();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.intake.setIntakePower(0);
		timer.reset();
		timer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.intake.setIntakePower(power);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (timer.get() > (duration / 1000)) || Robot.intake.containsCargo();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.intake.setIntakePower(0);
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}