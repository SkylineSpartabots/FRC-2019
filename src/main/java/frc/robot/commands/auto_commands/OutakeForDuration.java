package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Debouncer;

public class OutakeForDuration extends Command {

	private double duration;
  private Timer timer;

	/**
	 * Runs the intake at a specified power for a specified duration.
	 * 
	 * @param duration number of seconds to run the intake at
	 */
	public OutakeForDuration(double duration) {
		requires(Robot.intake);
		
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
    Robot.intake.setIntakePower(-1);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (timer.get() > duration) || Math.abs(Robot.oi.driveStick.getRX()) > 0.1 || Math.abs(Robot.oi.driveStick.getLY()) > 0.1;
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