package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeForDuration extends Command {

	private double intakePower;
	private double timeInMsec;
	private Timer timer;

	public IntakeForDuration(double intakePower, double timeInMsec) {
		requires(Robot.intake);
		
		this.intakePower = intakePower;
		this.timeInMsec = timeInMsec;
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
		Robot.intake.setIntakePower(intakePower);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (timer.get() > (timeInMsec / 1000)) || Robot.intake.isCargo();
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