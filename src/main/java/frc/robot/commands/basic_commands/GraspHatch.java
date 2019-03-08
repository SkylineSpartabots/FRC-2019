package frc.robot.commands.basic_commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.VibrateControllers;

public class GraspHatch extends Command {

	private VibrateControllers vibrateControllers;

	public GraspHatch() {
		requires(Robot.hatchMechanism);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.hatchMechanism.graspHatch();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if(Robot.hatchMechanism.isLotusOpen()) {
			try {
				vibrateControllers = new VibrateControllers(0.3, Robot.oi.driveStick, Robot.oi.secondStick);
				vibrateControllers.start();
				
			} finally {
				vibrateControllers.close();
			}
			return true;
		} else{
			return false;
		}
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