package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class DriveWithJoystick extends Command {

	private double turn;
	private double forward;

	public DriveWithJoystick() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		turn = Robot.oi.driveStick.getRawAxis(OI.Axis.RX.getAxisNumber());
		forward = Robot.oi.driveStick.getRawAxis(OI.Axis.LY.getAxisNumber());
		Robot.driveTrain.arcadeDrive(-Robot.oi.clipDeadzone(forward), Robot.oi.clipDeadzone(turn));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}