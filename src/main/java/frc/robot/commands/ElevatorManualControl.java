package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class ElevatorManualControl extends Command {

	private double power;

	/**
	 * Specify a power to make the elevator go up or down
	 * 
	 * @param power
	 */
	public ElevatorManualControl(double power) {
		requires(Robot.elevator);
		this.power = power;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.setPower(0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.elevator.setPower(power);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return !Robot.oi.driveStick.getRawButton(OI.Button.LBumper.getBtnNumber());
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.elevator.setPower(0);
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}