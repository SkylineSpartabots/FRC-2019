package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class ElevatorControl extends Command {

	public ElevatorControl() {
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.setPower(0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double joyVal = Robot.oi.secondStick.getRawAxis(OI.Axis.RY.getAxisNumber());
		joyVal = Robot.oi.clipDeadzone(joyVal);
		System.out.println(-joyVal);
		Robot.elevator.setRawPower(-joyVal); 
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
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