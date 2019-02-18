package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class IntakeControl extends Command {

	public IntakeControl() {
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.intake.setIntakePower(0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Controls for moving intake down and up using dpad
		if (Robot.oi.secondStick.isPOVUpish()) {
			Robot.intake.extendIntake();
		} else if (Robot.oi.secondStick.isPOVDownish() || Robot.elevator.getElevatorEncoderOutput() > Elevator.MIN_ENCODER_LIMIT) {
			Robot.intake.retractIntake();
			return;
		}

		double rTrigger = Robot.oi.secondStick.getRTrigger();
		double lTrigger = Robot.oi.secondStick.getLTrigger();

		// Sets power to the intake motors. Uses whichever trigger is more pressed
		if (rTrigger > lTrigger) {
			Robot.intake.setIntakePower(-rTrigger);
		} else if (rTrigger < lTrigger) {
			Robot.intake.setIntakePower(lTrigger);
		} else {
			Robot.intake.setIntakePower(0);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
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