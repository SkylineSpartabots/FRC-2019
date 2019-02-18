package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class IntakeControl extends Command {

	private double rTrigger;
	private double lTrigger;

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

		//Controls for moving intake down and up using dpad
		int dpad_val = Robot.oi.secondStick.getPOV();

		if (dpad_val == 315 || dpad_val == 0 || dpad_val == 45) {
			Robot.intake.extendIntake();
		} else if (dpad_val == 225 || dpad_val == 180 || dpad_val == 135 ||  Robot.elevator.getElevatorEncoderOutput() > Elevator.MIN_ENCODER_LIMIT) {
			Robot.intake.retractIntake();
			return;
		}


		rTrigger = Robot.oi.secondStick.getRawAxis(OI.Axis.RTrigger.getAxisNumber());
		rTrigger = Robot.oi.clipDeadzone(rTrigger);
		lTrigger = Robot.oi.secondStick.getRawAxis(OI.Axis.LTrigger.getAxisNumber());
		lTrigger = Robot.oi.clipDeadzone(lTrigger);

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