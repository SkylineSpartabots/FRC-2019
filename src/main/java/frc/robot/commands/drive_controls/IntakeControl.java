package frc.robot.commands.drive_controls;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.VibrateControllers;
import frc.robot.subsystems.Elevator;

public class IntakeControl extends Command {

	boolean hasVibrated = true;
	VibrateControllers vibrateControllers;
	boolean manualOverride;
	
	public IntakeControl() {
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.intake.setIntakePower(0);
		manualOverride = false;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Controls for moving intake down and up using dpad

		if(Robot.intake.isCargo() && !hasVibrated){
			hasVibrated = true;
			try {
				vibrateControllers = new VibrateControllers(0.4, Robot.oi.secondStick);
				vibrateControllers.start();
			  } finally {
				vibrateControllers.close();
			  }
		} else if (!Robot.intake.isCargo()){
			hasVibrated = false;
		}
		
		manualOverride = true;
		
		
		if(!manualOverride){
			if (Robot.oi.secondStick.isPOVDownish() || Robot.elevator.getElevatorEncoderOutput() > Elevator.MIN_ENCODER_LIMIT) {
				Robot.intake.extendIntake();
			} else if (Robot.oi.secondStick.isPOVUpish()) {
				Robot.intake.retractIntake();
			}
			
			boolean rTriggerOn = Robot.oi.secondStick.getRTrigger() > 0.2;
			boolean lTriggerOn = Robot.oi.secondStick.getLTrigger() > 0.2;
			// Sets power to the intake motors. Uses whichever trigger is more pressed
			if (rTriggerOn) {
				Robot.intake.setIntakePower(-1);
			} else if (lTriggerOn) {
				Robot.intake.setRawIntakePower(0.75);
			} else {
				Robot.intake.setRawIntakePower(0);
			}
		} else {
			if(Robot.oi.secondStick.isPOVDownish()){
				Robot.intake.extendIntake();
			} else if(Robot.oi.secondStick.isPOVUpish() ){
				Robot.intake.retractIntake();
			}

			boolean rTriggerOn = Robot.oi.secondStick.getRTrigger() > 0.2;
			boolean lTriggerOn = Robot.oi.secondStick.getLTrigger() > 0.2;
			// Sets power to the intake motors. Uses whichever trigger is more pressed
			if (rTriggerOn) {
				Robot.intake.setRawIntakePower(-1);
			} else if (lTriggerOn) {
				Robot.intake.setRawIntakePower(0.75);
			} else {
				Robot.intake.setRawIntakePower(0);
			}
			
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