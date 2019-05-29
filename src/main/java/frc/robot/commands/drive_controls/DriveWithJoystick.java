package frc.robot.commands.drive_controls;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.controllers.Xbox;

public class DriveWithJoystick extends Command {

	private double forwardClipAmount, turnClipAmount;


	public DriveWithJoystick() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveTrain.stop();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double turn = Robot.oi.driveStick.getRX();
		double forward = Robot.oi.driveStick.getLY();
		if(Robot.elevator.getElevatorEncoderOutput() >= 420 && Robot.elevator.getElevatorEncoderOutput() <= 700){
			forwardClipAmount = 0.7;
			turnClipAmount = 0.75;
		} else if(Robot.elevator.getElevatorEncoderOutput() >= 700){
			forwardClipAmount = 0.65;
			turnClipAmount = 0.6;
		} else{
			forwardClipAmount = 1;
			turnClipAmount = 1;
		}

		if(Robot.oi.driveStick.buttonA.get()) {
			Robot.driveTrain.arcadeDrive(-Xbox.clipAxis(forward, forwardClipAmount), Xbox.clipAxis(Robot.limelight.getSteerCorrection(), turnClipAmount));
		} else {
			Robot.driveTrain.arcadeDrive(-Xbox.clipAxis(forward, forwardClipAmount), 0.75*(Xbox.clipAxis(turn, turnClipAmount)));
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
		Robot.driveTrain.stop();
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
