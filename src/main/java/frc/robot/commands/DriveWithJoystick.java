package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
public class DriveWithJoystick extends Command {


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
		double left = Robot.oi.driveStick.getLY();
		double right = Robot.oi.driveStick.getRY();
		double turn = Robot.oi.driveStick.getRX();
		double forward = Robot.oi.driveStick.getLY();
		
		//Robot.driveTrain.rawMotorOutput(-left, -right);
		Robot.driveTrain.arcadeDrive(-forward, turn);
		//System.out.println("Driving: " + forward);
		//Robot.driveTrain.testMotor();
		//Robot.driveTrain.tankDrive(-left, -right);
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
