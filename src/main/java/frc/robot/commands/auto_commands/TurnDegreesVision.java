package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;

public class TurnDegreesVision extends Command {
	private double angleFudgeFactor = -0.8;
	private int clockCounter = 0;
	private double angle;
	private final double CLOCK_MAX = 10;
	private boolean isFinished = false;
	private double error;

	private Timer timer;
	private double output = 0;
	private PIDSource turnSource;
	private SimplePID turnPID;

	double kP = 0.016;
	double kI = 0.00056;
	double kD = 0.0012;

	private double turnThreshold = 4;
	private double timeOutSecs;

	public TurnDegreesVision(double timeOutSecs) {
		requires(Robot.driveTrain);

		timer = new Timer();
		this.timeOutSecs = timeOutSecs;

		turnSource = () -> Robot.rps.getNavxAngle();


		//double[] pidConstants = Robot.driveTrain.getTurnPID();
		//turnPID = new SimplePID(turnSource, 0, pidConstants[0],pidConstants[1], pidConstants[2], "TurnDegreesPID",true);
		
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		turnPID = new SimplePID(turnSource, 0, kP, kI, kD, "TurnDegreesPID",true);
		turnPID.setOutputLimits(-1, 1);

		turnPID.resetPID();
		this.angle = Robot.rps.getYawToVisionTargetRawDegrees()*angleFudgeFactor + Robot.rps.getNavxAngle();
		turnPID.setSetpoint(this.angle);
		timer.reset();
		timer.start();

		isFinished = false;
		clockCounter = 0;
		timeOutSecs += timer.get();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		error = turnPID.getError();
		output = turnPID.compute();

		if (Math.abs(error) < turnThreshold) {
			clockCounter++;
			if (clockCounter >= CLOCK_MAX) {
				isFinished = true;
			}
		} else {
			clockCounter = 0;
		}
		System.out.println("!!!!!!!! EXECUTING !!!!!!!!!!");
		//System.out.println("Turn Power" + output);
		Robot.driveTrain.tankDrive(output, -output);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isFinished || timer.get() > timeOutSecs
			|| Math.abs(Robot.oi.driveStick.getLY()) > 0.1 || Math.abs(Robot.oi.driveStick.getRX()) > 0.1;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		turnPID.resetPID();
		Robot.driveTrain.tankDrive(0, 0);
		timer.stop();
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}