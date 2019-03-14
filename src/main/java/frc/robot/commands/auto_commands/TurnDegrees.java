package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Debouncer;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;

public class TurnDegrees extends Command {


	private double angle;
	private final int CLOCK_MAX = 5;
	private Timer timer;
	private double output = 0;
	private PIDSource turnSource;
	private SimplePID turnPID;

	private double turnThreshold;
	private double timeOutSecs;

	private Debouncer pidDebouncer;
	private Debouncer.RawInput pidDebouncerInput;

	public TurnDegrees(double angle, double timeOutSecs) {
		requires(Robot.driveTrain);

		timer = new Timer();
		this.angle = angle + Robot.rps.getNavxAngle();
		this.timeOutSecs = timeOutSecs;

		turnSource = () -> Robot.rps.getNavxAngle();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		double[] pidConstants = Robot.driveTrain.getTurnPID();
		
		turnPID = new SimplePID(turnSource, this.angle, pidConstants[0], pidConstants[1], pidConstants[2], "TurnDegreesPID", false);
		turnPID.setOutputLimits(-1, 1);

		pidDebouncerInput = () -> Math.abs(turnPID.getError()) <= turnThreshold;
		pidDebouncer = new Debouncer(pidDebouncerInput, CLOCK_MAX);
		timer.reset();
		timer.start();
		turnPID.resetPID();


		timeOutSecs += timer.get();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		output = turnPID.compute();
		Robot.driveTrain.tankDrive(output, -output);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return pidDebouncer.getDebouncedValue() || timer.get() > timeOutSecs;
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