package frc.robot.commands.auto_commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Debouncer;
import frc.robot.util.SimplePID;

public class TurnDegrees extends Command {

	private double kP, kI, kD;

	/**
	 * kP = 0.023
	 * kI = 0.006
	 * kD = 0.011
	 */

	private double angle;
	private final int CLOCK_MAX = 1;
	private Timer timer;
	private double output = 0;
	private DoubleSupplier turnSource;
	private SimplePID turnPID;

	private double turnThreshold;
	private double timeOutSecs;

	private Debouncer pidDebouncer;
	private BooleanSupplier pidDebouncerInput;

	public TurnDegrees(double angle, double timeOutSecs) {
		requires(Robot.driveTrain);
		timer = new Timer();
		this.angle = angle;
		this.timeOutSecs = timeOutSecs;
		turnSource = () -> Robot.navx.getAngle();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		kP = 0.022;
		kI = 0.003;
		kD = 0.009;
		
		turnThreshold = 1.5;
		 
		
		turnPID = new SimplePID(turnSource, this.angle, kP, kI, kD);
		turnPID.setOutputLimits(-.65, .65);

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
		return pidDebouncer.getDebouncedValue() || timer.get() > timeOutSecs 
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