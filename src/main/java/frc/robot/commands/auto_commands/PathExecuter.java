package frc.robot.commands.auto_commands;

import java.io.File;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.Debouncer;
import frc.robot.util.Logger;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * @author Neil (the great) Hazra
 */

public class PathExecuter extends Command {
	public Debouncer isFinishedDebouncer;
	public boolean prematureTermination = false;
	private boolean isBackwards;
	//private Timer timer;

	private double proportionOfMaxVelocity = 0.7;
	private double P = 0.9;
	private double D = 0.01;
	private double k_a = 0.02;	

	private double TurnP = 0.005;
	private double TurnI = 0.0;
	private double TurnD = 0.0005;

	private DistanceFollower left, right;
	private PIDSource NAVXSource;
	private SimplePID turnPID;
	private Logger PathingLog;

	private double LeftMotorOutput = 0;
	private double RightMotorOutput = 0;
	private boolean log = true;

	private double timeOutSecs;


	public void getPathingConstants(){
		double[] pathConstants = Robot.driveTrain.getPathConstants();
		P = pathConstants[0];
		D = pathConstants[1];
		proportionOfMaxVelocity = pathConstants[2];
		k_a = pathConstants[3];
	}


	public void initPathExecuter(String FileName) {
		try {
			File f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + FileName);
			if(!f.exists())	{
				prematureTermination = true;
				return;
			}
			Trajectory trajectory = Pathfinder.readFromCSV(f);
			Robot.SystemLog.writeWithTimeStamp("PathExecuter: Trajectory Loaded From File");
			TankModifier modifier = new TankModifier(trajectory).modify(RobotMap.TRACK_WIDTH);
			left = new DistanceFollower(modifier.getLeftTrajectory());
			right = new DistanceFollower(modifier.getRightTrajectory());
			left.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			right.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);

			NAVXSource = () -> Robot.rps.getNavxAngle();

			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, FileName+"TurnPID",log);
			PathingLog = new Logger(FileName + "Log");
		} catch (Exception e) {
			prematureTermination = true;
		}
	}
	public void initPathExecuter(Waypoint[] points,String FileName) {
		try {
			System.out.println("Generating Trajectory");
			Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_LOW, 0.02, proportionOfMaxVelocity*RobotMap.MAX_VELOCITY, 2.0, 60.0);

			Trajectory trajectory = Pathfinder.generate(points, config);
			System.out.println("Trajectory Length: " + trajectory.length());
			File f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + FileName);
			if(f.exists())	{
				Pathfinder.writeToCSV(f, trajectory);
			}
				Robot.SystemLog.writeNewData("PathExecuter: Trajectory Path Saved To File");

			TankModifier modifier = new TankModifier(trajectory).modify(RobotMap.TRACK_WIDTH);
			left = new DistanceFollower(modifier.getLeftTrajectory());
			right = new DistanceFollower(modifier.getRightTrajectory());
			left.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			right.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			NAVXSource = new PIDSource() {
				public double getInput() {
					return Robot.rps.getNavxAngle();
				}
			};			
			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, FileName+"TurnPID",log);
			PathingLog = new Logger(FileName + "Log");
		} catch (Exception e) {
			prematureTermination = true;
		}
	}

	public PathExecuter(String FileName, boolean isBackwards, double timeOutSecs, double appliedProportionofMaxVelocity, boolean log) {
		requires(Robot.driveTrain);	
		this.log = log;	
		getPathingConstants();
		initPathExecuter(FileName);

		/*timer = new Timer();
		timer.reset();
		timer.start();*/

		this.isBackwards = isBackwards;
		proportionOfMaxVelocity *= appliedProportionofMaxVelocity;
		//this.timeOutSecs = timeOutSecs + timer.get();
		
		prematureTermination = Math.abs(Robot.oi.driveStick.getRX()) > 0.1 || Math.abs(Robot.oi.driveStick.getLY()) > 0.1;
	}
	public PathExecuter(Waypoint[] points, boolean isBackwards,double timeOutSecs, double appliedProportionofMaxVelocity, String FileName, boolean log) {
		requires(Robot.driveTrain);
		this.log = log;
		getPathingConstants();
		initPathExecuter(points, FileName);
		
		/*timer = new Timer();
		timer.reset();
		timer.start();*/

		this.isBackwards = isBackwards;
		proportionOfMaxVelocity *= appliedProportionofMaxVelocity;
		//this.timeOutSecs = timeOutSecs + timer.get();
		prematureTermination = Math.abs(Robot.oi.driveStick.getRX()) > 0.1 || Math.abs(Robot.oi.driveStick.getLY()) > 0.1;
	}

	public void updateMotorOutputs(double LeftEncoderDistance, double RightEncoderDistance) {
		double l = 0;
		double r = 0;
		Segment left_s = null;
		Segment right_s = null;
		if(!left.isFinished() && !right.isFinished())	{
			left_s = left.getSegment();
			right_s = right.getSegment();
			l = left.calculate(LeftEncoderDistance);
			r = right.calculate(RightEncoderDistance);
		}
		double desired_heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));
		turnPID.setSetpoint(-desired_heading);
		double turn = turnPID.compute();

		if(!isBackwards) {
			LeftMotorOutput = l + turn;
			RightMotorOutput = r - turn;
		} else {
			LeftMotorOutput = l - turn;
			RightMotorOutput = r + turn;
		}

		if(log && !left.isFinished() && !right.isFinished())	{
				PathingLog.writeNewData(
					Timer.getFPGATimestamp()+","+desired_heading+","+left_s.position+","+right_s.position+","+
					turnPID.getInput()+","+Robot.driveTrain.getLeftEncoderDistanceMeters()+","+Robot.driveTrain.getRightEncoderDistanceMeters()+","+
					LeftMotorOutput+","+RightMotorOutput);
		}
	}

	/**
	 * All units in metric meters
	 */
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if(!prematureTermination)	{
			isFinishedDebouncer = new Debouncer(10);
			turnPID.resetPID();
			Robot.rps.reset();
			Robot.driveTrain.resetEncoders();
			left.reset();
			right.reset();
			Robot.SystemLog.writeWithTimeStamp("Path Executer Initialized: Angle=" + Robot.rps.getNavxAngle());
			if(log)	PathingLog.writeNewData("Time, Desired Heading, Desired Left Position, Desired Right Position, Heading, Left Position, Right Posiion, LeftPower, RightPower");	
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(!prematureTermination)	{

			if(!isBackwards){
				updateMotorOutputs(Robot.driveTrain.getLeftEncoderDistanceMeters(), Robot.driveTrain.getRightEncoderDistanceMeters());
				Robot.driveTrain.rawMotorOutput(LeftMotorOutput, RightMotorOutput);
			} else {
				updateMotorOutputs(-Robot.driveTrain.getLeftEncoderDistanceMeters(), -Robot.driveTrain.getRightEncoderDistanceMeters());
				Robot.driveTrain.rawMotorOutput(-LeftMotorOutput, -RightMotorOutput);
			}
			
			System.out.println("kp:\t" + P + "kd:\t" + D + "maxv:\t" + proportionOfMaxVelocity + "ka:\t" + k_a);
		}
	}
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return prematureTermination || isFinishedDebouncer.getDebouncedValue((left.isFinished() && right.isFinished())) || Math.abs(Robot.oi.driveStick.getLY()) > 0.1;
	}
	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.rps.reset();
		left.reset();
		right.reset();
		turnPID.resetPID();
		Robot.driveTrain.rawMotorOutput(0,0);
		Robot.driveTrain.setBrake();
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		Robot.SystemLog.writeWithTimeStamp("Path Executer Interrupted");
		end();
	}
}