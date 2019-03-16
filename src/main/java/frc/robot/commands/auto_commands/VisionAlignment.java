package frc.robot.commands.auto_commands;

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

public class VisionAlignment extends Command {
	/*
	private final double P = 1.1;
	private final double D = 0;
	private final double k_a = 0.02;

	private final double TurnP = 0.027;
	private final double TurnI = 0.0;
	private final double TurnD = 0.00;

	*/
	public Debouncer isFinishedDebouncer;
	public boolean prematureTermination = false;
	public boolean log = true;
	private double P = 0.9;
	private double D = 0.01;
	private double k_a = 0.02;
	private double proportionOfMaxVelocity = 0.7;
	private double TurnP = 0.005;
	private double TurnI = 0.0;
	private double TurnD = 0.0005;



	private DistanceFollower left, right;
	private PIDSource NAVXSource;
	private SimplePID turnPID;
	private Logger PathingLog;

	private double LeftMotorOutput = 0;
	private double RightMotorOutput = 0;
		

	public void initPathExecuter(Trajectory traj, String FileName, boolean logPID) {

		/*pathConstants = Robot.driveTrain.getPathPID();
		turnConstants = Robot.driveTrain.getTurnPID();

		P = pathConstants[0];
		D = pathConstants[1];
		proportionOfMaxVelocity = pathConstants[2];
		k_a = pathConstants[3];

		TurnP = turnConstants[0];
		TurnI = turnConstants[1];
		TurnD = turnConstants[2];*/


		try {
			TankModifier modifier = new TankModifier(traj).modify(RobotMap.TRACK_WIDTH);
			left = new DistanceFollower(modifier.getLeftTrajectory());
			right = new DistanceFollower(modifier.getRightTrajectory());
			left.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			right.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);			
			NAVXSource = () -> Robot.rps.getNavxAngle(); //negative to bring coordinates to superimpose
			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, FileName+"TurnPID",logPID);
			PathingLog = new Logger(FileName + "Log");
		} catch (Exception e) {
			prematureTermination = true;
		}
	}
	
	public VisionAlignment() {
		requires(Robot.driveTrain);
	}

	private boolean verifyPathWaypoints(double z, double x)	{
		if(z < Math.abs(x))	return false; //too tight
		if(z < 0.2) return false; //too close
		if(((Double) x).isNaN() || ((Double) z).isNaN()) return false; //vision target cannot be seen
		return true;
	}
	@Override
	protected void initialize() {
		double x_dist = -Robot.rps.getXDisplacementEditedForCameraPositionMeters();
		double z_dist = Robot.rps.getZDisplacementEditedForCameraPositionMeters();

		if(!verifyPathWaypoints(z_dist, x_dist))	{
			prematureTermination = true;
			return;
		}
		if(!prematureTermination)	{
			Waypoint[] points 	= new Waypoint[]{
					new Waypoint(0, 0, 0),
					new Waypoint(z_dist, x_dist, 0)
			};
			try {
				System.out.println("Generating Trajectory");
				Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
						Trajectory.Config.SAMPLES_LOW, 0.02, proportionOfMaxVelocity*RobotMap.MAX_VELOCITY, 2.0, 60.0);
				Trajectory trajectory = Pathfinder.generate(points, config);
				System.out.println("Trajectory Length: " + trajectory.length());
				isFinishedDebouncer = new Debouncer(10);
				initPathExecuter(trajectory, "Vision", true);
				turnPID.resetPID();
				Robot.rps.reset();
				Robot.driveTrain.resetEncoders();
				left.reset();
				right.reset();
				Robot.SystemLog.writeWithTimeStamp("Path Executer Initialized: Angle=" + Robot.rps.getNavxAngle());
				PathingLog.writeNewData("Time, Desired Heading, Desired Left Position, Desired Right Position, Heading, Left Position, Right Posiion, LeftPower, RightPower");

			} catch (Exception e) {
				prematureTermination = true;
			}
		}
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
		LeftMotorOutput = l + turn;
		RightMotorOutput = r - turn;
		if(log && !left.isFinished() && !right.isFinished())	{
				PathingLog.writeNewData(
					Timer.getFPGATimestamp()+","+desired_heading+","+left_s.position+","+right_s.position+","+
					turnPID.getInput()+","+Robot.driveTrain.getLeftEncoderDistanceMeters()+","+Robot.driveTrain.getRightEncoderDistanceMeters()+","+
					LeftMotorOutput+","+RightMotorOutput);
		}
	}
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(!prematureTermination)	{
			updateMotorOutputs(Robot.driveTrain.getLeftEncoderDistanceMeters(), Robot.driveTrain.getRightEncoderDistanceMeters());
			Robot.driveTrain.rawMotorOutput(LeftMotorOutput, RightMotorOutput);
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
		if(!prematureTermination)	{
			turnPID.resetPID();
		}
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
