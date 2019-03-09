package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.Logger;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class VisionAllignment extends Command {
	/*
	private final double P = 1.1;
	private final double D = 0;
	private final double k_a = 0.02;

	private final double TurnP = 0.027;
	private final double TurnI = 0.0;
	private final double TurnD = 0.00;

	*/
	public boolean prematureTermination = false;
	public boolean log = true;
	private final double proportionOfMaxVelocity = 0.15;

	private final double P = 1.0;
	private final double D = 0;
	private final double k_a = 0.02;

	private final double TurnP = 0.027;
	private final double TurnI = 0.0;
	private final double TurnD = 0.002;


	private DistanceFollower left, right;
	private PIDSource NAVXSource;
	private SimplePID turnPID;
	private Logger PathingLog;

	private double LeftMotorOutput = 0;
	private double RightMotorOutput = 0;

	public void initPathExecuter(Trajectory traj, String FileName, boolean logPID) {
		try {
			TankModifier modifier = new TankModifier(traj).modify(RobotMap.TRACK_WIDTH);
			left = new DistanceFollower(modifier.getLeftTrajectory());
			right = new DistanceFollower(modifier.getRightTrajectory());
			left.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			right.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);			
			NAVXSource = new PIDSource() {
				public double getInput() {
					return Robot.rps.getNavxAngle();
				}
			};
			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, FileName+"TurnPID",logPID);
			PathingLog = new Logger(FileName + "Log");
		} catch (Exception e) {
			prematureTermination = true;
		}
	}
	
	public VisionAllignment() {
		requires(Robot.driveTrain);
	}

	private boolean verifyPathWaypoints(double z, double x)	{
		if(z < x)	return false; //too tight
		if(z < 0.2) return false; //too close 
		if(x < -20) return false; //vision target cannot be seen
		if(z < -20) return false; //vision target cannot be seen
		return true;
	}
	@Override
	protected void initialize() {
		double x_dist = Robot.rps.getXDisplacementEditedForCameraPositionMeters();
		double z_dist = Robot.rps.getZDisplacementEditedForCameraPositionMeters();

		if(!verifyPathWaypoints(z_dist, x_dist))	{
			prematureTermination = true;
			return;
		}
		
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
			initPathExecuter(trajectory, "Vision", true);
		} catch (Exception e) {
			prematureTermination = true;
		}

		turnPID.resetPID();
		Robot.rps.reset();
		Robot.driveTrain.resetEncoders();
		left.reset();
		right.reset();
		Robot.SystemLog.writeWithTimeStamp("Path Executer Initialized: Angle=" + Robot.rps.getNavxAngle());
		PathingLog.writeNewData("Time, Desired Heading, Desired Left Position, Desired Right Position, Heading, Left Position, Right Posiion, LeftPower, RightPower");
	}

	public void updateMotorOutputs(double LeftEncoderDistance, double RightEncoderDistance) {
		Segment left_s = left.getSegment();
		Segment right_s = right.getSegment();
		double l = left.calculate(LeftEncoderDistance);
		double r = right.calculate(RightEncoderDistance);
		double desired_heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));
		turnPID.setSetpoint(desired_heading);
		//System.out.println(desired_heading);
		double turn = turnPID.compute();
		LeftMotorOutput = l + turn;
		RightMotorOutput = r - turn;
		if(log)	{
			try{
				PathingLog.writeNewData(
					Timer.getFPGATimestamp()+","+desired_heading+","+left_s.position+","+right_s.position+","+
					turnPID.getInput()+","+Robot.driveTrain.getLeftEncoderDistanceMeters()+","+Robot.driveTrain.getRightEncoderDistanceMeters()+","+
					LeftMotorOutput+","+RightMotorOutput);
			}catch(Exception e)	{
				System.out.println("Should not be in here anymore");
				//pass this error
			}
		}
	}
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(!prematureTermination)	{
			updateMotorOutputs(Robot.driveTrain.getLeftEncoderDistanceMeters(), Robot.driveTrain.getRightEncoderDistanceMeters());
			Robot.driveTrain.rawMotorOutput(LeftMotorOutput, RightMotorOutput);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return prematureTermination || (left.isFinished() && right.isFinished()) || Math.abs(Robot.oi.driveStick.getLY()) > 0.1;
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