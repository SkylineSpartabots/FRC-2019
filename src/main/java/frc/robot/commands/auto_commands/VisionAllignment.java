package frc.robot.commands.auto_commands;

import java.io.File;

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
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * @author Neil (the great) Hazra
 */
public class VisionAllignment extends Command {
	/*
	private final double P = 1.1;
	private final double D = 0;
	private final double k_a = 0.02;
	private final double TurnP = 0.027;
	private final double TurnI = 0.0;
	private final double TurnD = 0.00;
	*/
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
	public void initPathExecuter(String left_s, String right_s, String FileName) {
		try {			
			File left_f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + left_s);
			File right_f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + right_s);
			
			left = new DistanceFollower(Pathfinder.readFromCSV(left_f));
			right = new DistanceFollower(Pathfinder.readFromCSV(right_f));
			left.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			right.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			
			NAVXSource = new PIDSource() {
				public double getInput() {
					return Robot.rps.getNavxAngle();
				}
			};
			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, FileName+"TurnPID",true);
			PathingLog = new Logger(FileName + "Log");
		} catch (Exception e) {			
		}
	}
	public void initPathExecuter(Trajectory traj, String FileName) {
		try {
			TankModifier modifier = new TankModifier(traj).modify(RobotMap.TRACK_WIDTH);
			File _f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + "lefttraj");
			Pathfinder.writeToCSV(_f, modifier.getLeftTrajectory());
			File f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + "righttraj");
			Pathfinder.writeToCSV(f, modifier.getRightTrajectory());
			left = new DistanceFollower(modifier.getLeftTrajectory());
			right = new DistanceFollower(modifier.getRightTrajectory());
			left.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			right.configurePIDVA(P, 0.0, D, 1.0/RobotMap.MAX_VELOCITY, k_a);
			
			NAVXSource = new PIDSource() {
				public double getInput() {
					return Robot.rps.getNavxAngle();
				}
			};
			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, FileName+"TurnPID",true);
			PathingLog = new Logger(FileName + "Log");
		} catch (Exception e) {
			
		}
	}
	
	public VisionAllignment() {
		requires(Robot.driveTrain);

	}
	public void updateMotorOutputs(double LeftEncoderDistance, double RightEncoderDistance) {
		double l = left.calculate(LeftEncoderDistance);
		double r = left.calculate(RightEncoderDistance);
		double desired_heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));
		turnPID.setSetpoint(desired_heading);
		System.out.println(desired_heading);
		double turn = turnPID.compute();
		LeftMotorOutput = l + turn;
		RightMotorOutput = r - turn;
		
		try{
			PathingLog.writeNewData(
				Timer.getFPGATimestamp()+","+desired_heading+","+left.getSegment().position+","+right.getSegment().position+","+
				turnPID.getInput()+","+Robot.driveTrain.getLeftEncoderDistanceMeters()+","+Robot.driveTrain.getRightEncoderDistanceMeters()+","+
				LeftMotorOutput+","+RightMotorOutput);
		}catch(Exception e)	{

		}
		

			
		//PathingLog.flushLogData();
		//Robot.SystemLog.flushLogData();
	}

	/**
	 * All units in metric meters
	 */
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//this needs to be moved to init
		double x_dist;
		if(Robot.rps.getXDisplacementToVisionTarget()>0)	{
			x_dist = Robot.rps.getXDisplacementToVisionTarget()*1.4;
		}	else {
			x_dist = Robot.rps.getXDisplacementToVisionTarget()*1.1;
		}
		 
		Waypoint[] points 	= new Waypoint[]{
				new Waypoint(0, 0, 0),
				new Waypoint(Robot.rps.getZDisplacementToVisionTarget()*0.0254-0.45, x_dist*0.0254, 0)
		}; 
		try {
			System.out.println("Generating Trajectory");
			Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_LOW, 0.02, 0.15*RobotMap.MAX_VELOCITY, 2.0, 60.0);
			
			Trajectory trajectory = Pathfinder.generate(points, config);
			System.out.println("Trajectory Length: " + trajectory.length());
			Robot.SystemLog.writeNewData("PathExecuter: Trajectory Path Saved To File");
			initPathExecuter(trajectory, "Vision");
		} catch (Exception e) {
			Robot.SystemLog.writeNewData("PathExecuter Line 39: Error Creating Trajectory Path"  +e.getMessage());
		}
		turnPID.resetPID();
		Robot.rps.reset();
		Robot.driveTrain.resetEncoders();
		left.reset();
		right.reset();
		Robot.SystemLog.writeWithTimeStamp("Path Executer Initialized: Angle=" + Robot.rps.getNavxAngle());
		PathingLog.writeNewData("Time, Desired Heading, Desired Left Position, Desired Right Position, Heading, Left Position, Right Posiion, LeftPower, RightPower");
    Robot.oi.driveStick.vibrate(0.3);
  }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		updateMotorOutputs(Robot.driveTrain.getLeftEncoderDistanceMeters(), Robot.driveTrain.getRightEncoderDistanceMeters());
		Robot.driveTrain.rawMotorOutput(LeftMotorOutput, RightMotorOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (left.isFinished() && right.isFinished()) || Robot.rps.getZDisplacementToVisionTarget() < 20;
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
    Robot.oi.driveStick.stopVibrate();
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		Robot.SystemLog.writeWithTimeStamp("Path Executer Interrupted");
		end();
	}
}