package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class PathExecuter extends Command {

	private final double P = 20;
	private final double D = 0;
	private final double k_a = 2;

	private final double TurnP = 0.03;
	private final double TurnI = 0;
	private final double TurnD = 0.05;

	private Timer timer;

	private DistanceFollower left, right;
	private PIDSource NAVXSource;
	private SimplePID turnPID;
	private Logger PathingLog;

	private double LeftMotorOutput = 0;
	private double RightMotorOutput = 0;

	public void initPathExecuter(Trajectory traj, String FileName) {
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
			timer = new Timer();
			turnPID = new SimplePID(NAVXSource, 0, TurnP, TurnI, TurnD, false);
			PathingLog = new Logger(FileName + "-Log.txt");
		} catch (Exception e) {
			
		}
	}

	public PathExecuter(String FileName) {
		requires(Robot.driveTrain);
		try {
			File f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + FileName);
			Trajectory trajectory = Pathfinder.readFromFile(f);
			initPathExecuter(trajectory, FileName);
		} catch (Exception e) {
			
		}
	}

	public PathExecuter(Waypoint[] points, String FileName) {
		requires(Robot.driveTrain);
		try {
			System.out.println("Generating Trajectory");
			Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_FAST, 0.02, RobotMap.MAX_VELOCITY, 2.0, 60.0);
			
			Trajectory trajectory = Pathfinder.generate(points, config);
			System.out.println("Trajectory Length: " + trajectory.length());
			//System.out.println(trajectory.get(72).x);
			//System.out.println(trajectory.get(72).y);
			//File f = new File(RobotMap.AUTO_TRAJECTORY_PATH_LOCATIONS + FileName);
			//Pathfinder.writeToFile(f, trajectory);
			//Robot.SystemLog.writeNewData("PathExecuter: Trajectory Path Saved To File");
			initPathExecuter(trajectory, FileName);
		} catch (Exception e) {
			//Robot.SystemLog.writeNewData("PathExecuter Line 39: Error Creating Trajectory Path");
		}
	}

	public void updateMotorOutputs(double LeftEncoderDistance, double RightEncoderDistance) {
		double l = left.calculate(LeftEncoderDistance);
		double r = left.calculate(RightEncoderDistance);
		double desired_heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));
		turnPID.setSetpoint(desired_heading);
		double turn = turnPID.compute();
		System.out.println("LeftDesiredPosition" + left.getSegment().position + "ActualPosition:" + Robot.driveTrain.getLeftEncoderDistanceMeters());
		System.out.println("RightDesiredPosition" + right.getSegment().position + "ActualPosition:" + Robot.driveTrain.getRightEncoderDistanceMeters());
		
		System.out.println("Heading Error: " + turnPID.getError());
		LeftMotorOutput = l/100 - turn;
		RightMotorOutput = r/100 + turn;
		PathingLog.writeNewData(
				"Pathing Update: LeftMotorOutput: " + LeftMotorOutput + " RightMotorOutput: " + RightMotorOutput
						+ " DesiredHeading: " + desired_heading + " Actual Heading: " + Robot.rps.getNavxAngle());
	}

	/**
	 * All units in metric meters
	 */
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		turnPID.resetPID();
		Robot.rps.reset();
		Robot.driveTrain.resetEncoders();
		left.reset();
		right.reset();

		//Robot.SystemLog.writeNewData("Path Executer Initialized: Angle=" + Robot.rps.getNavxAngle());
		System.out.println("Path Executer Initialize \n\n Angle=" + Robot.rps.getNavxAngle());
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		updateMotorOutputs(Robot.driveTrain.getLeftEncoderDistanceMeters(), Robot.driveTrain.getRightEncoderDistanceMeters());
		System.out.println("PathExecuter Motor Power: " + LeftMotorOutput + ", " + RightMotorOutput);
		System.out.println("Encoder Values:" + Robot.driveTrain.getRightEncoderDistanceMeters() + ", " + Robot.driveTrain.getLeftEncoderDistanceMeters());
		SmartDashboard.putNumber("Heading", Robot.rps.getNavxAngle());
		//System.out.println(left.getSegment().x);
		
		
		Robot.driveTrain.rawMotorOutput(LeftMotorOutput, RightMotorOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return left.isFinished() && right.isFinished();
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
		//Robot.SystemLog.writeNewData("Path Executer Interrupted");
		end();
	}
}