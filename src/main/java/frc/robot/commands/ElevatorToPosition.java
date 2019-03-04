package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;

public class ElevatorToPosition extends Command {

	private final static int ELEVATOR_THRESHOLD = 5;
	private final static int CLOCK_MAX = 10;

	private SimplePID elevatorPID;
	private PIDSource elevatorSource;
	private int elevatorTarget;	
	private double output;
	private double error;
	private int clockCounter = 0;
	private boolean isFinished = false;

	private int countsPerSecond = 350;
	private Timer timer;

	private double kP = 0.04; // TODO: add values
	private double kI = 0.005;
	private double kD = 0;

	//Setpoint ramp
	private double startTime;
	private double elapsedTime;
	private double setpoint;
	/**
	 * Specify an elevator position using the "Elevator Position" enum and the robot
	 * will autonomously proceed to that position
	 * 
	 * @param elevatorPosition
	 */
	public ElevatorToPosition(Elevator.ElevatorPosition elevatorPosition) {
		requires(Robot.elevator);

		elevatorSource = () -> Robot.elevator.getElevatorEncoderOutput();

		// returns value whether it is in the cargo or hatch position
		elevatorTarget = elevatorPosition.getPosition();

		timer = new Timer();
		elevatorPID = new SimplePID(elevatorSource, elevatorTarget, kP, kI, kD, "ElevatorPositionPID",false);
		elevatorPID.setOutputLimits(-0.1, 0.55);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.setPower(0);
		elevatorPID.resetPID();
		elevatorPID.setSetpoint(0);
		startTime = Timer.getFPGATimestamp();
		setpoint = 0;
		isFinished = false;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(isFinished)	{
			Robot.elevator.setStallPower();
		}	else	{
			elapsedTime = Timer.getFPGATimestamp()-startTime;
			if(elapsedTime*countsPerSecond < elevatorTarget)	{	
				setpoint = elapsedTime*countsPerSecond;		
				elevatorPID.setSetpointRamp(setpoint);
			}
			output = elevatorPID.compute();
			error = elevatorPID.getError();
			System.out.println(output + "," + error + "," + setpoint + "," + elevatorPID.getInput());
			Robot.elevator.setPower(output);
			/**
			 * logic for ending the the command, if it is within a certain range for a
			 * period of time meaning its velocity isn't too high, then end the command
			 */
			if (Math.abs(elevatorPID.getInput()-elevatorTarget) < ELEVATOR_THRESHOLD) {
				System.out.println("Counting: " + elevatorTarget);
				clockCounter++;
				if (clockCounter > CLOCK_MAX) {
					isFinished = true;
				}
			} else {
				clockCounter = 0;
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Math.abs(Robot.oi.secondStick.getRY())>0.1;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		elevatorPID.resetPID();
	}

	// Called when another command which requires one or more of the same subsystems
	// is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}