package frc.robot.commands.drive_controls;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.VibrateControllers;
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

  private boolean isBottom;
  private Elevator.ElevatorPosition elevatorPosition;

  private VibrateControllers vibrateControllers;
  private boolean hasVibrated;
  private double[] elevatorConstants;

  /**
   * Specify an elevator position using the "Elevator Position" enum and the robot
   * will autonomously proceed to that position
   * 
   * @param elevatorPosition
   */
  public ElevatorToPosition(Elevator.ElevatorPosition elevatorPosition) {
    requires(Robot.elevator);

    elevatorSource = () -> Robot.elevator.getElevatorEncoderOutput();
    this.elevatorPosition = elevatorPosition;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    elevatorConstants = Robot.elevator.getElevatorConstants();

    elevatorTarget = this.elevatorPosition.getPosition();

    elevatorPID = new SimplePID(elevatorSource, elevatorTarget, elevatorConstants[0], elevatorConstants[1], elevatorConstants[2], "ElevatorPositionPID", false);
    
    isBottom = elevatorTarget == Elevator.ElevatorPosition.DOWN.getPosition();

    if (isBottom) {
      elevatorPID.setOutputLimits(-0.3, 0.60);
    } else {
      elevatorPID.setOutputLimits(0, 0.60);
    }

    Robot.elevator.setPower(0);
    elevatorPID.setSetpoint(elevatorPosition.getPosition());
    isFinished = false;
    hasVibrated = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
    if(isFinished){
      if(!hasVibrated){
        try {
          vibrateControllers = new VibrateControllers(0.3, Robot.oi.secondStick);
          vibrateControllers.start();
        } finally {
          vibrateControllers.close();
        }
        hasVibrated = true;
      }
      Robot.elevator.stall();

    } else {
      output = elevatorPID.compute();
      error = elevatorPID.getError();
      Robot.elevator.setPower(output);
      /*
       * logic for ending the the pid loop, if it is within a certain range for a
       * period of time meaning its velocity isn't too high, then stall the motors
       */
      if (Math.abs(error) <= ELEVATOR_THRESHOLD) {
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
    return Math.abs(Robot.oi.secondStick.getRY()) > 0.1;
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