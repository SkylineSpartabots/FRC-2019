/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;

public class ElevatorToPosition extends Command {
  public final static int ELEVATOR_THRESHOLD = 50;
  public final static int CLOCK_MAX = 5;;

  private SimplePID elevatorPID;
  private PIDSource elevatorSource;
  private int elevatorTarget;
  private double output;
  private double error;
  private int clockCounter = 0;
  private boolean isFinished = false;

  private double kP = 0; //TODO: add values
  private double kI = 0;
  private double kD = 0;

  /**
   * Specify an elevator position using the "Elevator Position" enum and the robot will autonomously proceed to that position
   * @param elevatorPosition
   */

  public ElevatorToPosition(Elevator.ElevatorPositions elevatorPosition) {
    requires(Robot.elevator);

    elevatorSource = () -> Robot.elevator.elevatorEncoder.getDistance();

    elevatorTarget = elevatorPosition.getPosition(); //returns value whether it is in the cargo or hatch position

    elevatorPID = new SimplePID(elevatorSource, elevatorTarget, kP, kI, kD, false, "ElevatorPID");
    elevatorPID.setOutputLimits(-0.4, 0.4);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.setPower(0);
    elevatorPID.resetPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    output = elevatorPID.compute();
    error = elevatorPID.getError();

    Robot.elevator.setPower(output);

    /**
     * logic for ending the the command, if it is within a certain range for a period of time
     * meaning its velocity isn't too high, then end the command
     *  */
    if(error < ELEVATOR_THRESHOLD){  
      clockCounter++;
      if(clockCounter > CLOCK_MAX){
        isFinished = true;
      }
    } else{
      clockCounter = 0;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.setPower(0);
    elevatorPID.resetPID();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
