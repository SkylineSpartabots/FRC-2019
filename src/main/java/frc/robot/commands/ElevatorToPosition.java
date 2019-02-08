/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;

public class ElevatorToPosition extends Command {

  public enum ElevatorPositions{
      CARGO_SHIP,
      ROCKET_FIRST,
      ROCKET_SECOND, 
      ROCKET_THIRD;
    }
  

  private enum ElevatorControlPositions{
      DOWN_POSITION (0), //rocket first level hatch and cargo ship hatch
      CARGO_SHIP_CARGO (10),
      ROCKET_SHIP_FIRST_CARGO (100),
      ROCKET_SHIP_SECOND_HATCH (1000),
      ROCKET_SHIP_SECOND_CARGO (10000),
      ROCKET_SHIP_THIRD_HATCH (100000),
      ROCKET_SHIP_THIRD_CARGO (1000000);

      private final int position;

      private ElevatorControlPositions(int position){
        this.position = position;
      }

      private int getPosition(){
        return this.position;
      }
    }

  

  public final static int ELEVATOR_THRESHOLD = 50;
  public final static int CLOCK_MAX = 5;;


  private SimplePID elevatorPID;
  private PIDSource elevatorSource;
  private int elevatorTarget;
  private double output;
  private double error;
  private int clockCounter = 0;
  private boolean isFinished = false;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  /**
   * Specify an elevator position using the "Elevator Position" enum and the robot will autonomously proceed to that position
   * @param elevatorPosition
   */

  public ElevatorToPosition(ElevatorPositions elevatorPosition) {
    requires(Robot.elevator);

  
    
    switch (elevatorPosition) {
      case CARGO_SHIP:
        if(Robot.intake.isCargo()){
          elevatorTarget = ElevatorControlPositions.CARGO_SHIP_CARGO.getPosition();
        } else{
          elevatorTarget = ElevatorControlPositions.DOWN_POSITION.getPosition();
        }
        break;

      case ROCKET_FIRST:
        if(Robot.intake.isCargo()){
          elevatorTarget = ElevatorControlPositions.ROCKET_SHIP_FIRST_CARGO.getPosition();
        } else{
          elevatorTarget = ElevatorControlPositions.DOWN_POSITION.getPosition();
        }
        break;

      case ROCKET_SECOND:
        if(Robot.intake.isCargo()){
          elevatorTarget = ElevatorControlPositions.ROCKET_SHIP_SECOND_CARGO.getPosition();
        } else{
          elevatorTarget = ElevatorControlPositions.ROCKET_SHIP_SECOND_HATCH.getPosition();
        }
        break;

      case ROCKET_THIRD:
        if(Robot.intake.isCargo()){
          elevatorTarget = ElevatorControlPositions.ROCKET_SHIP_THIRD_CARGO.getPosition();
        } else{
          elevatorTarget = ElevatorControlPositions.ROCKET_SHIP_THIRD_HATCH.getPosition();
        }
        break;
      
      default:
        elevatorTarget = ElevatorControlPositions.DOWN_POSITION.getPosition();
    }

    elevatorSource = new PIDSource(){
    
      @Override
      public double getInput() {
        return Robot.elevator.elevatorEncoder.getRaw();
      }
    };

    elevatorPID = new SimplePID(elevatorSource, elevatorTarget, kP, kI, kD, false, "ElevatorPID");
    elevatorPID.setOutputLimits(-1, 1);
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
