/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.util.Debouncer;
import frc.robot.util.RawInput;

/**
 * This logic controls the hatch mechanism Gamepad Controls (Second Stick):
 * Right Bumper -> release hatch Left Bumper -> grasp hatch
 * 
 * Also implements the limit swtich, if the limit swich and changed from a false
 * to true state, grasp hatch
 */

public class HatchMechanismControl extends Command {

  private boolean prevHatchState, currentHatchState;
  private RawInput hatchRawInput;
  private Debouncer hatchDebouncer;

  public HatchMechanismControl() {
    requires(Robot.hatchMechanism);

    hatchRawInput = () -> Robot.hatchMechanism.getHatchState();
    hatchDebouncer = new Debouncer(hatchRawInput, 5);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    currentHatchState = hatchDebouncer.compute();

    // Controls hatch mechanism taking input from the bumpers and the limit switch
    if (Robot.oi.secondStick.buttonRBumper.get()) {
      Robot.hatchMechanism.releaseHatch();
    } else {
      if (Robot.oi.secondStick.buttonLBumper.get()) {
        Robot.hatchMechanism.graspHatch();
      } else if (currentHatchState && !prevHatchState) { // only take action when the state changes from true to false
        Robot.hatchMechanism.graspHatch();
      }
    }

    prevHatchState = currentHatchState;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
