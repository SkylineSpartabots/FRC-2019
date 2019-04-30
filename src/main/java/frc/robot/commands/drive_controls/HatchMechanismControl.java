/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive_controls;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * This logic controls the hatch mechanism Gamepad Controls (Second Stick):
 * Right Bumper -> release hatch Left Bumper -> grasp hatch
 * 
 * Also implements the limit swtich, if the limit swich and changed from a false
 * to true state, grasp hatch
 */

public class HatchMechanismControl extends Command {


  public HatchMechanismControl() {
    requires(Robot.hatchMechanism);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.oi.driveStick.buttonLBumper.get()){
      Robot.hatchMechanism.releaseHatch();
    } else if(Robot.oi.driveStick.buttonRBumper.get()){
      Robot.hatchMechanism.graspHatch();
    }

    if(Robot.isAuto){
      if (Robot.oi.secondStick.buttonLJoystick.get()) {
        Robot.hatchMechanism.slideOut();
      } else if(Robot.oi.secondStick.buttonRJoystick.get()){
        Robot.hatchMechanism.slideIn();
      }
    } else {
      if (Robot.oi.secondStick.buttonLJoystick.get()) {
        Robot.hatchMechanism.slideOut();
      } else {
        Robot.hatchMechanism.slideIn();
      }
    }
    


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
