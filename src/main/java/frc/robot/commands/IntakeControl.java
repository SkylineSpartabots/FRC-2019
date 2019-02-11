/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class IntakeControl extends Command {

  private double power = 0;
  private double rTrigger;
  private double lTrigger;

  public IntakeControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intake.setIntakePower(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    rTrigger = Robot.oi.driveStick.getRawAxis(OI.Axis.RTrigger.getAxisNumber());
    rTrigger = Robot.oi.clipDeadzone(rTrigger);

    lTrigger = Robot.oi.driveStick.getRawAxis(OI.Axis.LTrigger.getAxisNumber());
    lTrigger = Robot.oi.clipDeadzone(lTrigger);

    if(rTrigger > lTrigger){
      Robot.intake.setIntakePower(rTrigger);
    } else if(lTrigger < rTrigger){
      Robot.intake.setIntakePower(-lTrigger);
    } else{
      Robot.intake.setIntakePower(0);
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
    Robot.intake.setIntakePower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
