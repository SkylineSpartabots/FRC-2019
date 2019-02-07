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

public class RollIn extends Command {

  private double power = 0;
  private double rTrigger;
  private double lTrigger;

  public RollIn() {
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
    //power = 0.75;       if you choose to make it set to a specific power, then another command has to be made for RollOut, this way it's just in the joystick's triggers.
    rTrigger = Robot.oi.driveStick.getRawAxis(OI.Axis.RTrigger.getBtnNumber());
    lTrigger = Robot.oi.driveStick.getRawAxis(OI.Axis.LTrigger.getBtnNumber());
    power = rTrigger - lTrigger;
    Robot.intake.setIntakePower(power);
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
