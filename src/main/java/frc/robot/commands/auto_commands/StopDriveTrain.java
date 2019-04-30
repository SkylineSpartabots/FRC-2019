/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StopDriveTrain extends Command {
  
  private double encoderDiff;
  private int clockCounter;
  private int clockMax;
  private int encoderDiffThreshold = 0;
  private double prevEncoderVal;
  
  public StopDriveTrain() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.setBrake();
    Robot.driveTrain.tankDrive(0, 0);
    clockCounter = 0;
    encoderDiff = 500;
    prevEncoderVal = Robot.driveTrain.getLeftEncoderDistanceInches();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(clockCounter < clockMax){
      clockCounter++;
    } else {
      encoderDiff = Robot.driveTrain.getLeftEncoderDistanceInches() - prevEncoderVal;
      prevEncoderVal = Robot.driveTrain.getLeftEncoderDistanceInches();
      clockCounter = 0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return encoderDiff <= encoderDiffThreshold 
      || Math.abs(Robot.oi.driveStick.getRX()) > 0.1 || Math.abs(Robot.oi.driveStick.getLY()) > 0.1;
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
