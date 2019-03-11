/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive_controls;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.controllers.Xbox;

public class DriveWithVision extends Command {

  double forwardClipAmount, turnClipAmount;

  double vibrateProportion;

  public DriveWithVision() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.driveTrain.tankDrive(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    vibrateProportion = Robot.driveTrain.getVibrate();
    double turn = Robot.oi.driveStick.getRX();
		double forward = Robot.oi.driveStick.getLY();
		if(Robot.elevator.getElevatorEncoderOutput() >= 420 && Robot.elevator.getElevatorEncoderOutput() <= 700){
			forwardClipAmount = 0.7;
			turnClipAmount = 0.75;
		} else if(Robot.elevator.getElevatorEncoderOutput() >= 700){
			forwardClipAmount = 0.5;
			turnClipAmount = 0.6;
		} else{
			forwardClipAmount = 1;
			turnClipAmount = 1;
    }
    
    if(Robot.rps.getZDisplacementToVisionTargetRawInches() < 70 && Robot.rps.getZDisplacementToVisionTargetRawInches() > 10){
      double xDisplacement = Robot.rps.getXDisplacementToVisionTargetRawInches();
      if(xDisplacement > 0){
        Robot.oi.driveStick.vibrateRight(Math.abs(xDisplacement) * vibrateProportion);
        Robot.oi.driveStick.vibrateLeft(0);
      } else if(xDisplacement < 0){
        Robot.oi.driveStick.vibrateLeft(Math.abs(xDisplacement) * vibrateProportion);
        Robot.oi.driveStick.vibrateRight(0);
      } else {
        Robot.oi.driveStick.vibrate(0);
      }
    }

		Robot.driveTrain.arcadeDrive(-Xbox.clipAxis(forward, forwardClipAmount), 0.75*(Xbox.clipAxis(turn, turnClipAmount)));
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
