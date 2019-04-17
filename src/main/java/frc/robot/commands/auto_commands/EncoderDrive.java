package frc.robot.commands.auto_commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;



public class EncoderDrive extends Command {

  private double distanceInches;
  private double leftPower, rightPower;
  public boolean isForward = true;

  public EncoderDrive(double distanceInches, double leftPower, double rightPower){
    
    this.distanceInches = distanceInches;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    if(distanceInches<0) isForward = false;
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.resetEncoders();
    Robot.driveTrain.tankDrive(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.tankDrive(leftPower, rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isForward){
      if(Robot.driveTrain.getLeftEncoderDistanceInches() >= distanceInches){
        return true;
      }         
    } else{
      if(Robot.driveTrain.getLeftEncoderDistanceInches() <= distanceInches){
        return true;
      } 
    }    
    return Math.abs(Robot.oi.driveStick.getLY()) > 0.1 || Math.abs(Robot.oi.driveStick.getRX()) > 0.1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.tankDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }

}