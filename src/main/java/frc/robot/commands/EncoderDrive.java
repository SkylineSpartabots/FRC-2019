package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class EncoderDrive extends Command {

  private double distanceMeters;
  private double leftPower, rightPower;
  private Timer timer; 
	private boolean isFinished = false;
  public boolean isForward = true;

  public EncoderDrive(double distanceMeters, double leftPower, double rightPower){
    
    this.distanceMeters = distanceMeters;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    if(distanceMeters<0) isForward = false;
    timer = new Timer();
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
      if(Robot.driveTrain.getLeftEncoderDistanceMeters() >= distanceMeters){
        return true;
      }         
    } else{
      if(Robot.driveTrain.getLeftEncoderDistanceMeters() <= distanceMeters){
        return true;
      } 
    }    
    return false;
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