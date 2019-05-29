package frc.robot.commands.auto_commands;

import frc.robot.Robot;
import frc.robot.util.SimplePID;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;



public class PerfectlyStraightDrive extends Command {


  private double distanceInches;
  private boolean direction;

  private double leftPower, rightPower;

  private double rightTarget, leftTarget;

	private double angle;

  private double output = 0;
 
	private DoubleSupplier gyroSource;
	private SimplePID turnPID;

  public double kP = 0.020;
	public double kI = 0.00001;
	public double kD = 0.00135;

  public PerfectlyStraightDrive(double distanceInches, double leftPower, double rightPower){
    
    this.distanceInches = distanceInches;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    requires(Robot.driveTrain);
    angle = Robot.navx.getAngle();
    
		gyroSource = () -> Robot.navx.getAngle();

		turnPID = new SimplePID(gyroSource, this.angle, kP, kI, kD);
		turnPID.setOutputLimits(-0.5, 0.5);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rightTarget = this.distanceInches + Robot.driveTrain.getRightEncoderDistanceInches();
    leftTarget =  this.distanceInches + Robot.driveTrain.getLeftEncoderDistanceInches();
    Robot.driveTrain.tankDrive(0, 0);
    this.direction = distanceInches > 0;
    turnPID.resetPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    output = turnPID.compute();
    Robot.driveTrain.tankDrive(leftPower + output, rightPower - output);
  
    System.out.println(output);
  

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(direction){
      if(leftTarget <= Robot.driveTrain.getLeftEncoderDistanceInches() || rightTarget <= Robot.driveTrain.getRightEncoderDistanceInches()){
        turnPID.resetPID();
        return true;
      } else{
        return false;
      }
        
    } else{
      if(leftTarget >= Robot.driveTrain.getLeftEncoderDistanceInches() || rightTarget >= Robot.driveTrain.getRightEncoderDistanceInches()){
        turnPID.resetPID();
        return true;
      } else{
        return false;
      }
    }
      
    
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