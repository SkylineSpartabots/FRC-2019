package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.util.PIDSource;
import frc.robot.util.SimplePID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class PerfectlyStraightDrive extends Command {

  

  private final static double COUNTS_PER_REV = 1000;
  private final static double WHEEL_DIAMETER = 4;
  private final static double DRIVE_WHEEL_REDUCTION = 1;
  private final static double COUNTS_PER_INCH = (COUNTS_PER_REV/DRIVE_WHEEL_REDUCTION)/(WHEEL_DIAMETER*Math.PI);

  private double distanceInches;
  private boolean direction;

  private double leftPower, rightPower;

  private double rightTarget, leftTarget;

	private double angle;
	private boolean isFinished = false;
	private double error;
	
  private double output = 0;
  private Timer timer;
	PIDSource NavxSource;
	SimplePID turnPID;

  public double kP = 0.020;
	public double kI = 0.00001;
	public double kD = 0.00135;



  public PerfectlyStraightDrive(double distanceInches, double leftPower, double rightPower){
    
    this.distanceInches = distanceInches;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    timer = new Timer();
    requires(Robot.driveTrain);
    angle = Robot.rps.getNavxAngle();
    
		NavxSource = new PIDSource(){
			@Override
			public double getInput() {
				return Robot.rps.getNavxAngle();
			}
		};

    SmartDashboard.putNumber("kP, Perfect Drive", kP);


    
		turnPID = new SimplePID(NavxSource, this.angle, kP, kI, kD, "DriveStraingTurnPID" ,true);
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
    Robot.driveTrain.tankDrive(leftPower - output, rightPower + output);
  
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