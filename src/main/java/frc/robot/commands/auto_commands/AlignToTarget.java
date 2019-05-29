/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.SimplePID;
import frc.robot.util.vision.Limelight;

public class AlignToTarget extends Command {

  private final double DESIRED_TARGET_AREA = 0;
  private SimplePID distancePID;
  private DoubleSupplier distanceSupplier;
  private double kP = 0.001;
  private double kI = 0.00;
  private double kD = 0.00;


  public AlignToTarget() {
    requires(Robot.driveTrain);
    distanceSupplier = () -> Limelight.getTargetArea();
    distancePID = new SimplePID(distanceSupplier, DESIRED_TARGET_AREA, kP, kI, kD);
    distancePID.setOutputLimits(-0.3, 0.3);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.arcadeDrive(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.oi.driveStick.vibrate(0.5);

    Robot.driveTrain.arcadeDrive(distancePID.compute(), Robot.limelight.getSteerCorrection());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Limelight.isTargetVisible() || !Robot.oi.driveStick.buttonB.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.arcadeDrive(0, 0);
    Robot.oi.driveStick.vibrate(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }





}
