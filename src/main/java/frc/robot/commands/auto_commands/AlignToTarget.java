/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_commands;

import java.util.function.DoubleSupplier;

import edu.wpi.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.SimplePID;
import frc.robot.util.TelemetryUtil;
import frc.robot.util.TelemetryUtil.PrintStyle;
import frc.robot.util.vision.Limelight;

public class AlignToTarget extends Command {

  
  private SimplePID distancePID, anglePID;
  private DoubleSupplier distanceSupplier, angleSupplier;
  private double distancekP = 0.001;
  private double distancekI = 0.00;
  private double distancekD = 0.00;

  private double anglekP = 0.001;
  private double anglekI = 0;
  private double anglekD = 0;


  public AlignToTarget() {
    requires(Robot.driveTrain);

    distanceSupplier = () -> Limelight.getTargetArea();
    distancePID = new SimplePID(distanceSupplier, Limelight.DESIRED_TARGET_AREA, distancekP, distancekI, distancekD);
    distancePID.setOutputLimits(-0.2, 0.4);

    angleSupplier = () -> Limelight.getHorizontalOffset();
    anglePID = new SimplePID(angleSupplier, 0, anglekP, anglekI, anglekD);
    anglePID.setOutputLimits(-0.3, 0.3);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.tankDrive(0, 0);
    Robot.oi.driveStick.vibrate(0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    TelemetryUtil.print("auto-aligning", PrintStyle.INFO);
    double forwardPower = distancePID.compute();
    double steerCorrection = anglePID.compute();
    Robot.driveTrain.tankDrive(forwardPower + steerCorrection, forwardPower-steerCorrection);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !(Limelight.isTargetVisible() && Robot.oi.driveStick.buttonA.get());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.tankDrive(0, 0);
    Robot.oi.driveStick.vibrate(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }





}
