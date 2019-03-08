/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.controllers.Xbox;

/**
 * Command the vibrates a given controller for a set period of time
 */
public class VibrateControllers extends Command {
  
  Timer timer;
  Xbox[] controllers;

  double timeOutSecs;

  public VibrateControllers(double timeOutSecs, Xbox... controllers) {
    timer = new Timer();
    this.controllers = controllers;
    this.timeOutSecs = timeOutSecs;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    stopVibrate();

    timer.reset();
    timer.start();

    timeOutSecs += timer.get();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      vibrate();
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (timer.get() >= timeOutSecs);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    stopVibrate();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  private void stopVibrate(){
    for(int i = 0; i < controllers.length; i++){
      controllers[i].stopVibrate();
    }
  }

  private void vibrate(){
    for(int i = 0; i < controllers.length; i++){
      controllers[i].vibrate();
    }
  }



}
