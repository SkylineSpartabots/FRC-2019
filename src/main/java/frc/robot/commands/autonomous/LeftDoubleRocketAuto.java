/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.auto_commands.*;
import frc.robot.commands.basic_commands.*;
import jaci.pathfinder.Waypoint;


public class LeftDoubleRocketAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftDoubleRocketAuto() {
    Waypoint[] toFrontRocket = {
      new Waypoint(0, 0, 0),
      new Waypoint(1, 1.75, 30)
    };

    Waypoint[] toBackRocket = {
      new Waypoint(0, 0, 0),
      new Waypoint(3.25, 0, 0),
      new Waypoint(4.25, -0.85, -40),
      new Waypoint(5.5, -1.35, 0),
      new Waypoint(6.5, -0.85, 80),
      new Waypoint(6, -0.2, 150)
    };

    //Place first hatch on the near side of rocket
    //addSequential(new PerfectlyStraightDrive(60, 0.5, 0.5));
    //addSequential(new PathExecuter(toFrontRocket, "To Front Rocket", false));
    addSequential(new TurnDegrees(-90, 4));
    //addSequential(new PerfectlyStraightDrive(84, 0.6, 0.6));
    //addSequential(new TurnDegrees(-45, 4));
    //addSequential(new StopDriveTrain());
    //addParallel(new SlideHatchOut());
    //addSequential(new VisionAlignment());
    //addSequential(new ReleaseHatch());
    //addSequential(new Wait(100));

    //Get second hatch from depot
    /*addSequential(new EncoderDrive(-15, -15, -0.6));
    addParallel(new SlideHatchIn());
    addSequential(new TurnDegrees(180, 3));
    addParallel(new GraspHatch());
    addSequential(new PerfectlyStraightDrive(140, 0.8, 0.8));
    addSequential(new StopDriveTrain());
    addParallel(new SlideHatchOut());
    addSequential(new VisionAlignment());
    addSequential(new GraspHatch());
    addSequential(new Wait(100));

    //Place second hatch on far side of rocket
    addSequential(new EncoderDrive(-15, -15, -0.5));
    addParallel(new SlideHatchIn());
    addSequential(new TurnDegrees(0, 2));
    addSequential(new PathExecuter(toBackRocket, "To Back Rocket", false));
    addSequential(new StopDriveTrain());
    addParallel(new SlideHatchOut());
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());
    addSequential(new Wait(100));
    addSequential(new EncoderDrive(-15, -15, -0.5));
    addParallel(new SlideHatchIn());
    addSequential(new GraspHatch());*/
  }
}
