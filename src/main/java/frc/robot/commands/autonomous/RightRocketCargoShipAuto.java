/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto_commands.*;
import frc.robot.commands.basic_commands.*;
import jaci.pathfinder.Waypoint;


public class RightRocketCargoShipAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightRocketCargoShipAuto() {
    //TODO: Transfer sequential to parallel logic of hatch mechanism
    //align with edge of hab level 1
    Waypoint[] toFrontRocket = {
      new Waypoint(0, 0, 0),
      new Waypoint(1, -1.75, -30)
    };


    addSequential(new PerfectlyStraightDrive(60, 0.5, 0.5));//in inches
    //addSequential(new PathExecuter(toFrontRocket, "To Front Rocket", false));
    addSequential(new StopDriveTrain());
    addParallel(new SlideHatchOut());
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());

    addSequential(new EncoderDrive(-40, -0.5, -0.5));
    addSequential(new SlideHatchIn());
    addSequential(new GraspHatch());
    addSequential(new TurnDegrees(180, 2));
    addSequential(new PerfectlyStraightDrive(390, 0.9, 0.9));
    addSequential(new StopDriveTrain());
    addSequential(new ReleaseHatch());
    addSequential(new SlideHatchOut());
    addSequential(new VisionAlignment());
    
    addSequential(new GraspHatch());
    addSequential(new SlideHatchIn());
    addSequential(new EncoderDrive(-84, -0.7, -0.7));
    addSequential(new TurnDegrees(-330, 2));
    addSequential(new PerfectlyStraightDrive(176, 0.9, 0.9));
    addSequential(new TurnDegrees(-270, 2));
    addSequential(new VisionAlignment());
    addParallel(new SlideHatchOut());
    addSequential(new ReleaseHatch());
    addSequential(new EncoderDrive(-20, -20, -0.6));
    addParallel(new SlideHatchIn());
    addSequential(new GraspHatch());
  }
}
