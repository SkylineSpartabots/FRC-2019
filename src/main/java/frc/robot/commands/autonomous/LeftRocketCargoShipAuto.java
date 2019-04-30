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

public class LeftRocketCargoShipAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftRocketCargoShipAuto() {
    Waypoint[] toNoseCargoShip = {
      new Waypoint(0, 0, 0),
      new Waypoint(2.5, 0.3, 0)
    };

    Waypoint[] toHatchDepot = {
      new Waypoint(0, 0, 0),
      new Waypoint(3.15, 3.1, 180)
    };

    Waypoint[] toSideCargoShip = {
      new Waypoint(0, 0, 0),
      new Waypoint(2.0, 0, 0),
      new Waypoint(4.5, -2.1, 0),
      new Waypoint(5.5, -2.1, 0),
      new Waypoint(6.6, -1.45, -90)
    };

    //addSequential(new WaitForStart());

   /* //Place First Hatch
    addSequential(new PathExecuter(toNoseCargoShip, false, 0.5, "To Nose", false));//in inches
    addSequential(new StopDriveTrain());
    addSequential(new VisionAlignment());
    addSequential(new SlideHatchOut());
    addSequential(new Wait(200));
    addSequential(new ReleaseHatch());
    addSequential(new Wait(200));
    addSequential(new EncoderDrive(-40, -0.6, -0.6));
    addParallel(new SlideHatchIn());
    

    //Grasp Second Hatch
    addSequential(new TurnDegrees(-90, 2));
    addParallel(new GraspHatch());
    addSequential(new PathExecuter(toHatchDepot, false, 0.5, "To Hatch Depot", false));
    addSequential(new StopDriveTrain());
    addParallel(new ReleaseHatch());
    addSequential(new VisionAlignment());
    addSequential(new SlideHatchOut());
    addSequential(new Wait(200));
    addSequential(new GraspHatch());

    

    //Drive and Place Second Hatch
    addSequential(new PathExecuter(toSideCargoShip, true, 0.3, "To Side Cargo Ship", false));
    addParallel(new SlideHatchIn());
    addSequential(new StopDriveTrain());
    addSequential(new VisionAlignment());
    addParallel(new SlideHatchOut());
    addSequential(new ReleaseHatch());
    addSequential(new Wait(100));
    addSequential(new EncoderDrive(-20, -20, -0.6));
    addParallel(new SlideHatchIn());
    addSequential(new GraspHatch());*/
  }
}
