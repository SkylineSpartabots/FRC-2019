/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.VibrateControllers;
import frc.robot.commands.Wait;
import frc.robot.commands.auto_commands.*;
import frc.robot.commands.basic_commands.*;
import jaci.pathfinder.Waypoint;

public class LeftDoubleCargoShipAuto extends CommandGroup {
  
  /**
   * Started at the center of the hab
   */

  public LeftDoubleCargoShipAuto() {

    Waypoint[] toNoseCargoShip = {
      new Waypoint(0, 0, 0),
      new Waypoint(2, 0.3, 0)
    };

    Waypoint[] toHatchDepot = {
      new Waypoint(0, 0, 0),
      new Waypoint(3, 3, -90)

    };

    Waypoint[] toSideCargoShip = {
      new Waypoint(0, 0, 0),
      new Waypoint(4.0, 1.1, 0),
      new Waypoint(4.75, 1.1, 0),
      new Waypoint(5.65, 1.85, -90),
    };

    //addSequential(new WaitForStart());

    //Place First Hatch
    //addSequential(new VibrateControllers(1, Robot.oi.driveStick, Robot.oi.secondStick));
    addSequential(new PathExecuter(toNoseCargoShip, false, 3,  0.3, "To Nose", false));//in inches
    addSequential(new StopDriveTrain());
    addSequential(new VisionAlignment());
    addSequential(new SlideHatchOut());
    addSequential(new VibrateControllers(1, Robot.oi.driveStick, Robot.oi.secondStick));
    
    
    /*addSequential(new Wait(200));
    addSequential(new ReleaseHatch());
    addSequential(new Wait(200));
    addParallel(new SlideHatchIn());
    addSequential(new EncoderDrive(-30, -0.8, -0.8));
    
    
    

    //Grasp Second Hatch
    addParallel(new GraspHatch());
    addSequential(new TurnDegrees(-90, 2));
    addSequential(new PathExecuter(toHatchDepot, false, 3, 0.3, "To Hatch Depot", false));
    addSequential(new StopDriveTrain());
    addParallel(new ReleaseHatch());
    addSequential(new TurnDegrees(-180, 2));
    addSequential(new VisionAlignment());
    addSequential(new SlideHatchOut());
    addSequential(new Wait(200));
    addSequential(new GraspHatch());

    
    
    //Drive and Place Second Hatch
    addParallel(new SlideHatchIn());
    //addSequential(new EncoderDrive(-30, -0.8, -0.8));
    addSequential(new VibrateControllers(1, Robot.oi.driveStick, Robot.oi.secondStick));
    //addSequential(new TurnDegrees(0, 3));
    
    /*addSequential(new PathExecuter(toSideCargoShip, false, 0.5, "To Side Cargo Ship", false));
    addSequential(new StopDriveTrain());
    addParallel(new SlideHatchOut());
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());
    addSequential(new Wait(100));
    addSequential(new EncoderDrive(-20, -20, -0.6));
    addSequential(new GraspHatch());*/
  }
}
