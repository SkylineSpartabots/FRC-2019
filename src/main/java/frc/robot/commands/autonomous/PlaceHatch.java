/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto_commands.AllignAndPathToTarget;
import frc.robot.commands.auto_commands.EncoderDrive;
import frc.robot.commands.auto_commands.PathExecuter;
import jaci.pathfinder.Waypoint;

public class PlaceHatch extends CommandGroup {

  Waypoint[] toCargo = new Waypoint[]{
    new Waypoint(0, 0, 0),
    new Waypoint(2, -0.5, 0),
  };

  public PlaceHatch() {
    //addSequential(new PathExecuter(toCargo, "TestCargoPath", true));
    addSequential(new AllignAndPathToTarget());
    //addSequential(new );
    addSequential(new EncoderDrive(-0.6, -0.6, -0.6));
    //dispense 

  }
}
