/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto_commands.PathExecuter;
import frc.robot.commands.auto_commands.StopDriveTrain;
import frc.robot.commands.auto_commands.TurnDegrees;
import frc.robot.commands.auto_commands.VisionAlignment;
import jaci.pathfinder.Waypoint;

public class PathingTest extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathingTest() {
    addSequential(new TurnDegrees(180, 4));
    //addSequential(new StopDriveTrain());
    //addSequential(new VisionAlignment());
  }
}
