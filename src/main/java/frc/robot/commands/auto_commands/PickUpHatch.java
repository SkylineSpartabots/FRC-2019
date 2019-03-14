/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.basic_commands.GraspHatch;
import frc.robot.commands.basic_commands.ReleaseHatch;

public class PickUpHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickUpHatch() {
    addSequential(new ReleaseHatch());
    addSequential(new AlignAndPathToTarget());
    addSequential(new GraspHatch());
  }
}
