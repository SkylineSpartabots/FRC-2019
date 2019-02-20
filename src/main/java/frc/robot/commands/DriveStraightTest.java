/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveStraightTest extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DriveStraightTest() {
    addSequential(new PerfectlyStraightDrive(100, 0.5, 0.5));
  }
}
