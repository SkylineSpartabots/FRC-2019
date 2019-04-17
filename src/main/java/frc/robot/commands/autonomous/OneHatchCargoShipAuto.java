/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto_commands.PerfectlyStraightDrive;
import frc.robot.commands.auto_commands.StopDriveTrain;
import frc.robot.commands.auto_commands.TurnDegrees;
import frc.robot.commands.auto_commands.VisionAlignment;
import frc.robot.commands.auto_commands.WaitForStart;
import frc.robot.commands.basic_commands.ReleaseHatch;
import frc.robot.commands.basic_commands.SlideHatchOut;

public class OneHatchCargoShipAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public OneHatchCargoShipAuto() {
    addSequential(new WaitForStart());
    addSequential(new PerfectlyStraightDrive(60, 0.5, 0.5));//in inches
    addSequential(new StopDriveTrain());
    addParallel(new SlideHatchOut());
    addSequential(new TurnDegrees(0, 2));
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());
  }
}
