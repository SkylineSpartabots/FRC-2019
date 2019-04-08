/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.basic_commands.ReleaseHatch;
import frc.robot.commands.basic_commands.SlideHatchOut;
import frc.robot.commands.Wait;
import frc.robot.commands.auto_commands.*;

public class AlignAndPathToDepot extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AlignAndPathToDepot() {
    addSequential(new PerfectlyStraightDrive(60, 0.5, 0.5));//in inches
    
    addSequential(new StopDriveTrain());
    addSequential(new Wait(400));
    addParallel(new SlideHatchOut());
    //addSequential(new TurnDegreesVision(2));
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());
    
  }
}
