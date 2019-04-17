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

public class LeftDoubleCargoShipAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftDoubleCargoShipAuto() {
    addSequential(new WaitForStart());

    //Place First Hatch
    addSequential(new PerfectlyStraightDrive(70, 0.6, 0.6));//in inches
    addParallel(new GraspHatch());
    addSequential(new StopDriveTrain());
    addParallel(new SlideHatchOut());
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());
    addSequential(new Wait(100));
    addSequential(new EncoderDrive(-40, -0.6, -0.6));
    addParallel(new SlideHatchIn());
    

    //Grasp Second Hatch
    addSequential(new TurnDegrees(150, 2));
    addParallel(new GraspHatch());
    addSequential(new PerfectlyStraightDrive(160, 0.9, 0.9));
    addSequential(new TurnDegrees(180, 2));
    addSequential(new PerfectlyStraightDrive(50, 0.8, 0.8));
    addSequential(new StopDriveTrain());
    addParallel(new ReleaseHatch());
    addSequential(new VisionAlignment());
    addParallel(new SlideHatchOut());
    addSequential(new GraspHatch());
    addSequential(new Wait(100));

    //Drive and Place Second Hatch
    addSequential(new EncoderDrive(-84, -0.7, -0.7));
    addParallel(new SlideHatchIn());
    addSequential(new TurnDegrees(330, 2));
    addSequential(new PerfectlyStraightDrive(176, 0.9, 0.9));
    addSequential(new TurnDegrees(270, 2));
    addSequential(new VisionAlignment());
    addParallel(new SlideHatchOut());
    addSequential(new ReleaseHatch());
    addSequential(new Wait(100));
    addSequential(new EncoderDrive(-20, -20, -0.6));
    addParallel(new SlideHatchIn());
    addSequential(new GraspHatch());
  }
}
