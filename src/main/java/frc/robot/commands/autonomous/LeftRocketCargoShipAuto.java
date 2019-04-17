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

public class LeftRocketCargoShipAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftRocketCargoShipAuto() {
    addSequential(new WaitForStart());


    addSequential(new PerfectlyStraightDrive(40, 0.5, 0.5));//in inches
    addSequential(new TurnDegrees(30, 2));
    addSequential(new PerfectlyStraightDrive(290, 0.9, 0.9));
    addSequential(new StopDriveTrain());
    addSequential(new SlideHatchOut());
    addSequential(new VisionAlignment());
    addSequential(new ReleaseHatch());


    addSequential(new EncoderDrive(-40, -0.5, -0.5));
    addSequential(new SlideHatchIn());
    addSequential(new GraspHatch());
    addSequential(new TurnDegrees(180, 2));
    
    addSequential(new PerfectlyStraightDrive(390, 0.9, 0.9));
    addSequential(new StopDriveTrain());
    addSequential(new VisionAlignment());
    addParallel(new ReleaseHatch());
    addParallel(new SlideHatchOut());
    addSequential(new GraspHatch());
    addSequential(new SlideHatchIn());
    addSequential(new EncoderDrive(-84, -0.7, -0.7));
    addSequential(new TurnDegrees(330, 2)); 
    addSequential(new PerfectlyStraightDrive(176, 0.9, 0.9));
    addSequential(new TurnDegrees(270, 2));
    addSequential(new VisionAlignment());
    addParallel(new SlideHatchOut());
    addSequential(new ReleaseHatch());
    addSequential(new EncoderDrive(-20, -20, -0.6));
    addParallel(new SlideHatchIn());
    addSequential(new GraspHatch());
  }
}
