/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto_commands.PerfectlyStraightDrive;
import frc.robot.commands.auto_commands.VisionAlignment;
import frc.robot.commands.basic_commands.ReleaseHatch;

public class PlaceHatch extends CommandGroup {

  

  public PlaceHatch() {

    addSequential(new PerfectlyStraightDrive(70, 0.6, 0.6));//in inches
    addParallel(new ReleaseHatch());
    addSequential(new VisionAlignment());
    //addSequential(new EncoderDrive(0.1,0.5,0.5)); //in meters
    //addSequential(new EncoderDrive(-0.1,-0.5,-0.5));

    //addSequential(new PathExecuter(toCargo, "TestCargoPath", true));
    //addSequential(new AllignAndPathToTarget());
    //addSequential(new );
    //addSequential(new EncoderDrive(-0.6, -0.6, -0.6));
    //dispense 

  }
}
