/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchMechanism extends Subsystem {
  
  private Solenoid hatchSolenoid;
  
  public HatchMechanism(){
    hatchSolenoid = new Solenoid(RobotMap.hatchSolenoid);
    hatchSolenoid.set(false);
  }

  public void release(){
    hatchSolenoid.set(false);
  }

  public void grasp(){
    hatchSolenoid.set(true);
  }

  public boolean getState(){
    return hatchSolenoid.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
