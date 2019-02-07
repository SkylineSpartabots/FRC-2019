/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static int rightFrontDrivePort = 0;
  public static int rightMidDrivePort = 1;
  public static int rightBackDrivePort = 2;

  public static int leftFrontDrivePort = 3;
  public static int leftMidDrivePort = 4;
  public static int leftBackDrivePort = 5;

  public static int masterElevatorPort = 6;
  public static int slaveElevatorPort = 7;

  public static int[] elevatorEncoderPorts = {0, 1};
  public static int elevatorLimitSwitch = 2;

}
