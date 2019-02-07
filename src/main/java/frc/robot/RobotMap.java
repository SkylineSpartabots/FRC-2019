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
  //Robot Constants
  public static double TrackWidth = 0.5; //in meters
  public static double MaxVelocity = 0; //m/s
  public static double EncoderDistancePerPule  = 0.1; //Distance robot travels in one pulse
  //Path Trajectories
  public static String AutoTrajectoryPathLocations = "//home//lvuser//deploy//";

  //Socket Comm Ports
  public static String JetsonStaticIP = "10.29.76.86";
  public static int JetsonCommPort = 5804;
  public static String SystemLogPath = "//home//lvuser//deploy//";

  //Encoder Ports
  public static int RightWheelEncoderA = 1;
  public static int RightWheelEncoderB = 2;
  public static int LeftWheelEncoderA = 3;
  public static int LeftWheelEncoderB = 4;

  public static int leftIntakeMotor = 5;
  public static int rightIntakeMotor = 7;
  public static int centerIntakeMotor = 6;

  public static int rightIntakeSolenoid = 1;
  public static int leftIntakeSolenoid = 2;

  public static int driveStick = 0;
  public static int secondStick = 1;

  public static int rightFrontDrivePort = 0;
  public static int rightMidDrivePort = 1;
  public static int rightBackDrivePort = 2;

  public static int leftFrontDrivePort = 3;
  public static int leftMidDrivePort = 4;
  public static int leftBackDrivePort = 5;

  public static int[] rightWheelEncoderPorts = {0, 1};
  public static int[] leftWheelEncoderPorts = {2, 3};

  public static int masterElevatorPort = 6;
  public static int slaveElevatorPort = 7;

  public static int[] elevatorEncoderPorts = {4, 5};
  public static int elevatorLimitSwitch = 2;


}
