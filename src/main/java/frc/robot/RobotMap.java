package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Robot Constants (metrics)
	public static double TrackWidth = 0.5; // in meters
	public static double MaxVelocity = 0; // in m/s
	public static double EncoderDistancePerPule = 0.1; // Distance robot travels in one pulse

	// Path Trajectories
	public static String AutoTrajectoryPathLocations = "//home//lvuser//deploy//";

	// Socket Comm Ports
	public static String JetsonStaticIP = "10.29.76.86";
	public static int JetsonCommPort = 5806;
	public static String SystemLogPath = "//home//lvuser//deploy//";

	public static int compressorPort = 0;

	public static int rightIntakeMotor = 5;
	public static int leftIntakeMotor = 7;

	public static int innerIntakeMotor = 6;
	public static int intakeSensorPort = 0;
	public static int intakeSensorThreshold = 0;

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

	public static int[] rightWheelEncoderPorts = { 0, 1 };
	public static int[] leftWheelEncoderPorts = { 2, 3 };

	public static int rightElevatorPort = 6;
	public static int leftElevatorPort = 7;

	public static int[] elevatorEncoderPorts = { 4, 5 };
	public static int elevatorLimitSwitch = 2;

	public static int hatchSolenoid = 3;

	public static final double JOYSTICK_DEADZONE = 0.05;
}