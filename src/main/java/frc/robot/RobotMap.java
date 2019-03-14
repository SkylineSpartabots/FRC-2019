package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Robot physical metrics
	public static final double TRACK_WIDTH = 0.68; // in meters
	public static final double MAX_VELOCITY = 3.3; // in m/s
	public static final double ENCODER_DISTANCE_PER_PULSE = Math.PI * 4 / 1000; // Distance in inches robot travels in one pulse

	// Path trajectories for sandstorm modes
	public static final String AUTO_TRAJECTORY_PATH_LOCATIONS = "//home//lvuser//deploy//";

	// Jetson communications
	public static final String JETSON_STATIC_IP = "10.29.76.12";
	public static final int JETSON_COMM_PORT = 5806;
	public static final String SYSTEM_LOG_PATH = "//home//lvuser//deploy//";

	public static final int COMPRESSOR = 0;

	public static final int RIGHT_INTAKE_MOTOR = 6;
	public static final int LEFT_INTAKE_MOTOR = 11;

	public static final int INNER_INTAKE_MOTOR = 5;
	public static final int INTAKE_SENSOR = 0;
	public static final int INTAKE_SENSOR_THRESHOLD = 50;

	public static final int INTAKE_SOLENOID = 2;

	// drive train motor ports - WCD
	public static final int RIGHT_FRONT_DRIVE_MOTOR = 10;
	public static final int RIGHT_MID_DRIVE_MOTOR = 9;
	public static final int RIGHT_BACK_DRIVE_MOTOR = 8;
	public static final int LEFT_FRONT_DRIVE_MOTOR = 4;
	public static final int LEFT_MID_DRIVE_MOTOR = 3;
	public static final int LEFT_BACK_DRIVE_MOTOR = 2;
	// whether each drive motor is inverted or not
	public static final boolean IS_RIGHT_FRONT_DRIVE_INVERTED = true;
	public static final boolean IS_RIGHT_MID_DRIVE_INVERTED = true;
	public static final boolean IS_RIGHT_BACK_DRIVE_INVERTED = true;
	public static final boolean IS_LEFT_FRONT_DRIVE_INVERTED = false;
	public static final boolean IS_LEFT_MID_DRIVE_INVERTED = false;
	public static final boolean IS_LEFT_BACK_DRIVE_INVERTED = false;

	public static final int RIGHT_ELEVATOR = 7; // master
	public static final int LEFT_ELEVATOR = 1; // slave

	// encoders on drive train
	public static final int LEFT_WHEEL_ENCODER_PORT_A = 2;
	public static final int LEFT_WHEEL_ENCODER_PORT_B = 3;
	public static final int RIGHT_WHEEL_ENCODER_PORT_A = 1;
	public static final int RIGHT_WHEEL_ENCODER_PORT_B = 0;

	// encoders on elevator
	public static final int ELEVATOR_ENCODER_PORT_A = 4;
	public static final int ELEVATOR_ENCODER_PORT_B = 5;

	// elevator limit switch port
	public static final int ELEVATOR_LIMIT_SWITCH = 6;

	// hatch mechanism solenoid port
	public static final int HATCH_SOLENOID = 1;

	public static final int HATCH_SLIDER_SOLENOID = 0;

	//hatch mechanism limit switch port
	public static final int HATCH_LIMIT_SWTICH = 2;

	//Threshold for analog limit switch
	public static final int HATCH_LIMIT_SWTICH_THRESHOLD = 1800;
}
