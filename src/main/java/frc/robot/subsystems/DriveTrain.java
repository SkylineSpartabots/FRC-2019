package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive_controls.DriveWithJoystick;


/**
 * Subsystem for the West-Coast-style drive train.
 * Has methods for driving in tank mode or arcade mode.
 */
public class DriveTrain extends Subsystem {

	private WPI_TalonSRX leftFront, rightFront;
	private WPI_VictorSPX  leftMid, leftBack, rightMid, rightBack;
	private Encoder encoderLeft, encoderRight;
	private DifferentialDrive m_drive;
	private SpeedControllerGroup left, right;
	

	private static final ShuffleboardTab TAB = Shuffleboard.getTab("DriveConstants");
	private static NetworkTableEntry turnkP, turnkI, turnkD, proportionVelocity, pathkP, pathkD, pathkA, forward, lateral, vibrateProportion;
	//private static NetworkTableEntry zOffset, zScale, positiveXFudge, negativeXFudge, positiveOffset, negativeOffset;
	public DriveTrain() {
		leftFront = new WPI_TalonSRX(RobotMap.LEFT_FRONT_DRIVE_MOTOR);
		leftMid = new WPI_VictorSPX(RobotMap.LEFT_MID_DRIVE_MOTOR);
		leftBack = new WPI_VictorSPX(RobotMap.LEFT_BACK_DRIVE_MOTOR);
		rightFront = new WPI_TalonSRX(RobotMap.RIGHT_FRONT_DRIVE_MOTOR);
		rightMid = new WPI_VictorSPX(RobotMap.RIGHT_MID_DRIVE_MOTOR);
		rightBack = new WPI_VictorSPX(RobotMap.RIGHT_BACK_DRIVE_MOTOR);

		leftFront.setInverted(RobotMap.IS_LEFT_FRONT_DRIVE_INVERTED);
		leftMid.setInverted(RobotMap.IS_LEFT_MID_DRIVE_INVERTED);
		leftBack.setInverted(RobotMap.IS_LEFT_BACK_DRIVE_INVERTED);
		rightFront.setInverted(RobotMap.IS_RIGHT_FRONT_DRIVE_INVERTED);
		rightMid.setInverted(RobotMap.IS_RIGHT_MID_DRIVE_INVERTED);
		rightBack.setInverted(RobotMap.IS_RIGHT_BACK_DRIVE_INVERTED);

		encoderRight = new Encoder(RobotMap.RIGHT_WHEEL_ENCODER_PORT_A, RobotMap.RIGHT_WHEEL_ENCODER_PORT_B);
		encoderLeft = new Encoder(RobotMap.LEFT_WHEEL_ENCODER_PORT_A, RobotMap.LEFT_WHEEL_ENCODER_PORT_B);

		// Set in brake mode
		leftFront.setNeutralMode(NeutralMode.Brake);
		leftMid.setNeutralMode(NeutralMode.Brake);
		leftBack.setNeutralMode(NeutralMode.Brake);
		rightFront.setNeutralMode(NeutralMode.Brake);
		rightMid.setNeutralMode(NeutralMode.Brake);
		rightBack.setNeutralMode(NeutralMode.Brake);

		left = new SpeedControllerGroup(leftFront, leftMid, leftBack);
		right = new SpeedControllerGroup(rightFront, rightMid, rightBack);


		m_drive = new DifferentialDrive(left, right);
		m_drive.setRightSideInverted(false);

		//Shuffleboard inputs
		turnkP = TAB.add("Turn kP", 0.016).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		turnkI = TAB.add("Turn kI", 0.00056).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		turnkD = TAB.add("Turn kD", 0.0012).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();


		pathkP = TAB.add("Path kP", 1.1).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		proportionVelocity = TAB.add("Path velo", 0.0012).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		pathkD = TAB.add("Path kD", 0.00056).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		pathkA = TAB.add("Path kA", 0.00056).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
	
		lateral = TAB.add("lateral", 0).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		forward = TAB.add("forward", 2).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();

		vibrateProportion = TAB.add("vibrate", 0.2).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 1)).getEntry();
		
		/*zOffset = TAB.add("Z-Axis Offset", 0.45).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		zScale = TAB.add("Z-Axis Scale", 1.1).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		
		positiveXFudge = TAB.add("Postive X Fudge Factor", 1.5).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		negativeXFudge = TAB.add("Negative X Fudge Factor", 1.35).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		
		positiveOffset = TAB.add("Postive X Fudge Offset", 0.1).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();
		negativeXFudge = TAB.add("Negative X Fudge Offset", 0.08).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("Min", 0.0, "Max", 5)).getEntry();*/
		
	
	}

	/**
	 * 
	 * @return returns double array of turn pid constants in form of kp, ki, kd
	 */
	public double[] getTurnPID() {
		double[] constants = {turnkP.getDouble(0.00001), turnkI.getDouble(0.00001), turnkD.getDouble(0.00001)};
		return constants;
	}

	public double[] getPathPID(){
		double[] constants = {pathkP.getDouble(0.00001), pathkD.getDouble(0.00001), proportionVelocity.getDouble(0.00001), pathkA.getDouble(0.00001)};
		return constants;
	}

	public double[] distances(){
		double[] constants = {forward.getDouble(0.001), lateral.getDouble(0.001)};
		return constants;
	}

	public double getVibrate(){
		return vibrateProportion.getDouble(0.001);
	}

	
	/**
	 * 
	 * @return	zOffset, zScale, positive fudge, negative fudge, positive offset, negative offset
	 */
	/*public double[] getVisionConstants(){
		double[] constants = {zOffset.getDouble(0.001), zScale.getDouble(0.001), positiveXFudge.getDouble(0.001), negativeXFudge.getDouble(0.001),
			 positiveOffset.getDouble(0.001), negativeOffset.getDouble(0.001)};
		return constants;
	}*/


	/**
	 * Resets both the left and right encoders
	 */
	public void resetEncoders() {
		encoderLeft.reset();
		encoderRight.reset();
	}
	/**
	 * Returns the distance in meters from the left encoder
	 * @return distance in inches travelled by the drive train's left side
	 */
	public double getLeftEncoderDistanceInches() {
		double inches = encoderLeft.getRaw() * RobotMap.ENCODER_DISTANCE_PER_PULSE;
		return inches;
	}
	public double getLeftEncoderDistanceMeters() {
		double meters = encoderLeft.getRaw() * RobotMap.ENCODER_DISTANCE_PER_PULSE*0.0254;
		return meters;
	}
	/**
	 * Returns the distance in meters from the right encoder
	 * @return distance in inches travelled by the drive train's right side
	 */
	public double getRightEncoderDistanceInches() {
		double inches = encoderRight.getRaw() * RobotMap.ENCODER_DISTANCE_PER_PULSE;
		//System.out.println("RightENcoder " + inches);
		return inches;
	}
	public double getRightEncoderDistanceMeters() {
		double meters = encoderRight.getRaw() * RobotMap.ENCODER_DISTANCE_PER_PULSE*0.0254;
		return meters;
	}
	public void setBrake() {
		leftFront.neutralOutput();
		leftMid.neutralOutput();
		leftBack.neutralOutput();
		rightFront.neutralOutput();
		rightMid.neutralOutput();
		rightBack.neutralOutput();
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		m_drive.tankDrive(leftSpeed, rightSpeed);
	}

	/**
	 * Stops all drive train motors
	 */
	public void stop() {
		tankDrive(0, 0);
	}

	public void arcadeDrive(double forward, double turn) {
		m_drive.arcadeDrive(forward, turn);
	}

	public void rawMotorOutput(double leftSpeed, double rightSpeed) {
		left.set(leftSpeed);
		right.set(rightSpeed);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoystick());
	}

	public void setDriveTrainDataOnDisplay() {
		SmartDashboard.putNumber("Right Drive Encoder In Inches", getRightEncoderDistanceInches());
		SmartDashboard.putNumber("Left Drive Encoder In Inches", getLeftEncoderDistanceInches());
		SmartDashboard.putNumber("Relative Heading", Robot.rps.getNavxAngle());
		SmartDashboard.putNumber("Absolute Heading", Robot.rps.getAbsoluteAngle());
	}

	public boolean checkSubsystem(){
		System.out.println("\n\n\n\nTesting Drive Train..........................");

		double kEncoderThreshold = 20;
		boolean rightEncoderFailure = false;
		boolean leftEncoderFailure = false;

		resetEncoders();
		
		System.out.println("\n\nTesting Drive Encoders...........................");
		tankDrive(0.6, 0.6);
		Timer.delay(1);
		tankDrive(0, 0);

		if(encoderRight.get() > kEncoderThreshold){
			System.out.println("######## SUCCESSFUL: GO FOR RIGHT DRIVE ENCODERS ########");
		} else {
			System.out.println("!!!!!!!! FAILURE: RIGHT ENCODERS ARE NOT FUNCTIONAL !!!!!!!!");
			rightEncoderFailure = true;
		}

		if(encoderLeft.get() > kEncoderThreshold){
			System.out.println("######## SUCCESSFUL: GO FOR LEFT DRIVE ENCODERS ########");
		} else {
			System.out.println("!!!!!!!! FAILURE: LEFT ENCODERS ARE NOT FUNCTIONAL !!!!!!!!");
			leftEncoderFailure = true;
		}

		return !leftEncoderFailure && !rightEncoderFailure;



	}

	


}
