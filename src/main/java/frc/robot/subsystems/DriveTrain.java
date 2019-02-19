package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;

/**
 * Subsystem for the West-Coast-style drive train.
 * Has methods for driving in tank mode or arcade mode.
 */
public class DriveTrain extends Subsystem {

	private WPI_TalonSRX leftFront, rightFront;
	private WPI_VictorSPX  leftMid, leftBack, rightMid, rightBack;
	private SpeedControllerGroup left, right;
	private Encoder encoderLeft, encoderRight;
	private DifferentialDrive m_drive;

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
	}

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
	public double getLeftEncoderDistance() {
		return encoderLeft.getRaw() * RobotMap.ENCODER_DISTANCE_PER_PULSE;
	}

	/**
	 * Returns the distance in meters from the right encoder
	 * @return distance in inches travelled by the drive train's right side
	 */
	public double getRightEncoderDistance() {
		return encoderRight.getRaw() * RobotMap.ENCODER_DISTANCE_PER_PULSE;
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
		right.set(-rightSpeed);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoystick());
	}
}