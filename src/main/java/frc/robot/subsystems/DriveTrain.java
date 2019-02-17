package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {

	private WPI_TalonSRX leftFront, rightFront;
	private VictorSPX  leftMid, leftBack, rightMid, rightBack;
	private SpeedControllerGroup left, right;
	private Encoder encoderLeft, encoderRight;
	private DifferentialDrive m_drive;

	public DriveTrain() {
		leftFront = new WPI_TalonSRX(RobotMap.LEFT_FRONT_DRIVE_MOTOR);
		leftMid = new VictorSPX(RobotMap.LEFT_MID_DRIVE_MOTOR);
		leftBack = new VictorSPX(RobotMap.LEFT_BACK_DRIVE_MOTOR);
		rightFront = new WPI_TalonSRX(RobotMap.RIGHT_FRONT_DRIVE_MOTOR);
		rightFront.setInverted(true);
		rightMid = new VictorSPX(RobotMap.RIGHT_MID_DRIVE_MOTOR);
		rightBack = new VictorSPX(RobotMap.RIGHT_BACK_DRIVE_MOTOR);

		encoderRight = new Encoder(RobotMap.RIGHT_WHEEL_ENCODER_PORT_A, RobotMap.RIGHT_WHEEL_ENCODER_PORT_B);
		encoderLeft = new Encoder(RobotMap.LEFT_WHEEL_ENCODER_PORT_A, RobotMap.LEFT_WHEEL_ENCODER_PORT_B);

		encoderLeft.setDistancePerPulse(RobotMap.ENCODER_DISTANCE_PER_PULSE);
		encoderRight.setDistancePerPulse(RobotMap.ENCODER_DISTANCE_PER_PULSE);

		// Set in brake mode
		leftFront.setNeutralMode(NeutralMode.Brake);
		leftMid.setNeutralMode(NeutralMode.Brake);
		leftBack.setNeutralMode(NeutralMode.Brake);
		rightFront.setNeutralMode(NeutralMode.Brake);
		rightMid.setNeutralMode(NeutralMode.Brake);
		rightBack.setNeutralMode(NeutralMode.Brake);

		rightMid.follow(rightFront);
		rightBack.follow(rightFront);
		leftMid.follow(leftFront);
		leftBack.follow(leftFront);
		left = new SpeedControllerGroup(leftFront);
		right = new SpeedControllerGroup(rightFront);

		m_drive = new DifferentialDrive(left, right);
	}

	public void driveMotor()	{
		//leftFront.set(0.4);
		//rightFront.set(0.4);
		  rightBack.set(Mode, demand0, demand1);
	}
	public void resetEncoders() {
		encoderLeft.reset();
		encoderRight.reset();
	}

	public double getLeftEncoderDistance() {
		return encoderLeft.getDistance();
	}

	public double getRightEncoderDistance() {
		return encoderRight.getDistance();
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

	public void arcadeDrive(double forward, double turn) {
		m_drive.arcadeDrive(forward, turn);
	}

	public void rawMotorOutput(double leftSpeed, double rightSpeed) {
		left.set(leftSpeed);
		right.set(rightSpeed);
	}

	public void drive(double speed, double rotation) {
		SmartDashboard.putNumber("Power", speed);
		m_drive.arcadeDrive(speed, rotation, true);
	}

	public void initDefaultCommand() {
		//setDefaultCommand(new DriveWithJoystick());
	}
}
