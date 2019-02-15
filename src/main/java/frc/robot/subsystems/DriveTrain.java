package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {

	WPI_TalonSRX leftFront, leftMid, leftBack, rightFront, rightMid, rightBack;
	SpeedControllerGroup left, right;
	Encoder encoderLeft, encoderRight;
	DifferentialDrive m_drive;

	public DriveTrain() {
		leftFront = new WPI_TalonSRX(RobotMap.leftFrontDrivePort);
		leftMid = new WPI_TalonSRX(RobotMap.leftMidDrivePort);
		leftBack = new WPI_TalonSRX(RobotMap.leftBackDrivePort);
		rightFront = new WPI_TalonSRX(RobotMap.rightFrontDrivePort);
		rightMid = new WPI_TalonSRX(RobotMap.rightMidDrivePort);
		rightBack = new WPI_TalonSRX(RobotMap.rightBackDrivePort);

		encoderRight = new Encoder(RobotMap.rightWheelEncoderPorts[0], RobotMap.rightWheelEncoderPorts[1]);
		encoderLeft = new Encoder(RobotMap.leftWheelEncoderPorts[0], RobotMap.leftWheelEncoderPorts[1]);

		encoderLeft.setDistancePerPulse(RobotMap.EncoderDistancePerPule);
		encoderRight.setDistancePerPulse(RobotMap.EncoderDistancePerPule);

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
		setDefaultCommand(new DriveWithJoystick());
	}
}
