package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.util.Navx;
import frc.robot.util.vision.Limelight;
import frc.robot.util.vision.Limelight.LEDState;


public class Robot extends TimedRobot {
	public static Navx navx;

	public static DriveTrain driveTrain;
	public static Intake intake;
	public static Elevator elevator;
	public static HatchMechanism hatchMechanism;
	public static Limelight limelight;
	public static OI oi;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();



	@Override
	public void robotInit() {
	
		driveTrain = new DriveTrain();
		intake = new Intake();
		elevator = new Elevator();
		hatchMechanism = new HatchMechanism();
		navx = new Navx();
		limelight = new Limelight();
		oi = new OI();


		Limelight.setLEDState(LEDState.PIPELINE);


		try (Compressor compressor = new Compressor(RobotMap.COMPRESSOR)) {
			compressor.start();
		}

		SmartDashboard.putData("Auto mode", m_chooser);


		CameraServer.getInstance().startAutomaticCapture();
	}

	
	@Override
	public void robotPeriodic() {
		driveTrain.setDriveTrainDataOnDisplay();
		elevator.setElevatorDataOnDisplay();
		hatchMechanism.setHatchMechanismDataOnDisplay();
		intake.setIntakeDataOnDisplay();
		limelight.setDataOnDisplay();

	}

	@Override
	public void disabledInit() {
		Robot.oi.driveStick.stopVibrate();
		Robot.oi.secondStick.stopVibrate();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	
	@Override
	public void autonomousInit() {
		navx.reset();
		driveTrain.resetEncoders();
		Robot.hatchMechanism.slideOut();
		elevator.elevatorEncoder.reset();

		
		//m_autonomousCommand = m_chooser.getSelected();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		hatchMechanism.graspHatch();
		intake.retractIntake();
	}

	
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

}
