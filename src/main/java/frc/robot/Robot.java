package frc.robot;

import java.io.IOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;
import frc.robot.util.Logger;
import frc.robot.util.RPS;


public class Robot extends TimedRobot {
	public static NetworkTableInstance NetworkInst;
	public static NetworkTable JetsonTable;
	public static Logger SystemLog;
	public static RPS rps;

	public static DriveTrain driveTrain;
	public static Intake intake;
	public static Elevator elevator;
	public static HatchMechanism hatchMechanism;

	public static boolean isAuto = false;

	public static double attemptedStartTime = 0;
	private double bootUpAttempts = 0;
	public static int numAttempts = 5;
	public static OI oi;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();



	@Override
	public void robotInit() {
		NetworkInst = NetworkTableInstance.getDefault();
		JetsonTable = NetworkInst.getTable("JetsonData");
		SystemLog = new Logger("SystemLog");
		driveTrain = new DriveTrain();
		intake = new Intake();
		elevator = new Elevator();
		hatchMechanism = new HatchMechanism();
		rps = new RPS();
		oi = new OI();

		// try-with-resource makes sure that there is no resource leak
		try (Compressor compressor = new Compressor(RobotMap.COMPRESSOR)) {
			compressor.start();
		}

		SmartDashboard.putData("Auto mode", m_chooser);
		m_chooser.addOption("Left One Hatch Auto", new LeftDoubleCargoShipAuto());
		m_chooser.addOption("Right One Hatch Auto", new RightDoubleCargoShipAuto());
		m_chooser.addOption("No Auto", new NoAuto());
		//m_chooser.addOption("Pure Right Double CS Auto", new RightDoubleCargoShipAuto());
		// m_chooser.setDefaultOption("OneHatchAuto", new PlaceHatch());

		CameraServer.getInstance().startAutomaticCapture();
	}

	
	@Override
	public void robotPeriodic() {

		isAuto = isAutonomous();

		driveTrain.setDriveTrainDataOnDisplay();
		elevator.setElevatorDataOnDisplay();
		hatchMechanism.setHatchMechanismDataOnDisplay();
		intake.setIntakeDataOnDisplay();
		rps.setVisionDataOnDisplay();

	}

		// SmartDashboard.putNumber("NavxResetOffset", rps.angleResetOffset);

	@Override
	public void disabledInit() {
		Robot.oi.driveStick.stopVibrate();
		Robot.oi.secondStick.stopVibrate();
	}

	@Override
	public void disabledPeriodic() {
		if (!Robot.rps.isVisionAlive() && bootUpAttempts % 20 == 0) {

			String jetsonCmd = "ssh ubuntu@10.29.76.12 /bin/bash -c '/home/ubuntu/VisionProcessing/Deploy/run_vision_program.sh'";
			ProcessBuilder jetsonProcessStart = new ProcessBuilder();
			jetsonProcessStart.command("sh", "-c", jetsonCmd);
			jetsonProcessStart.inheritIO();

			try {
				jetsonProcessStart.start();
			} catch (IOException e) {
				System.out.println("!!!!!!!!!! Vision Produced Following Error !!!!!!!!!!\n" + e.getMessage());
			}

			SystemLog.writeWithTimeStamp(".......... Jetson Bootup Attempted ..........");
		}
		bootUpAttempts++;
		Scheduler.getInstance().run();
	}

	
	@Override
	public void autonomousInit() {
		rps.reset();
		rps.angleOffset = 0;
		driveTrain.resetEncoders();
		Robot.hatchMechanism.slideOut();
		elevator.elevatorEncoder.reset();


		m_autonomousCommand = m_chooser.getSelected();
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
		Logger.flushAllLogs();
		Scheduler.getInstance().run();
	}

}
