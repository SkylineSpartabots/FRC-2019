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
import frc.robot.commands.auto_commands.TurnDegreesVision;
import frc.robot.commands.auto_commands.VisionAllignment;
import frc.robot.commands.autonomous.PlaceHatch;
import frc.robot.subsystems.*;
import frc.robot.util.Logger;
import frc.robot.util.RPS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static NetworkTableInstance NetworkInst;
	public static NetworkTable JetsonTable;
	public static Logger SystemLog;
	public static RPS rps;

	public static DriveTrain driveTrain;
	public static Intake intake;
	public static Elevator elevator;
	public static HatchMechanism hatchMechanism;

	public static OI oi;
	

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		NetworkInst = NetworkTableInstance.getDefault();
		JetsonTable = NetworkInst.getTable("JetsonData");
		SystemLog = new Logger("SystemLog");
		rps = new RPS();
		driveTrain = new DriveTrain();
		intake = new Intake();
		elevator = new Elevator();
		hatchMechanism = new HatchMechanism();
		oi = new OI();
	
		// try-with-resource makes sure that there is no resource leak
		try (Compressor compressor = new Compressor(RobotMap.COMPRESSOR)) {
			compressor.start();
		}

		SmartDashboard.putData("Auto mode", m_chooser);
		m_chooser.addOption("OneHatchAuto", new PlaceHatch());
		m_chooser.addOption("visiona", new VisionAllignment());
		m_chooser.addOption("turnv", new TurnDegreesVision(15));
		m_chooser.setDefaultOption("OneHatchAuto", new PlaceHatch());

		System.out.println("Starting Jetson");	
		SystemLog.writeWithTimeStamp("Starting Jetson");
		String jetsonCmd = "ssh ubuntu@10.29.76.12 /bin/bash -c '/home/ubuntu/VisionProcessing/Deploy/run_vision_program.sh'";
		ProcessBuilder jetsonProcessStart = new ProcessBuilder();
		jetsonProcessStart.command("sh", "-c", jetsonCmd);
		jetsonProcessStart.inheritIO();
		try{
			jetsonProcessStart.start();
		}	catch (IOException e){
			System.out.println("Errpr" + e.getMessage());
			SystemLog.writeWithTimeStamp("IOException at Jetson Start: " + e.getMessage());
		}
		SystemLog.writeWithTimeStamp("Jetson Process Start Attempted | Did not Block");

		CameraServer.getInstance().startAutomaticCapture();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {	
		driveTrain.setDriveTrainDataOnDisplay();
		elevator.setElevatorDataOnDisplay();
		hatchMechanism.setHatchMechanismDataOnDisplay();
		intake.setIntakeDataOnDisplay();
		System.out.println("");
		SmartDashboard.putNumber("XDisp", Robot.rps.getXDisplacementToVisionTargetRawInches());
		SmartDashboard.putNumber("ZDisp", Robot.rps.getZDisplacementToVisionTargetRawInches());
		SmartDashboard.putNumber("Angle", Robot.rps.getYawToVisionTargetRawDegrees());
		SmartDashboard.putNumber("Navx", Robot.rps.getNavxAngle());		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		//oi.driveStick.stopVibrate();
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		rps.reset();
		driveTrain.resetEncoders();	
		Robot.hatchMechanism.slideOut();	
		//m_autonomousCommand = m_chooser.getSelected();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		hatchMechanism.graspHatch();
		intake.retractIntake();
		elevator.elevatorEncoder.reset();
		rps.reset();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {	
		//Logger.flushAllLogs();
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
