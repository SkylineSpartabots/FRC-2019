package frc.robot.util;
import java.io.*;
import java.net.Socket;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class JetsonComm implements Runnable{
	private Socket sock;
	private PrintWriter writer;
	private BufferedReader in;
	public Thread TransferThread;
	public JetsonVisionData VisionData;
	private boolean isStopped = false;
	
	public JetsonComm()  {
		try {
			System.out.println("Socket: Connecting");
			Robot.SystemLog.writeNewData("Benign Message: Jetson Socket Connecting: TimeStamp: " + Timer.getFPGATimestamp());
			sock = new Socket(RobotMap.JetsonStaticIP,RobotMap.JetsonCommPort);
			writer = new PrintWriter(sock.getOutputStream(), true);
			in = new BufferedReader(new InputStreamReader(sock.getInputStream()));
		} catch (IOException e) {
			e.printStackTrace();
		}
		VisionData = new JetsonVisionData();
		TransferThread = new Thread(this);
		TransferThread.start();
		Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
	        public void run() {
	        	try {
					in.close();
					writer.close();
					sock.close();
				} catch (IOException e) {					
				}
	        }
	    }));
		Robot.SystemLog.writeNewData("Benign Message: Jetson Socket Ready: TimeStamp: " + Timer.getFPGATimestamp());
	}
	public void transferData()	{
		try {
			if (in.ready()) {
				String s = in.readLine();
				String[] data = s.split(";");
				VisionData.LastTimeStamp = System.currentTimeMillis();
				VisionData.Z_Distance_RobotCoordinateFrame = Double.parseDouble(data[0]);
				VisionData.Y_Distance_RobotCoordinateFrame = Double.parseDouble(data[1]);
				VisionData.X_Distance_RobotCoordinateFrame = Double.parseDouble(data[2]);
				VisionData.Y_Axis_Rotation_To_Vision_Target = Double.parseDouble(data[3]);
				VisionData.X_Axis_Rotation_To_Vision_Target = Double.parseDouble(data[4]);				
				SmartDashboard.putString("JetsonRawDataString", s);
				Thread.sleep(25);
			}
		} catch (Exception e) {
		}
	}
	public void stop()	{
		isStopped = true;
		try {
			TransferThread.join(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	@Override
	public void run() {
		while (!isStopped) {
			transferData();
		}
	}
	class JetsonVisionData	{
		/**
		 * Jetson displacement coordinates:
		 * 
		 * Z-Axis extends straight forward from the camera
		 * Y Axis is up/down (in direction of gravity)
		 * X-Axis is left/right 
		 */
		double Z_Distance_RobotCoordinateFrame;
		double Y_Distance_RobotCoordinateFrame;
		double X_Distance_RobotCoordinateFrame;

		double Y_Axis_Rotation_To_Vision_Target;
		double X_Axis_Rotation_To_Vision_Target;

		long LastTimeStamp;
	}

}



