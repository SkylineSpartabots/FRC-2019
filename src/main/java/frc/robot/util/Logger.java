package frc.robot.util;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class Logger {

	private PrintWriter writer;
	private boolean isLogging = true;
	private static ArrayList<Logger> instances = new ArrayList<Logger>();

	public Logger(String filename) {
		try {
			File f = new File(RobotMap.SYSTEM_LOG_PATH + filename + ".txt");
			f.createNewFile();
			f.setWritable(true);
			writer = new PrintWriter(f);
		} catch (IOException | SecurityException | NullPointerException ex) {
			System.out.println("Logger File Exception: " + ex.getMessage());
			isLogging = false;
		}
		instances.add(this);
	}
	public static void flushAllLogs()	{
		for(Logger l : instances)	{
			l.flushLogData();
		}
	}
	public void writeNewData(String s) {
		if (this.isLogging) {
			writer.println(s);
		} else {
			System.out.println("Logger is not in a logging state");
		}
	}
	public void writeWithTimeStamp(String s)	{
		writeNewData(Timer.getFPGATimestamp() + ": " + s);
	}

	public void flushLogData() {
		if (this.isLogging) {
			writer.flush();
		}
	}
}