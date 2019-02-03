package frc.robot.util;
import java.io.*;
import java.net.Socket;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonComm implements Runnable{
	private Socket sock;
	private PrintWriter writer;
	private BufferedReader in;

	public Thread TransferThread;
	private boolean isStopped = false;
	
	public JetsonComm()  {
		try {
			System.out.println("Socket: Connecting");
			sock = new Socket("10.29.76.86",5804);
			writer = new PrintWriter(sock.getOutputStream(), true);
			in = new BufferedReader(new InputStreamReader(sock.getInputStream()));
		} catch (IOException e) {
			e.printStackTrace();
		}

		
		TransferThread = new Thread(this);
		TransferThread.start();
		Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
	        public void run() {
	        	try {
					sock.close();
				} catch (IOException e) {
					
				}

	        }
	    }));
		System.out.println("Socket: Ready");
	}
	public void transferData()	{
		try {
			if (in.ready()) {
                String s = in.readLine();
                SmartDashboard.putString("Jetson", s);
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
}



