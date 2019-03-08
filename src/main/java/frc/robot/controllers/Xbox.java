package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Xbox {

	private static final double JOYSTICK_DEADZONE = 0.05;

	// main circle buttons
	private static final int BUTTON_A = 1;
	private static final int BUTTON_B = 2;
	private static final int BUTTON_X = 3;
	private static final int BUTTON_Y = 4;
	// other buttons
	private static final int BUTTON_LBUMPER = 5;
	private static final int BUTTON_RBUMPER = 6;
	private static final int BUTTON_BACK = 7;
	private static final int BUTTON_START = 8;
	private static final int BUTTON_LJOYSTICK = 9;
	private static final int BUTTON_RJOYSTICK = 10;

	// axes ports
	private static final int AXIS_LX = 0;
	private static final int AXIS_LY = 1;
	private static final int AXIS_RX = 4;
	private static final int AXIS_RY = 5;
	private static final int AXIS_LTRIGGER = 2;
	private static final int AXIS_RTRIGGER = 3;

	public final JoystickButton buttonA, buttonB, buttonX, buttonY;
	public final JoystickButton buttonBack, buttonStart;
	public final JoystickButton buttonLBumper, buttonRBumper, buttonLJoystick, buttonRJoystick;

	private Joystick joystick;

	public Xbox(int port) {
		joystick = new Joystick(port);
		buttonA = new JoystickButton(joystick, BUTTON_A);
		buttonB = new JoystickButton(joystick, BUTTON_B);
		buttonX = new JoystickButton(joystick, BUTTON_X);
		buttonY = new JoystickButton(joystick, BUTTON_Y);
		buttonBack = new JoystickButton(joystick, BUTTON_BACK);
		buttonStart = new JoystickButton(joystick, BUTTON_START);
		buttonLBumper = new JoystickButton(joystick, BUTTON_LBUMPER);
		buttonRBumper = new JoystickButton(joystick, BUTTON_RBUMPER);
		buttonLJoystick = new JoystickButton(joystick, BUTTON_LJOYSTICK);
		buttonRJoystick = new JoystickButton(joystick, BUTTON_RJOYSTICK);
	}

	public double getLX() {
		return clipDeadzone(joystick.getRawAxis(AXIS_LX));
	}
	
	public double getLY() {
		return clipDeadzone(joystick.getRawAxis(AXIS_LY));
	}

	public double getRX() {
		return clipDeadzone(joystick.getRawAxis(AXIS_RX));
	}

	public double getRY() {
		return clipDeadzone(joystick.getRawAxis(AXIS_RY));
	}

	public double getLTrigger() {
		return clipDeadzone(joystick.getRawAxis(AXIS_LTRIGGER));
	}

	public double getRTrigger() {
		return clipDeadzone(joystick.getRawAxis(AXIS_RTRIGGER));
	}

	private int getPOV() {
		return joystick.getPOV();
	}

	public boolean isPOVUpish() {
		double pov = getPOV();
		return (pov == 315 || pov == 0 || pov == 45);
	}

	public boolean isPOVDownish() {
		double pov = getPOV();
		return (pov == 225 || pov == 180 || pov == 135);
	}

	public void vibrate(){
		joystick.setRumble(RumbleType.kRightRumble, 1);
		joystick.setRumble(RumbleType.kLeftRumble, 1);
	}

	public void stopVibrate(){
		joystick.setRumble(RumbleType.kRightRumble, 0);
		joystick.setRumble(RumbleType.kLeftRumble, 0);
	}

	private double clipDeadzone(double rawValue) {
		if (Math.abs(rawValue) > JOYSTICK_DEADZONE) {
			return rawValue;
		}
		return 0;
	}

	public static double clipAxis(double rawValue, double clipAmount){
		if(Math.abs(rawValue) > clipAmount){
			return rawValue >= 0 ? clipAmount : -clipAmount;
		} else {
			return rawValue;
		}
	}
}