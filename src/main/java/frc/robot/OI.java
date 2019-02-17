package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.GraspHatch;
import frc.robot.commands.ReleaseHatch;
import frc.robot.commands.TestSubsystem;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private static final double JOYSTICK_DEADZONE = 0.05;

	public Joystick driveStick;
	public Joystick secondStick;

	public enum Button {
		RBumper(6), LBumper(5), A(1), B(2), X(3), Y(4), RightJoystickBtn(10), LeftJoystickBtn(9), Start(8), Back(7);

		private final int number;

		Button(int number) {
			this.number = number;
		}

		public int getBtnNumber() {
			return number;
		}
	}

	public enum Axis {
		LX(0), LY(1), LTrigger(2), RTrigger(3), RX(4), RY(5);
		private final int number;

		Axis(int number) {
			this.number = number;
		}

		public int getAxisNumber() {
			return number;
		}
	}

	/**
	 * 
	 * @param rawValue
	 * @return clips deadzone from joysticks and triggers when value passed in
	 */
	public double clipDeadzone(double rawValue) {
		if (Math.abs(rawValue) > JOYSTICK_DEADZONE) {
			return rawValue;
		} else {
			return 0;
		}
	}

	public OI() {
		driveStick = new Joystick(0);
		secondStick = new Joystick(1);

		JoystickButton buttonA = new JoystickButton(secondStick, OI.Button.A.getBtnNumber());
		JoystickButton buttonB = new JoystickButton(secondStick, OI.Button.B.getBtnNumber());
		JoystickButton buttonX = new JoystickButton(secondStick, OI.Button.X.getBtnNumber());
		JoystickButton buttonY = new JoystickButton(secondStick, OI.Button.Y.getBtnNumber());
		JoystickButton lBumper = new JoystickButton(secondStick, OI.Button.LBumper.getBtnNumber());
		JoystickButton rBumper = new JoystickButton(secondStick, OI.Button.RBumper.getBtnNumber());

		JoystickButton start = new JoystickButton(secondStick, OI.Button.Start.getBtnNumber());
		JoystickButton back = new JoystickButton(secondStick, OI.Button.Back.getBtnNumber());

		JoystickButton xDrive = new JoystickButton(driveStick, OI.Button.X.getBtnNumber());
		try {
			xDrive.whileHeld(new TestSubsystem(0.2));
			//buttonA.whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.ROCKET_FIRST));
			//buttonB.whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.CARGO_SHIP));
			//buttonX.whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.ROCKET_SECOND));
			//buttonY.whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.ROCKET_THIRD));

			//lBumper.whenPressed(new GraspHatch());
			//rBumper.whenPressed(new ReleaseHatch());

			//start.whenPressed(new SlideHatchOut());
			//back.whenPressed(new SlideHatchIn());

		} finally {
			xDrive.close();
			buttonA.close();
			buttonB.close();
			buttonX.close();
			buttonY.close();
			lBumper.close();
			rBumper.close();
			start.close();
			start.close();
		}

		// TODO: need to add controls for extending and retracting intake

	}

}