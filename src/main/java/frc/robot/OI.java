package frc.robot;

import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.GraspHatch;
import frc.robot.commands.ReleaseHatch;
import frc.robot.commands.SlideHatchIn;
import frc.robot.commands.SlideHatchOut;
import frc.robot.controllers.Logitech;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public Logitech driveStick;
	public Logitech secondStick;

	public OI() {
		driveStick = new Logitech(0);
		secondStick = new Logitech(1);

		secondStick.buttonB.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.CARGO_SHIP));
		secondStick.buttonA.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_FIRST));
		secondStick.buttonX.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_SECOND));
		secondStick.buttonY.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_THIRD));

		secondStick.buttonLBumper.whenPressed(new GraspHatch());
		secondStick.buttonRBumper.whenPressed(new ReleaseHatch());

		secondStick.buttonStart.whenPressed(new SlideHatchOut());
		secondStick.buttonBack.whenPressed(new SlideHatchIn());

		// TODO: need to add controls for extending and retracting intake

	}

}