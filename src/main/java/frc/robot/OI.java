package frc.robot;

import frc.robot.commands.auto_commands.AlignToTarget;
import frc.robot.commands.drive_controls.*;
import frc.robot.controllers.Xbox;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public Xbox driveStick;
	public Xbox secondStick;

	public OI() {
		driveStick = new Xbox(0);
		secondStick = new Xbox(1);

		secondStick.buttonRBumper.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.DOWN));
		secondStick.buttonLBumper.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.DOWN));
		secondStick.buttonX.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_SECOND));
		secondStick.buttonA.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_FIRST));
		secondStick.buttonY.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_THIRD));
		secondStick.buttonB.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.CARGO_SHIP));

		driveStick.buttonB.whenPressed(new AlignToTarget());


	}

}
