package frc.robot;

import frc.robot.commands.drive_controls.*;
import frc.robot.commands.basic_commands.*;
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
		/*
			Start Point: bottom of platform (left side)
			End Point: Near Cargo
			new Waypoint(0, 0, 0),
			new Waypoint(3.25, 1.25, 0),
		*/
		Waypoint[] toCargo = new Waypoint[]{
			new Waypoint(0, 0, 0),
			new Waypoint(3.25, 1.25, 0),
		};

		//driveStick.buttonA.whenPressed(new PathExecuter(toCargo, "TestCargoPath"));		
		//driveStick.buttonB.whenPressed(new EncoderDrive(-0.6, -0.6, -0.6));
		//driveStick.buttonY.whenPressed(new TurnDegrees(135, 3));
		//driveStick.buttonX.whenPressed(new PathExecuter(toHatchDispenser, "TestHatchPath"));
		//driveStick.buttonA.whenPressed(new PathExecuter("simpleslide_left.csv","simpleslide_right.csv", "TestPath"));
		//driveStick.buttonA.whenPressed(new VisionAllignment());
		//driveStick.buttonB.whenPressed(new TurnDegreesVision(20));
		//driveStick.buttonA.whenPressed(new PathExecuter("TestPath"));

		secondStick.buttonRBumper.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.DOWN));
		secondStick.buttonLBumper.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.DOWN));
		secondStick.buttonX.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_SECOND));
		secondStick.buttonA.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_FIRST));
		secondStick.buttonY.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_THIRD));
		secondStick.buttonB.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.CARGO_SHIP));

		secondStick.buttonStart.whenPressed(new SlideHatchOut());
		secondStick.buttonBack.whenPressed(new SlideHatchIn());

		driveStick.buttonRBumper.whenPressed(new ReleaseHatch());
		driveStick.buttonLBumper.whenPressed(new GraspHatch());

		// TODO: need to add controls for extending and retracting intake
	}

}
