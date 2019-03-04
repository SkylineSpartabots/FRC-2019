package frc.robot;

import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.EncoderDrive;
import frc.robot.commands.GraspHatch;
import frc.robot.commands.PathExecuter;
import frc.robot.commands.PerfectlyStraightDrive;
import frc.robot.commands.ReleaseHatch;
import frc.robot.commands.SlideHatchIn;
import frc.robot.commands.SlideHatchOut;
import frc.robot.commands.TurnDegrees;
import frc.robot.controllers.Logitech;
import frc.robot.subsystems.Elevator;
import jaci.pathfinder.Waypoint;

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
		/*
			Start Point: bottom of platform (left side)
			End Point: Near Cargo
			new Waypoint(0, 0, 0),
			new Waypoint(3.25, 1.25, 0),
		*/
		Waypoint[] toCargo = new Waypoint[]{
			new Waypoint(0, 0, 0),
			new Waypoint(3.25, 1.25, 0),
			//new Waypoint(0.1, 1.5, 0),	
		};
		Waypoint[] toHatchDispenser = new Waypoint[]{
			new Waypoint(0, 0, 0),
			new Waypoint(3, 1, Math.PI/4),
			//new Waypoint(0.1, 1.5, 0),	
		};
		driveStick.buttonA.whenPressed(new PathExecuter(toCargo, "TestCargoPath"));		
		driveStick.buttonB.whenPressed(new EncoderDrive(-0.6, -0.6, -0.6));
		driveStick.buttonY.whenPressed(new TurnDegrees(135, 3));
		driveStick.buttonX.whenPressed(new PathExecuter(toHatchDispenser, "TestHatchPath"));
		//driveStick.buttonA.whenPressed(new PathExecuter("simpleslide_left.csv","simpleslide_right.csv", "TestPath"));

		secondStick.buttonX.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_SECOND));
		secondStick.buttonA.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_FIRST));
		secondStick.buttonY.whenPressed(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_THIRD));

		secondStick.buttonLBumper.whenPressed(new GraspHatch());
		secondStick.buttonRBumper.whenPressed(new ReleaseHatch());

		secondStick.buttonStart.whenPressed(new SlideHatchOut());
		secondStick.buttonBack.whenPressed(new SlideHatchIn());

		// TODO: need to add controls for extending and retracting intake

	}

}
