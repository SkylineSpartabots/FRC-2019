package frc.robot;

import frc.robot.commands.drive_controls.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.auto_commands.PathExecuter;
import frc.robot.commands.auto_commands.StopDriveTrain;
import frc.robot.commands.auto_commands.TurnDegreesVision;
import frc.robot.commands.auto_commands.VisionAlignment;
import frc.robot.commands.auto_commands.VisionAlignmentWithTurn;
import frc.robot.commands.basic_commands.*;
import frc.robot.controllers.Xbox;
import frc.robot.subsystems.Elevator;
import jaci.pathfinder.Waypoint;

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

<<<<<<< Updated upstream
		//secondStick.buttonStart.whenPressed(new SlideHatchOut());
		//secondStick.buttonBack.whenPressed(new SlideHatchIn());
=======
		driveStick.buttonA.whenPressed(new AlignToTarget());
>>>>>>> Stashed changes

		//driveStick.buttonLBumper.whenPressed(new ReleaseHatch());
		//driveStick.buttonRBumper.whenPressed(new GraspHatch());
		//driveStick.buttonA.whenPressed(new StopDriveTrain());
		
		  driveStick.buttonA.whenPressed(new AlignAndPathToDepot());
		  driveStick.buttonB.whenPressed(new VisionAlignment());
		  driveStick.buttonX.whenPressed(new TurnDegreesVision(10));
		 
		  Waypoint[] points = new Waypoint[]{
			new Waypoint(0, 0, 0),
			new Waypoint(3, 1, 0)
		  };

		  driveStick.buttonY.whenPressed(new PathExecuter(new Waypoint[]{
															new Waypoint(0, 0, 0),
															new Waypoint(3, 1, 0)
														}, "Test", true));

/////////////////////////////////////////////////////////////////////////////////////
		//THis is good
		//driveStick.buttonB.whenPressed(new PickUpHatch());
		///driveStick.buttonA.whenPressed(new OutakeWithVision());
		//driveStick.buttonX.whenPressed(new AllignAndPathToTarget());
		

		// TODO: need to add controls for extending and retracting intake
	}

}
