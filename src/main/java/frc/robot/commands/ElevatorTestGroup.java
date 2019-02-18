package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Elevator;

public class ElevatorTestGroup extends CommandGroup {

	/**
	 * Simple elevator test program, moves it to all of the positions (cargo or no
	 * cargo) with a 2 second delay between each movement
	 */
	public ElevatorTestGroup() {
		addSequential(new ElevatorToPosition(Elevator.ElevatorPosition.CARGO_SHIP));
		addSequential(new Wait(2000));
		addSequential(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_FIRST));
		addSequential(new Wait(2000));
		addSequential(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_SECOND));
		addSequential(new Wait(2000));
		addSequential(new ElevatorToPosition(Elevator.ElevatorPosition.ROCKET_THIRD));
	}
}