package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TurnPIDTest extends CommandGroup {

	public TurnPIDTest() {
		addSequential(new TurnDegrees(90, 6));
	}
}
