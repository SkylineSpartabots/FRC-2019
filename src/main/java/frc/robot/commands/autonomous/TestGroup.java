package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.VibrateControllers;

public class TestGroup extends CommandGroup {


	public TestGroup() {
		//addSequential(new TurnDegrees(90, 6));
		//addSequential(new PathExecuter(path, "Test Path"));
		addSequential(new VibrateControllers(1, Robot.oi.driveStick));
		addSequential(new VibrateControllers(1, Robot.oi.secondStick));
		addSequential(new VibrateControllers(1, Robot.oi.secondStick, Robot.oi.driveStick));
	}
}
