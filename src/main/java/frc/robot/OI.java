/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.GraspHatch;
import frc.robot.commands.ReleaseHatch;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick driveStick;
  public Joystick secondStick;
   

	public enum Button {
		RBumper(6), LBumper(5), A(1), B(2), X(3), Y(4), RightJoystickBtn(10), LeftJoystickBtn(9);

		private final int number;

		Button(int number) {
			this.number = number;
		}

		public int getBtnNumber() {
			return number;
		}
		
  }

  public enum Axis {
    RX(5), LX(6), RTrigger(7), LTrigger(8), LY(9), RY(10);

    public final int number;

    private Axis(int number) {
      this.number = number;
    }

    public int getBtnNumber() {
      return number;
    }

  }

  /**
   * 
   * @param rawValue
   * @return clips deadzone (specified in RobotMap) from joysticks and triggers when value
   * passed in
   */
  public double clipDeadzone(double rawValue){
    if(Math.abs(rawValue) > RobotMap.JOYSTICK_DEADZONE){
      return rawValue;
    } else{
      return 0;
    }
  }




  public OI() {
  driveStick = new Joystick(RobotMap.driveStick);
  secondStick = new Joystick(RobotMap.secondStick);

  new JoystickButton(secondStick, OI.Button.A.getBtnNumber())
    .whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.ROCKET_FIRST));;

  new JoystickButton(secondStick, OI.Button.B.getBtnNumber())
    .whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.CARGO_SHIP));

  new JoystickButton(secondStick, OI.Button.X.getBtnNumber())
    .whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.ROCKET_SECOND));

  new JoystickButton(secondStick, OI.Button.Y.getBtnNumber())
    .whenPressed(new ElevatorToPosition(Elevator.ElevatorPositions.ROCKET_THIRD));

  new JoystickButton(secondStick, OI.Button.RBumper.getBtnNumber())
    .whenPressed(new GraspHatch());

  new JoystickButton(secondStick, OI.Button.LBumper.getBtnNumber())
    .whenPressed(new ReleaseHatch());

  //TODO: need to add controls for extending and retracting intake

}


}
