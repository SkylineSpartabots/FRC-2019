/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ElevatorManualAscent;
import frc.robot.commands.ElevatorManualDescent;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.ElevatorToPosition.ElevatorPositions;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick driveStick;
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
		LX(0), LY(1), LTrigger(2), RTrigger(3), RX(4), RY(5);
		private final int number;

		Axis(int number) {
			this.number = number;
		}

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

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

  public double clipDeadzone(double rawValue){
    if(Math.abs(rawValue) > 0.05){
      return rawValue;
    } else{
      return 0;
    }
  }



public OI() {
  driveStick = new Joystick(RobotMap.driveStick);

}


}
