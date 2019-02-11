/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick driveStick;
  public OI() {
    driveStick = new Joystick(RobotMap.driveStick); 
  }
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
    public int getAxisNumber() {
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
}