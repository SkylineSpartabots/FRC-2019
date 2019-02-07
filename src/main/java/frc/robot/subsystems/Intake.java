/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.*;
/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX leftIntakeMotor, rightIntakeMotor, centerIntakeMotor;
  Solenoid leftIntakeSolenoid, rightIntakeSolenoid;

  public Intake() {
    leftIntakeMotor = new WPI_TalonSRX(RobotMap.leftIntakeMotor);
    rightIntakeMotor = new WPI_TalonSRX(RobotMap.rightIntakeMotor);
    centerIntakeMotor = new WPI_TalonSRX(RobotMap.centerIntakeMotor);

    leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
    centerIntakeMotor.setNeutralMode(NeutralMode.Brake);

    leftIntakeSolenoid = new Solenoid(RobotMap.leftIntakeSolenoid);
    rightIntakeSolenoid = new Solenoid(RobotMap.rightIntakeSolenoid);
  }

  public void openCloseIntake(boolean open) {
    leftIntakeSolenoid.set(open);
    rightIntakeSolenoid.set(open);
  }

  public void setIntakePower(double power) {
    leftIntakeMotor.set(power);
    rightIntakeMotor.set(power);
    centerIntakeMotor.set(power);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RollIn());
  }
}
