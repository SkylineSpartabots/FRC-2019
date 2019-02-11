/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * 
 */
public class Elevator extends Subsystem {

  private WPI_TalonSRX elevatorMaster, elevatorSlave;
  public Encoder elevatorEncoder;
  private DigitalInput elevatorLimitSwitch;

  public final static int MAX_ENCODER_LIMIT = 1000; //TODO: Add limit
  public final static int MIN_ENCODER_LIMIT = 0; 
  
  public Elevator(){
    elevatorMaster = new WPI_TalonSRX(RobotMap.RightElevatorPort);
    elevatorSlave = new WPI_TalonSRX(RobotMap.LeftElevatorPort);

    elevatorMaster.setNeutralMode(NeutralMode.Brake);
    elevatorSlave.setNeutralMode(NeutralMode.Brake);   

    elevatorMaster.setInverted(false);
    elevatorSlave.setInverted(false);


    elevatorSlave.follow(elevatorMaster);

    elevatorEncoder = new Encoder(RobotMap.elevatorEncoderPorts[0], RobotMap.elevatorEncoderPorts[1]);
    elevatorLimitSwitch = new DigitalInput(RobotMap.elevatorLimitSwitch);
  }
  public void setPower(double power){
    boolean maxReached = elevatorEncoder.getRaw() >= MAX_ENCODER_LIMIT && power > 0;
    boolean minReached = elevatorEncoder.getRaw() <= MIN_ENCODER_LIMIT && power < 0;
    if(!maxReached || !minReached){
        elevatorMaster.set(power);
    } else{
        elevatorMaster.set(0);
    }
    if(getLimitSwitchState()){
        elevatorEncoder.reset();
    }
  }
  /**
   * 
   * @return returns the state of limit switch. True is active.
   */
  public boolean getLimitSwitchState(){
    return !elevatorLimitSwitch.get();
  }

  @Override
  public void initDefaultCommand() {
  }

  public static enum ElevatorControlPositions{
    DOWN_POSITION (0), //rocket first level hatch and cargo ship hatch
    CARGO_SHIP_CARGO (10),
    ROCKET_SHIP_FIRST_CARGO (100),
    ROCKET_SHIP_SECOND_HATCH (1000),
    ROCKET_SHIP_SECOND_CARGO (10000),
    ROCKET_SHIP_THIRD_HATCH (100000),
    ROCKET_SHIP_THIRD_CARGO (1000000);

    private final int position;

    private ElevatorControlPositions(int position){
      this.position = position;
    }

    public int getPosition(){
      return this.position;
    }
  }
}
