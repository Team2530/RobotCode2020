/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLimitSwitches;
import frc.robot.Constants.ElevatorMotors;

public class Elevator extends SubsystemBase {
  
  private static TalonSRX motor_Left_Leadscrew = new TalonSRX(Constants.motor_Left_Leadscrew_Port);
  private static TalonSRX motor_Right_Leadscrew = new TalonSRX(Constants.motor_Right_Leadscrew_Port);

  private static VictorSPX motor_Left_Pulley = new VictorSPX(Constants.motor_Left_Pulley_Port);
  private static VictorSPX motor_Right_Pulley = new VictorSPX(Constants.motor_Right_Pulley_Port);

  // private static Encoder encoder_Left_Leadscrew = new Encoder(Constants.encoder_Left_Leadscrew_Ports[0],Constants.encoder_Left_Leadscrew_Ports[1]);
  // private static Encoder encoder_Right_Leadscrew = new Encoder(Constants.encoder_Right_Leadscrew_Ports[0],Constants.encoder_Right_Leadscrew_Ports[1]);

  private static DigitalInput limit_Switch_Left_Leadscrew = new DigitalInput(Constants.limit_Switch_Left_Leadscrew_Port);
  private static DigitalInput limit_Switch_Right_Leadscrew = new DigitalInput(Constants.limit_Switch_Right_Leadscrew_Port);

  private static DigitalInput limit_Switch_Left_Pulley = new DigitalInput(Constants.limit_Switch_Left_Pulley_Port);
  private static DigitalInput limit_Switch_Right_Pulley = new DigitalInput(Constants.limit_Switch_Right_Pulley_Port);
  
  /**
   * Creates a new Elevator.
   */
  public Elevator() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //TODO create functions

  public void setMotorPower(final ElevatorMotors id, final double speed) {
    switch (id) {

      case LL:
        if(limit_Switch_Left_Leadscrew.get() && speed > 0) { // if limit switch is pressed and it wants to go up, dont
          motor_Left_Leadscrew.set(ControlMode.PercentOutput, 0);
          return;
        } else {
          motor_Left_Leadscrew.set(ControlMode.PercentOutput, speed);
          return;
        }

      case RL:
        if(limit_Switch_Right_Leadscrew.get() && speed > 0) { // if limit switch is pressed and it wants to go up, dont
          motor_Right_Leadscrew.set(ControlMode.PercentOutput, 0);
          return;
        } else {
          motor_Right_Leadscrew.set(ControlMode.PercentOutput, speed);
          return;
        }

      case LP:
        if(limit_Switch_Left_Pulley.get() && speed > 0) { // if limit switch is pressed and it wants to go up, dont
          motor_Left_Pulley.set(ControlMode.PercentOutput, 0);
          return;
        } else {
          motor_Left_Pulley.set(ControlMode.PercentOutput, speed);
          return;
        }

      case RP:
        if(limit_Switch_Right_Pulley.get() && speed > 0) { // if limit switch is pressed and it wants to go up, dont
          motor_Right_Pulley.set(ControlMode.PercentOutput, 0);
          return;
        } else {
          motor_Right_Pulley.set(ControlMode.PercentOutput, speed);
          return;
        }

      default:
        return;
    }
  }

  //TODO get Angle function return degrees
  public double getAngle() { 
    /**
     * pusdo code
     * angle = arctan(getHeight()/bottomLeg)
     */

     double radians = Math.atan2(Constants.bottomLeg, getHeight()); //(x,y)

     double degrees = (radians * 180)/Math.PI;

    return degrees;
  }
  
  //TODO get Height function return inches
  public double getHeight(){
    /**
     * pusdo code 
     * cant really do this until i know more specs of elevator from hardware
     * return +- from level i think would be easiest for getAngle()
     * 
     * * any gearing?
     * 
     * encoder pos/magic number (1024) = number of turns?
     * 
     * number of turns/turns per inch = inches traveled
     * 
     * if you just set enocder to 0 where you want inches to be 0 then never have to reset
     * will just tell you inches from that point
     * need to figure out how to reset at that 0 point
     * 
     */

    double encoderLeftPos = motor_Left_Leadscrew.getSelectedSensorPosition(1); //* if 1 doesnt work try 0, look at pheonix tuner
    double encoderRightPos = motor_Right_Leadscrew.getSelectedSensorPosition(1);

    double numberOfTurnsLeft = encoderLeftPos/Constants.ENCODER_TICKS_PER_REVOLUTION;
    double numberOfTurnsRight = encoderRightPos/Constants.ENCODER_TICKS_PER_REVOLUTION;

    double leftHeight = numberOfTurnsLeft/Constants.turnsPerInch;
    double RightHeight = numberOfTurnsRight/Constants.turnsPerInch;

    return 0; //average them maybe?
  }
  
  public void Stop() {
    for (final ElevatorMotors motor : ElevatorMotors.values()) {
      setMotorPower(motor, 0);
    }
  }

  public boolean getLimitSwitchValue(final ElevatorLimitSwitches id) { // * true = pressed, false = not pressed
    switch(id) { //!make sure limit switches are wired correctly 
      case LL:
        return limit_Switch_Left_Leadscrew.get(); 
      case RL:
        return limit_Switch_Right_Leadscrew.get();
      case LP:
        return limit_Switch_Left_Pulley.get();
      case RP:
        return limit_Switch_Right_Pulley.get();
      default:
        return false; //uhm false? idk
    }
  }
  
}
