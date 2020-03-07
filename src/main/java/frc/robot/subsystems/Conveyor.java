/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {

  private static WPI_VictorSPX motor_Right = new WPI_VictorSPX(Constants.motor_Conveyor_Right_Port);
  private static WPI_VictorSPX motor_Left = new WPI_VictorSPX(Constants.motor_Conveyor_Left_Port);

  /**
   * Creates a new Conveyor.
   */
  public Conveyor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void in() {
    motor_Right.set(ControlMode.PercentOutput, 1);
    motor_Left.set(ControlMode.PercentOutput, 1);
  }

  public void out() {
    motor_Right.set(ControlMode.PercentOutput, -1);
    motor_Left.set(ControlMode.PercentOutput, -1);
  }

  public void stopIntake() {
    motor_Right.set(ControlMode.PercentOutput, 0);
    motor_Left.set(ControlMode.PercentOutput, 0);
  }
}
