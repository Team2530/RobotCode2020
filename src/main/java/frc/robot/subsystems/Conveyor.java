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

  private static WPI_VictorSPX motor_Conveyor = new WPI_VictorSPX(Constants.motor_Conveyor_Port);
  private static WPI_VictorSPX motor_Conveyor1 = new WPI_VictorSPX(Constants.motor_Conveyor_Port1);

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
    motor_Conveyor.set(ControlMode.PercentOutput, 1);
    motor_Conveyor1.set(ControlMode.PercentOutput, 1);
  }

  public void out() {
    motor_Conveyor.set(ControlMode.PercentOutput, -1);
    motor_Conveyor1.set(ControlMode.PercentOutput, -1);
  }

  public void stopIntake() {
    motor_Conveyor.set(ControlMode.PercentOutput, 0);
    motor_Conveyor1.set(ControlMode.PercentOutput, 0);
  }
}
