/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private static VictorSPX motor_Left_Leadscrew = new VictorSPX(Constants.motor_Left_Leadscrew_Port);
  private static TalonSRX motor_Right_Leadscrew = new TalonSRX(Constants.motor_Right_Leadscrew_Port);

  private static TalonSRX motor_Left_Pulley_Port = new TalonSRX(Constants.motor_Left_Pulley_Port);
  private static VictorSPX motor_Right_Pulley_Port = new VictorSPX(Constants.motor_Right_Pulley_Port);

  private static Encoder encoder_Left_Leadscrew = new Encoder(Constants.encoder_Left_Leadscrew_Ports[0],Constants.encoder_Left_Leadscrew_Ports[1]);
  private static Encoder encoder_Right_Leadscrew = new Encoder(Constants.encoder_Right_Leadscrew_Port[0],Constants.encoder_Right_Leadscrew_Port[1]);
  
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
  
}
