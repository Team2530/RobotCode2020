/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static TalonSRX motor_Left_FlyWheel = new TalonSRX(Constants.motor_Left_FlyWheel_Port);
  private static TalonSRX motor_Right_FlyWheel = new TalonSRX(Constants.motor_Right_Flywheel_Port);

  private static TalonSRX motor_Ball_Leadscrew_Port = new TalonSRX(Constants.motor_Ball_Leadscrew_Port);

  private static Encoder encoder_Left = new Encoder(Constants.encoder_Left_Flywheel_Ports[0],Constants.encoder_Left_Flywheel_Ports[1]);
  private static Encoder encoder_Right = new Encoder(Constants.encoder_Right_Flywheel_Port[0],Constants.encoder_Right_Flywheel_Port[1]);

  public Shooter() {
    motor_Right_FlyWheel.set(ControlMode.Follower,Constants.motor_Left_FlyWheel_Port);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void startFW(){
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, 1);
  }
  public void stopFW(){
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, 1);
  }

  public void setFWPower(double power){
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, power);
  }
  public double getShooterEnergy(){
    double rpm = Constants.ENCODER_TICKS_PER_REVOLUTION*((encoder_Left.getDistance()+encoder_Right.getDistance())/2);
    
    return Constants.I*rpm*Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2);
  }
  public void fireBall(){
    
  }


}
