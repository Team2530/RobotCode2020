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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static WPI_TalonSRX motor_Left_FlyWheel = new WPI_TalonSRX(Constants.motor_Left_FlyWheel_Port);
  private static WPI_TalonSRX motor_Right_FlyWheel = new WPI_TalonSRX(Constants.motor_Right_Flywheel_Port);

  private static WPI_VictorSPX motor_Ball_Intake = new WPI_VictorSPX(Constants.motor_Ball_Pully_Port);
  private static double currentSpeed = 0;

  //private static Encoder encoder_Left = new Encoder(Constants.encoder_Left_Flywheel_Ports[0],Constants.encoder_Left_Flywheel_Ports[1]);
  //private static Encoder encoder_Right = new Encoder(Constants.encoder_Right_Flywheel_Port[0],Constants.encoder_Right_Flywheel_Port[1]);

  public Shooter() {
    motor_Right_FlyWheel.set(ControlMode.Follower,Constants.motor_Left_FlyWheel_Port);
  }

  @Override
  public void periodic() {
    setFWPower(currentSpeed);
    // This method will be called once per scheduler run
  }
  public void startFW(){
    currentSpeed = 1;
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, currentSpeed);
  }
  public void stopFW(){
    currentSpeed = 0;
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, currentSpeed);
  }
  /**
   * @return Average Velocity of flyweels in rad/s
   */
  public double getAvgSpeed(){
    return (motor_Left_FlyWheel.getSelectedSensorVelocity()+motor_Right_FlyWheel.getSelectedSensorVelocity())/2/Constants.DROP_IN_DISTANCE_PER_REVOLUTION;
  }

  public void startSpinning(double targetballspeed){
     double wheelspeed = targetballspeed*Math.sqrt(Constants.I/(0.5*Constants.ball_Weight+Constants.I/(Math.pow(Constants.SHOOTER_WHEEL_RADIUS,2))));
    if(wheelspeed<Constants.MAX_SHOOTING_VEOLCITY && wheelspeed<=getAvgSpeed()){
      currentSpeed += 0.05;
    }

  }
  public void setFWPower(double power){
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, power);
  }
  public void fireBall(){
    
  }



}
