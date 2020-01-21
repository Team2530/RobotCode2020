/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  // private static final Port i2c_port_id = null;
  /**
   * Creates a new DriveTrain.
   */
  private static VictorSPX motor_Front_Left = new VictorSPX(Constants.motor_Front_Left_Port);
  private static TalonSRX motor_Back_Left = new TalonSRX(Constants.motor_Back_Left_Port);
  private static TalonSRX motor_Back_Right = new TalonSRX(Constants.motor_Back_Right_Port);
  private static VictorSPX motor_Front_Right = new VictorSPX(Constants.motor_Front_Right_Port);
  
  private static Encoder encoder_Left = new Encoder(Constants.encoder_Left_Ports[0],Constants.encoder_Left_Ports[1]);
  private static Encoder encoder_Right = new Encoder(Constants.encoder_Right_Ports[0],Constants.encoder_Right_Ports[1]);

  public static AHRS ahrs = new AHRS();//! NEED A PORT ID
  public DriveTrain() {

  }

  public void resetEncoders()
  {
    encoder_Left.reset();
    encoder_Right.reset();
  }

  public void setMotorPower(final DriveMotors id, final double speed) {
    switch (id){// TODO THESE ARE ARBITRARY
      case FL:
        motor_Front_Left.set(ControlMode.PercentOutput, speed);
        return;
      case BR:
        motor_Back_Right.set(ControlMode.PercentOutput, -speed);
        return;
      case BL:
        motor_Back_Left.set(ControlMode.PercentOutput, speed);
        return;
      case FR:
        motor_Front_Right.set(ControlMode.PercentOutput, -speed);
        return;
      default:
        return;
    }
  }
  
  public void Stop() 
  {
    for (final DriveMotors motor : DriveMotors.values()) {
      setMotorPower(motor, 0);
    }
  }

  //TODO:complete drive for Distance
  public void driveDistance(double distance, double power)//Distance is inches and set power to negative to go backwards
  {
    double encoderdistance = Constants.ENCODER_TICKS_PER_REVOLUTION*Math.PI*Math.pow(Constants.WHEEL_RADIUS, 2)*distance; 

    resetEncoders();
    //starts all motors at starting speed
    for (final DriveMotors motor : DriveMotors.values()) {
      setMotorPower(motor, power);
    }

    /* while(getGreatestEncoder()<encoderdistance){
      if()
    } */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
