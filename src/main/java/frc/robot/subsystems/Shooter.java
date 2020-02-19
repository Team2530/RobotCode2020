/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private static WPI_TalonSRX motor_Left_FlyWheel = new WPI_TalonSRX(Constants.motor_Left_FlyWheel_Port);
  private static WPI_TalonSRX motor_Right_FlyWheel = new WPI_TalonSRX(Constants.motor_Right_Flywheel_Port);
  private XboxController xbox;

  private static double currentSpeed = 0;

  // private static Encoder encoder_Left = new
  // Encoder(Constants.encoder_Left_Flywheel_Ports[0],Constants.encoder_Left_Flywheel_Ports[1]);
  // private static Encoder encoder_Right = new
  // Encoder(Constants.encoder_Right_Flywheel_Port[0],Constants.encoder_Right_Flywheel_Port[1]);

  public Shooter() {
    this.xbox = xbox;

    motor_Right_FlyWheel.follow(motor_Left_FlyWheel);
    motor_Right_FlyWheel.setInverted(true);
    // motor_Right_FlyWheel.set(ControlMode.Follower,
    // Constants.motor_Left_FlyWheel_Port);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Avg shooter speed", getAvgSpeed());
    if (xbox.getRawAxis(3) > 0) {
      startFW(currentSpeed);
    } else if (xbox.getRawAxis(3) < 0) {
      startFW(-0.3);
    } else {
      stopFW();
    }
    
    // This method will be called once per scheduler run
  }

  public void startFW(double speed) {
    // currentSpeed = 0.75; //for testing
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, speed);
    motor_Right_FlyWheel.set(ControlMode.PercentOutput, -speed);
  }

  public void stopFW() {
    // currentSpeed = 0;
    motor_Left_FlyWheel.set(ControlMode.PercentOutput, 0);
    motor_Right_FlyWheel.set(ControlMode.PercentOutput, 0);
  }

  // public void startIntake(double speed) {
  // //currentSpeed = 0.3;
  // motor_Left_FlyWheel.set(ControlMode.PercentOutput, -speed);
  // motor_Right_FlyWheel.set(ControlMode.PercentOutput, speed);
  // }

  /**
   * @return Average Velocity of flyweels in rad/s
   */
  public double getAvgSpeed() {
    return ((motor_Left_FlyWheel.getSelectedSensorVelocity() - motor_Right_FlyWheel.getSelectedSensorVelocity()) / 2);
    // / Constants.DROP_IN_DISTANCE_PER_REVOLUTION;
    // return 1.0;
  }

  public void startSpinning(double targetballspeed) {
    double wheelspeed = targetballspeed * Math.sqrt(
        Constants.I / (0.5 * Constants.ball_Weight + Constants.I / (Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2))));
    if (wheelspeed < Constants.MAX_SHOOTING_VEOLCITY && wheelspeed <= getAvgSpeed()) {
      currentSpeed += 0.05;
    }
    SmartDashboard.putNumber("Shooting Velocity", wheelspeed);
  }

  // public void setFWPower(double power) {
  // // motor_Left_FlyWheel.set(ControlMode.PercentOutput, power);
  // }

  public void fireBall() {

  }

  public void increaseSpeed() {
    currentSpeed = currentSpeed + 0.1;

    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

  public void decreaseSpeed() {

    currentSpeed = currentSpeed - 0.1;
    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

  public void setSpeed(double speed) {
    currentSpeed = speed;
    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

  public void setSpeed0() {
    currentSpeed = 0;
    SmartDashboard.putNumber("Current Shooter Speed", currentSpeed);
  }

}
