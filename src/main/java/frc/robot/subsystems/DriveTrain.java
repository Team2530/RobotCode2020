/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SingleJoystickDrive;
import frc.robot.commands.XboxDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;

//!!!!IMPORTANT NOTE!!!!     -slot 0 = Xbox controller  -slot 1 = stick1  -slot 2 = stick2

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new XboxDrive());
  }

  public void setMotorPower(int id, double speed) {
    switch (id) {//TODO THESE ARE ARBITRARY
    case 1:
      RobotMap.motor_Front_Left.set(ControlMode.PercentOutput,speed);
      return;
    case 0:
      RobotMap.motor_Back_Right.set(ControlMode.PercentOutput, -speed);
      return;
    case 3:
      RobotMap.motor_Back_Left.set(ControlMode.PercentOutput,speed);
      return;
    case 2:
      RobotMap.motor_Front_Right.set(ControlMode.PercentOutput, -speed);
      return;
    default:
      return;
    }

  }
  public void Stop() {
    for (int i = 0; i < 3; i++) {
      setMotorPower(i, 0);
    }
  }

  // ! do not use
  public void FlipDrive() {
    RobotMap.driveDirection *= -1;
  }

}
