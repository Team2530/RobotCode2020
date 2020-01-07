/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DualJoystickDrive extends Command {

  Joystick stick1;
  Joystick stick2;

  double y1;
  double y2;
  double z1;

  double leftPow;
  double rightPow;

  public DualJoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
    stick1 = Robot.m_oi.getJoystick();
    stick2 = Robot.m_oi.getJoystick2();
  }

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {
    // x1 = stick1.getX();
    y1 = stick1.getY();
    // x2 = stick2.getX();
    y2 = stick2.getY();
    if (Math.abs(y2) <= RobotMap.deadzone) {
      y2 = 0;
    }
    if (Math.abs(y1) <= RobotMap.deadzone) {
      y1 = 0;
    }

    rightPow = (y1);
    leftPow = (y2); 

    double powerfactor = -stick1.getRawAxis(3);
    powerfactor = 0.5 * (powerfactor + 1);
    SmartDashboard.putNumber("powerfactor", powerfactor);

    rightPow = powerfactor * (0.5 * Math.pow(rightPow, 3) + 0.5 * rightPow);
    leftPow = powerfactor * (0.5 * Math.pow(leftPow, 3) + 0.5 * leftPow);

    Robot.driveTrain.setMotorPower(0, rightPow);
    Robot.driveTrain.setMotorPower(2, rightPow);

    Robot.driveTrain.setMotorPower(1, leftPow);
    Robot.driveTrain.setMotorPower(3, leftPow);

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.Stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.Stop();
  }
}
