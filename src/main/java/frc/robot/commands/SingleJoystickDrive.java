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

public class SingleJoystickDrive extends Command {
  
  Joystick stick;
  
  double y1;
  
  double x1;
  double z1;

  double leftPow;
  double rightPow;

  double powerfactor;
  
  public SingleJoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time



  @Override
  protected void initialize() {
    stick = Robot.m_oi.getJoystick();
  }

  // Called repeatedly when this Command is scheduled to run
  
  @Override
  protected void execute() {
    x1 = RobotMap.driveDirection*stick.getX();
    y1 = -RobotMap.driveDirection*stick.getY();
    z1 = -stick.getZ();
    if (Math.abs(x1) <= RobotMap.deadzone) {
      x1 = 0;
    } 
    if (Math.abs(y1) <= RobotMap.deadzone) {
      y1 = 0;
    } 
    if (Math.abs(z1) <= RobotMap.zDeadzone) {
      z1 = 0;
    }

    powerfactor = -stick.getRawAxis(3);
    powerfactor = 0.5 * (powerfactor + 1); //changes max power based on slider
    SmartDashboard.putNumber("powerfactor", powerfactor);

    x1 = powerfactor*(0.5 * Math.pow(x1, 3) + 0.5 * x1);
    y1 = powerfactor*(0.5 * Math.pow(y1, 3) + 0.5 * y1); //fancy exponent growth stuff
    z1 = powerfactor*(0.5 * Math.pow(z1, 3) + 0.5 * z1);


    SmartDashboard.putNumber("x", x1);
    SmartDashboard.putNumber("y", y1);
    SmartDashboard.putNumber("z", z1);

  
    Robot.driveTrain.setMotorPower(0,y1-z1+x1);//br
    Robot.driveTrain.setMotorPower(2,y1-z1-x1);//fr

    Robot.driveTrain.setMotorPower(1,y1+z1+x1);//fl
    Robot.driveTrain.setMotorPower(3, y1+z1-x1);//bl

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
