/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class XboxDrive extends Command {

  XboxController xbox;

  double x1;
  double x2;
  double y1;
  double y2;

  double leftPow;
  double rightPow;

  public XboxDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
    xbox = Robot.m_oi.getXbox();
  }

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {//this can be switched
    x2 = Robot.control.exponentialRebind(-xbox.getX(Hand.kLeft));
    y2 = Robot.control.exponentialRebind(xbox.getY(Hand.kLeft));
    x1 = Robot.control.exponentialRebind(-xbox.getX(Hand.kRight));
    y1 = Robot.control.exponentialRebind(xbox.getY(Hand.kRight));
    if (x1 >= -RobotMap.xboxDeadzone && x1 <= RobotMap.xboxDeadzone) {
      x1 = 0;
    }
    if (x2 >= -RobotMap.xboxDeadzone && x2 <= RobotMap.xboxDeadzone) {
      x2 = 0;
    }
    if (y2 >= -RobotMap.xboxDeadzone && y2 <= RobotMap.xboxDeadzone) {
      y2 = 0;
    }
    if (y1 >= -RobotMap.xboxDeadzone && y1 <= RobotMap.xboxDeadzone) {
      y1 = 0;
    }
    SmartDashboard.putNumber("x", x1);
    SmartDashboard.putNumber("y", y1);
    SmartDashboard.putNumber("x2", x2);

    Robot.driveTrain.setMotorPower(0,y1-x2+x1);//br
    Robot.driveTrain.setMotorPower(2,y1-x2-x1);//fr

    Robot.driveTrain.setMotorPower(1,y1+x2+x1);//fl
    Robot.driveTrain.setMotorPower(3, y1+x2-x1);//bl

    
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
