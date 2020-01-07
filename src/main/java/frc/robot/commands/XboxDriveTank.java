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
import frc.robot.Robot;
import frc.robot.RobotMap;

public class XboxDriveTank extends Command {

  XboxController xbox;

  double y1;
  double y2;

  double leftPow;
  double rightPow;

  public XboxDriveTank() {
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
  protected void execute() {
    y1 = xbox.getY(Hand.kLeft);
    y2 = xbox.getY(Hand.kRight);
    if (y2 >= -RobotMap.deadzone && y2 <=RobotMap.deadzone) {
      y2 = 0;
    }
    if (y1 >= -RobotMap.deadzone && y1 <= RobotMap.deadzone) {
      y1 = 0;
    }
    

    rightPow = (y1);
    leftPow = (y2); // should? be tank drive

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
