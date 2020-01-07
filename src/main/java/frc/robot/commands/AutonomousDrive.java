/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class AutonomousDrive extends Command {

  Float distance;
  double defaultPower;
  double initialHeading;
  double leftPower;
  double rightPower;

  public AutonomousDrive(Float distance, double power) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    requires(Robot.positionalTracker);
    this.distance = distance;
    this.defaultPower = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.positionalTracker.initAHRS();
    leftPower = defaultPower;
    rightPower = defaultPower;
    initialHeading =Robot.positionalTracker.getHeading();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
  //?Robot.positionalTracker.getHeading()
  //?Robot.positionalTracker.getDriveEncodersAdjusted();
  if(initialHeading < Robot.positionalTracker.getHeading()+RobotMap.driveDelta)
  
  
    //left
  Robot.driveTrain.setMotorPower(0, leftPower);
  Robot.driveTrain.setMotorPower(1, leftPower);
  //right
  Robot.driveTrain.setMotorPower(2, rightPower);
  Robot.driveTrain.setMotorPower(3, rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}