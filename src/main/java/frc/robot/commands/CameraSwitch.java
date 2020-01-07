/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
//import edu.wpi.first.wpilibj.CameraServer;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CameraSwitch extends Command {
  int camEnabled = 0;
  UsbCamera camera0;
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  
  public CameraSwitch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.camera);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    camera0 = Robot.camera.getCamera(0);
    camera1 = Robot.camera.getCamera(1);
    camera2 = Robot.camera.getCamera(2);
    server = Robot.camera.getServer();
    camera0.setBrightness(50);
    camera1.setBrightness(50);
    camera2.setBrightness(50);
    server.setSource(camera0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch(camEnabled) { //switch works but camera not showing up
      case 0: //camera0 is enabled
        server.setSource(camera1);
        camEnabled = 1;
        SmartDashboard.putNumber("camEnabled2", 1);
        break;
      case 1:
        server.setSource(camera2);
        camEnabled = 2;
        SmartDashboard.putNumber("camEnabled2", camEnabled);
        break;
      case 2:
        server.setSource(camera0);
        camEnabled = 0;
        SmartDashboard.putNumber("camEnabled2", camEnabled);
        break;
      default: 
        SmartDashboard.putNumber("camEnabled2", -1);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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