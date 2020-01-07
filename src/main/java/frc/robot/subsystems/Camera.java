/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture(0);
  UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(1);
  UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(2);
  VideoSink server = CameraServer.getInstance().getServer();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public UsbCamera getCamera(int camera) {
     switch(camera) {
       case 0:
         return camera0;
       case 1:
         return camera1;
       case 2:
         return camera2;
       default:
         return camera0;
     }
  }

   public VideoSink getServer() {
     return server;
     }
}
