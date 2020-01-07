/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class PositionalTracking extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  

  public AHRS ahrs = new AHRS(SPI.Port.kMXP); 
  Float[] rawgyro = new Float[3];
  Float[] rawacclerometer= new Float[3];
  Float[] acclerometer = new Float[3];//order is {x,y,z}={pitch,yaw,roll}
  Float[] displacement = new Float[3];//!Is EXPERIMENTAL
  Float rawcompass;
  Float fusedheading;

  boolean motionDetected;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void initAHRS(){
    ahrs.reset();
    motionDetected = ahrs.isMoving();
    rawacclerometer[0] = ahrs.getRawAccelX();
    rawacclerometer[1] = ahrs.getRawAccelY();
    rawacclerometer[2] = ahrs.getRawAccelZ();
    acclerometer[0] = ahrs.getPitch();
    acclerometer[1] = ahrs.getYaw();
    acclerometer[2] = ahrs.getRoll();
    rawgyro[0] = ahrs.getRawGyroX();
    rawgyro[1] = ahrs.getRawGyroY();
    rawgyro[2] = ahrs.getRawGyroZ();
    displacement[0] = ahrs.getDisplacementX();
    displacement[1] = ahrs.getDisplacementY();
    displacement[2] = ahrs.getDisplacementZ();
    fusedheading = ahrs.getFusedHeading();
    
  }
  //TODO idk what encoder we use so this is place holder
  public double[] getDriveEncodersAdjusted(){//Just an array of the encoder values with gear ratios,tic/rev,etc. already accounted for
    double encoder1 = 0;
    double encoder2 = 0;

    double[] encoders = {encoder1,encoder2};
    return encoders;
  }

  //TODO
  public void Magnetometercalibrate(){

  }
  public float getHeading(){
    return fusedheading;
  }
  public void logAll(){
    boolean motionDetected = ahrs.isMoving();
    SmartDashboard.putBoolean("MotionDetected", motionDetected);
    SmartDashboard.putNumber("rawAcclx", rawacclerometer[0]);
    SmartDashboard.putNumber("rawAccly", rawacclerometer[1]);
    SmartDashboard.putNumber("rawAcclz", rawacclerometer[2]);
    SmartDashboard.putNumber("rawgyrox", rawgyro[0]);
    SmartDashboard.putNumber("rawgyroy", rawgyro[1]);
    SmartDashboard.putNumber("rawgyroz", rawgyro[2]);
    SmartDashboard.putNumber("pitch",acclerometer[0]);
    SmartDashboard.putNumber("yaw",acclerometer[1]);
    SmartDashboard.putNumber("roll",acclerometer[2]);
    SmartDashboard.putNumber("displacementx", displacement[0]);
    SmartDashboard.putNumber("displacementy", displacement[1]);
    SmartDashboard.putNumber("displacementz", displacement[2]);
    SmartDashboard.putNumber("fused heading",fusedheading);
  }
}
