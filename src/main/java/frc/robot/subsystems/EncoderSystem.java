/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderSystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final double shaftDiameter = 0.5; //driver
  private final double wheelDiameter = 6; //driven
  private final double driverC = shaftDiameter * Math.PI;
  private final double drivenC = wheelDiameter * Math.PI;
  private double tics;
  private final double gearRatio = drivenC / driverC;
  private double distance1;
  
  public EncoderSystem() {
    //initialise encoder
  }


  public double calculateTics(int distance) {
    tics = 360 / driverC * distance;
    return tics;
  }
  
  //calculateDistance(calculateTics(desiredDistance));
  public double calculateDistance(int tics) {
    distance1 = tics/ 360 * driverC;
    return distance1 * gearRatio;
  }
  
//Encoder ticks = (360 / circumference) * Distance to travel
//Distance traveled = (Encoder ticks / 360) * circumference
//6 inch wheels
//1/2 inch shaft thing (connected to the wheel)


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
