/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LimeLight extends SubsystemBase {
  /**
   * Creates a new LimeLight.
   */
  double tv;
  double tx;
  double ty;
  double ta;
  NetworkTable table;
  int light = 1;

  public LimeLight() {

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {

    // tx = Double.parseDouble(table.getEntry("tx").getValue().toString());
    // ty = Double.parseDouble(table.getEntry("ty").getValue().toString());
    // ta = Double.parseDouble(table.getEntry("ta").getValue().toString());

    tv = table.getEntry("tv").getDouble(0.0);
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("light", light);

    if (tv == 1) {
      if (Robot.arduinoOutbox == new byte[] {00}) {
        Robot.arduinoOutbox = new byte[] {20};
      } else if (Robot.arduinoOutbox == new byte[] {10}) {
        Robot.arduinoOutbox = new byte[] {12};
      }
    }
    // This method will be called once per scheduler run
  }

  public double[] getSphericalPosition(double angle, double height) {
    double[] position = { (Constants.target_Height - height) / (Math.tan(angle + ty)), tx, ty };
    return position;
  }

  public double[] getCartesianPosition(double angle, double height) {
    double[] Sposition = getSphericalPosition(angle, height);
    double[] position = { Sposition[0] * Math.cos(Sposition[1]) * Math.sin(Sposition[2]),
        Sposition[0] * Math.sin(Sposition[1]) * Math.sin(Sposition[2]), Sposition[0] * Math.cos(Sposition[2]) };
    return position;
  }

  public void toggleLights()
  {
    if(light == 1)
    {
      light = 3;
    }
    else
    {
      light = 1;
    
    }
    
    table.getEntry("ledMode").setNumber(light);
  }
}
