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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  /**
   * Creates a new LimeLight.
   */
  double tx;
  double ty;
  double ta;
  NetworkTable table;

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {

    tx = Double.parseDouble(table.getEntry("tx").getValue().toString());
    ty = Double.parseDouble(table.getEntry("ty").getValue().toString());
    ta = Double.parseDouble(table.getEntry("ta").getValue().toString());
    // This method will be called once per scheduler run
  }

  public double[] getSphericalPosition(double angle) {
    double[] position = { (Constants.target_Height - Constants.sensor_Limelight_Height) / (Math.tan(angle + ty)), tx,
        ty };
    return position;
  }

  public double[] getCartesianPosition(double angle) {
    double[] Sposition = getSphericalPosition(angle);
    double[] position = { Sposition[0] * Math.cos(Sposition[1]) * Math.sin(Sposition[2]),
        Sposition[0] * Math.sin(Sposition[1]) * Math.sin(Sposition[2]), Sposition[0] * Math.cos(Sposition[2]) };
    return position;
  }
}
