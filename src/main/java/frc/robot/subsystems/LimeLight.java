/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  
  double tx;
  double ty;
  double ta;
  NetworkTable table;
  int light = 3;
  boolean limelightDriveCamOn = false;
  
  /**
   * Creates a new LimeLight.
   */
  public LimeLight() {

    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("camMode").setNumber(0);
    table.getEntry("ledMode").setNumber(3);
    table.getEntry("pipeline").setNumber(1);
  }

  @Override
  public void periodic() {

    // tx = Double.parseDouble(table.getEntry("tx").getValue().toString());
    // ty = Double.parseDouble(table.getEntry("ty").getValue().toString());
    // ta = Double.parseDouble(table.getEntry("ta").getValue().toString());

    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("light", light);
    // This method will be called once per scheduler run
  }

  public double[] getCylindricalPosition(double angle, double height) { 
    final double[] position = { ((Constants.target_Height - height) / (Math.tan(Math.toRadians(angle + ty)))), tx, Constants.target_Height -height };
    return position;
  }

  public double[] getCartesianPosition(double angle, double height) {
    final double[] Cposition = getCylindricalPosition(angle, height);
    final double[] position = { Cposition[0] * Math.cos(Cposition[1]),
        Cposition[0] * Math.sin(Cposition[1]), Cposition[2] };
    return position;
  }
  public Pose2d getPose2d(){
    //!BROKE
    return new Pose2d(tx, ty, new Rotation2d(Math.toRadians(tx)));
  }
  public Pose2d getPose2dFromTarget(){
    return null;
  }

  public void toggleLights() {
    if(light == 1) {
      light = 3;
    } else {
      light = 1;
    }
    table.getEntry("ledMode").setNumber(light);
  }

  public void switchCamera() {
    if (limelightDriveCamOn == true) {
      table.getEntry("camMode").setNumber(0);
      // table.getEntry("pipeline").setNumber(1);
      limelightDriveCamOn = false;
    } else {
      // table.getEntry("pipeline").setNumber(1);
      table.getEntry("camMode").setNumber(1);
      limelightDriveCamOn = true;
    }
  }
}
