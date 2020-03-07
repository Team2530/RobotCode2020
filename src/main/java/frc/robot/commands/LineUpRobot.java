/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
// import frc.robot.Constants.DriveMotors;

public class LineUpRobot extends CommandBase {

  private DriveTrain driveTrain;
  private LimeLight limeLight;
  // private Elevator elevator;
  private double[] position;
  // private double correction = 0.2;
  private double power = 1;

  /**
   * Creates a new LineUp.
   */
  public LineUpRobot(DriveTrain driveTrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    // this.elevator = elevator;

    addRequirements(driveTrain);
    addRequirements(limeLight);
    // addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    power = 0.045;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = limeLight.getCylindricalPosition(Constants.sensor_Limelight_Angle, Constants.sensor_Limelight_Height);
    SmartDashboard.putNumber("position[0]", position[0]);
    SmartDashboard.putNumber("position[1]", position[1]);
    SmartDashboard.putNumber("position[2]", position[2]);
    // driveTrain.setSetpoint(position[1]); //this will not work, but this is the idea of what we need to do
    driveTrain.alignToTarget(power, 0, Constants.IDEAL_SHOOTING_DISTANCE, position[1], position[0]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.alignToTarget(power, 0, Constants.IDEAL_SHOOTING_DISTANCE, position[1], position[0]);
  }
}
