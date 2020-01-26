/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.DriveMotors;

public class LineUp extends CommandBase {

  private DriveTrain driveTrain;
  private LimeLight limeLight;
  private Elevator elevator;

  /**
   * Creates a new LineUp.
   */
  public LineUp(DriveTrain driveTrain, LimeLight limeLight, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.elevator = elevator;

    addRequirements(driveTrain);
    addRequirements(limeLight);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] position = limeLight.getSphericalPosition(elevator.getAngle(), elevator.getLimeLightHeight());
    double correction = 0.2;
    if(position[1] > 0) {
      driveTrain.setMotorPower(DriveMotors.FL, -correction);
      driveTrain.setMotorPower(DriveMotors.BL, -correction);
      driveTrain.setMotorPower(DriveMotors.FR, correction);
      driveTrain.setMotorPower(DriveMotors.BR, correction);
    } else if (position[1] < 0) {
      driveTrain.setMotorPower(DriveMotors.FL, correction);
      driveTrain.setMotorPower(DriveMotors.BL, correction);
      driveTrain.setMotorPower(DriveMotors.FR, -correction);
      driveTrain.setMotorPower(DriveMotors.BR, -correction);
    } else {
      driveTrain.setMotorPower(DriveMotors.FL, 0);
      driveTrain.setMotorPower(DriveMotors.BL, 0);
      driveTrain.setMotorPower(DriveMotors.FR, 0);
      driveTrain.setMotorPower(DriveMotors.BR, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
