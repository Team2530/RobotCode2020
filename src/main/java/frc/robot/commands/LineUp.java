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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.Constants.DriveMotors;

public class LineUp extends CommandBase {

  private DriveTrain driveTrain;
  private LimeLight limeLight;
  private Elevator elevator;
  private double[] position;
  private double correction = 0.2;
  private double power;
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
    power = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = position[1];
    double[] position = limeLight.getSphericalPosition(elevator.getAngle(), elevator.getLimeLightHeight());
    SmartDashboard.putNumber("position[0]", position[0]);
    SmartDashboard.putNumber("position[1]", position[1]);
    if(position[1] > 0 + Constants.angleTolerance) { //turn right, but its turning left? but it works

      driveTrain.setMotorPower(DriveMotors.FL, -power);
      driveTrain.setMotorPower(DriveMotors.BL, -power);
      driveTrain.setMotorPower(DriveMotors.FR, power);
      driveTrain.setMotorPower(DriveMotors.BR, power);

    } else if (position[1] < 0 - Constants.angleTolerance) { //turn left, but its turning right? but it works

      driveTrain.setMotorPower(DriveMotors.FL, power);
      driveTrain.setMotorPower(DriveMotors.BL, power);
      driveTrain.setMotorPower(DriveMotors.FR, -power);
      driveTrain.setMotorPower(DriveMotors.BR, -power);

    } else { //correct spot in regards to x angle

      if(position[0] > 96 + Constants.distanceTolerance) { //go forward

        driveTrain.setMotorPower(DriveMotors.FL, power);
        driveTrain.setMotorPower(DriveMotors.BL, power);
        driveTrain.setMotorPower(DriveMotors.FR, power);
        driveTrain.setMotorPower(DriveMotors.BR, power);

      } else if(position[0] < 96 - Constants.distanceTolerance) { //go backward

        driveTrain.setMotorPower(DriveMotors.FL, -power);
        driveTrain.setMotorPower(DriveMotors.BL, -power);
        driveTrain.setMotorPower(DriveMotors.FR, -power);
        driveTrain.setMotorPower(DriveMotors.BR, -power);

      } else { //dont move, correct spot

        driveTrain.setMotorPower(DriveMotors.FL, 0);
        driveTrain.setMotorPower(DriveMotors.BL, 0);
        driveTrain.setMotorPower(DriveMotors.FR, 0);
        driveTrain.setMotorPower(DriveMotors.BR, 0);

      }
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
