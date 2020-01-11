/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// import frc.robot.Robot;
import frc.robot.Constants.DriveMotors;
import frc.robot.subsystems.DriveTrain;

public class DualLargeJoystickDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveTrain m_driveTrain;

  Joystick stick1;
  Joystick stick2;

  double y1;
  double y2;
  double z1;

  double leftPow;
  double rightPow;

  /**
   * Creates a new Command.
   *
   * @param drivetrain The subsystem used by this command.
   */

  public DualLargeJoystickDrive(DriveTrain driveTrain, Joystick stick1, Joystick stick2) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stick1 = Robot.m_robotContainer.getJoystick();
    // stick2 = Robot.m_robotContainer.getJoystick2();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // x1 = stick1.getX();
    y1 = stick1.getY();
    // x2 = stick2.getX();
    y2 = stick2.getY();
    if (Math.abs(y2) <= Constants.deadzone) {
      y2 = 0;
    }
    if (Math.abs(y1) <= Constants.deadzone) {
      y1 = 0;
    }

    rightPow = (y1);
    leftPow = (y2); 

    double powerfactor = -stick1.getRawAxis(3);
    powerfactor = 0.5 * (powerfactor + 1);

    rightPow = powerfactor * (0.5 * Math.pow(rightPow, 3) + 0.5 * rightPow);
    leftPow = powerfactor * (0.5 * Math.pow(leftPow, 3) + 0.5 * leftPow);

    m_driveTrain.setMotorPower(DriveMotors.FR, rightPow);
    m_driveTrain.setMotorPower(DriveMotors.BR, rightPow);

    m_driveTrain.setMotorPower(DriveMotors.FL, leftPow);
    m_driveTrain.setMotorPower(DriveMotors.BL, leftPow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
