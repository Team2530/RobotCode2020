/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Constants.DriveMotors;
import frc.robot.subsystems.DriveTrain;

public class DualLargeJoystickDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveTrain m_driveTrain;

  Joystick stick1;
  Joystick stick2;

  // double leftPow;
  // double rightPow;

  /**
   * Creates a new Command.
   *
   * @param drivetrain  The subsystem used by this command.
   * @param stick1      The left Joystick
   * @param stick2      The right Joystick
   */
  public DualLargeJoystickDrive(DriveTrain driveTrain, Joystick stick1, Joystick stick2) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    this.stick1 = stick1;
    this.stick2 = stick2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.tankDrive(stick1.getY(), -stick2.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
