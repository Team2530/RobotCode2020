/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class LargeJoystickDrive extends CommandBase {
  /**
   * Creates a new LargeJoystickDrive.
   */
  private DriveTrain m_driveTrain;

  Joystick stick1;

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

  public LargeJoystickDrive(DriveTrain driveTrain, Joystick stick1) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    this.stick1 = stick1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("joyy:", stick1.getY());
    SmartDashboard.putNumber("joyz:",  stick1.getZ());
    m_driveTrain.arcadeDrive(stick1.getY(),stick1.getZ());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Stop();
    m_driveTrain.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
