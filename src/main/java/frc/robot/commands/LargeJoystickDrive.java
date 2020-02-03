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

  Joystick stick;

  

  /**
   * Creates a new Command.
   *
   * @param drivetrain The subsystem used by this command.
   */

  public LargeJoystickDrive(DriveTrain driveTrain, Joystick stick) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    this.stick = stick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("joyy:", stick.getY());
    SmartDashboard.putNumber("joyz:",  stick.getZ());
    m_driveTrain.arcadeDrive(stick.getY(),stick.getZ());
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
