/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleopDriveCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class LargeJoystickCurvatureDrive extends CommandBase {

  DriveTrain driveTrain;

  Joystick stick;
  double y;
  double z;

  /**
   * Creates a new LargeJoystickCurvatureDrive.
   */
  public LargeJoystickCurvatureDrive(DriveTrain driveTrain, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.stick = stick;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    y = stick.getY();
    z = stick.getZ();

    driveTrain.curvatureDrive(y, z, true); // permanently set false for now, add a button to toggle soon
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
