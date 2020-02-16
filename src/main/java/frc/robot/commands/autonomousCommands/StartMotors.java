/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveMotors;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StartMotors extends InstantCommand {
  DriveTrain driveTrain;
  public StartMotors(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveTrain.tankDrive(-0.3, 0.3);
    driveTrain.setSingleMotorPower(DriveMotors.BL, 0.3);
    driveTrain.setSingleMotorPower(DriveMotors.FL, 0.3);
    driveTrain.setSingleMotorPower(DriveMotors.BR, 0.3);
    driveTrain.setSingleMotorPower(DriveMotors.FR, 0.3);
    SmartDashboard.putBoolean("Auto Stopped", false);
  }
}
