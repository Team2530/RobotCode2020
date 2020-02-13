/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.RamseteController;

import java.io.IOException;
import java.nio.file.*;

import frc.robot.Constants;

public class TrajectoryTest extends RamseteCommand {
  /**
   * Creates a new TrajectoryTest.
   */
  String trajectoryJSON = "PathWeaver/DriveForwardFarBlue.wpilib.json";
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 60);
    DriveTrain m_driveTrain;
    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }
  public TrajectoryTest(DriveTrain m_driveTrain,Trajectory trajectory) {
    super (
      TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/DriveForwardFarBlue.wpilib.json")),
      m_driveTrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      m_driveTrain::getFeedForward,
      m_driveTrain::getKinematics,
      m_driveTrain::getWheelSpeeds,
      m_driveTrain::getController,
      m_driveTrain::getController,
      // RamseteCommand passes volts to the callback
      m_robotDrive::tankDriveVolts,
      m_robotDrive
  ));
    String trajectoryJSON = "PathWeaver/DriveFOrwardFarBlue.wpilib.json";
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 60);
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
