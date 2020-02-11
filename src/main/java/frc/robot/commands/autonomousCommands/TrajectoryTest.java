/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.*;

public class TrajectoryTest extends CommandBase {
  /**
   * Creates a new TrajectoryTest.
   */
  public TrajectoryTest() {
    String trajectoryJSON = "PathWeaver/DriveFOrwardFarBlue.wpilib.json";
    String configJSON = "pathweaver.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      Path configPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      TrajectoryConfig trajectoryConfig = TrajectoryUtil.fromPathweaverJson(configPath);
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
