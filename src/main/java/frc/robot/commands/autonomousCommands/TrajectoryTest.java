/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.controller.RamseteController;


import frc.robot.Constants;

public class TrajectoryTest extends RamseteCommand {
  
  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 60);
  DriveTrain m_driveTrain;

  /**
   * Creates a new TrajectoryTest.
   */
  public TrajectoryTest(DriveTrain m_driveTrain, Trajectory trajectory) {
    super (
      trajectory, // TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/DriveForwardFarBlue.wpilib.json")), //The Trajectory 
      m_driveTrain::getPose, //Pose (Supplier)
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), //Ramsete Controller
      m_driveTrain.getFeedForward(), //Feed Forward
      m_driveTrain.getKinematics(), //Kinematics
      m_driveTrain::getWheelSpeeds, //Wheel Speeds (Supplier)
      m_driveTrain.getController(), //Left PID Controller
      m_driveTrain.getController(), //Right PID Controller
      // RamseteCommand passes volts to the callback
      m_driveTrain::tankDriveVolts, //Function that uses the Output Volts (BiConsumer)
      m_driveTrain //SubSystem Requirments
    );

    // // String trajectoryJSON = "PathWeaver/DriveForwardFarBlue.wpilib.json";
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 60);
    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }
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
