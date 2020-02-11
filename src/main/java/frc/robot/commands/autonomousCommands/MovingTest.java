/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomousCommands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class MovingTest extends CommandBase {

  DriveTrain driveTrain;
  double metersToGo;
  double totalDistance;

  /**
   * Creates a new MovingTest.
   * 
   * @param I_driveTrain The DriveTrain subsystem
   * @param distance The distance in meters that the robot should move
   */
  public MovingTest(DriveTrain I_driveTrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = I_driveTrain;
    addRequirements(driveTrain);
    totalDistance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d start = new Pose2d(0, 0, new Rotation2d(0)); //start at 0, 0
    Pose2d end = new Pose2d(2, 0, new Rotation2d(0)); //end 2 meters ahead on the x

    List<Translation2d> interiorWaypoints = List.of( //waypoints for the robot to pass through

      new Translation2d(1,0) //Waypoint 1 meter ahead of start

    );

    TrajectoryConfig config = new TrajectoryConfig(Constants.maxVelocityMetersPerSecond, Constants.maxAccelerationMetersPerSecondSq);
    
    Trajectory move2Meters = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.timedDrive(0.25, 0); //drive straight forward
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //when we reach set point return true
  }
}
