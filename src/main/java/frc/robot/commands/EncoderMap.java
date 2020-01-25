/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EncoderSystem;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */

public class EncoderMap extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EncoderSystem m_encoderSystem;
  private final DriveTrain m_driveTrain;
  private float encoderX;
  private float encoderY;
 
  //private float RobotQ;
  private float RobotQX;
  private float RobotQY;
  private float RobotQAngle;
  


  /**
   * Creates a new ExampleCommand.
   *
   * @param encodersystem The subsystem used by this command.
   */
  public EncoderMap(EncoderSystem encoderSystem, DriveTrain driveTrain) {
    m_encoderSystem = encoderSystem;
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(encoderSystem);
    addRequirements(driveTrain);
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

    //robotcoord(getRobotX(), getRobotY());
  public void robotcoord(float robotX, float robotY) {
    RobotQX = robotX; //note: please fix.
    RobotQY = robotY;
  }

  public float getRobotX() {
    return encoderX;
  } 
  
  public float getRobotY() {
    return encoderY;
  }

  public boolean checkMovement(float motorMovement, float desiredDistance) {
    if (motorMovement >= desiredDistance) {
      return true;
    }
    else { return false; }
  }

  //changeMovement(checkMovement(ENCODERDATA, desiredDistance))?
  // or *** if (checkMovement) {  changeMovement();  }
  public void changeMovement() {

  }
  public void updateMovement(float motorMovement) {

  }

  public float getRobotAngle() {
    return RobotQAngle;
  }
  

  // 1. turn right until distance = certain length
  public void turnRight() {
    m_driveTrain.rotateRight();
    
  }
  
  public void turnLeft() {
    m_driveTrain.rotateLeft();
  }
  public void changeRobotAngle(int desiredAngle) {

  }
}
