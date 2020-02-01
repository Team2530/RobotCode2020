/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pixy;
import frc.robot.subsystems.Shooter;

public class LocateBall extends CommandBase {
  /**
   * Creates a new LocateBall.
   */
  private DriveTrain m_driveTrain;
  private Pixy m_pixy;
  private Shooter m_shooter;
  public LocateBall(Pixy pixy, DriveTrain driveTrain, Shooter shooter) {
    m_pixy = pixy;
    m_driveTrain = driveTrain;
    m_shooter = shooter;

    addRequirements(m_pixy);
    addRequirements(m_driveTrain);
    addRequirements(m_shooter);
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
  
/*
   ballWidth = loadedBlock.getWidth();
  ballHeight = loadedBlock.getHeight();
  ballX = loadedBlock.getX();
  ballY = loadedBlock.getY();
  ballAngle = loadedBlock.getAngle();
  ballAge = loadedBlock.getAge();
*/



  public void driveToBall() {
    if (checkBall() == 1) {
      m_driveTrain.setMotorPower(Constants.DriveMotors.FR, 0.5);
      while (checkBall() == 1) {  } //does nothing
      m_driveTrain.setMotorPower(Constants.DriveMotors.FL, 0);  
    }
    else if (checkBall() == 2) {
      m_driveTrain.setMotorPower(Constants.DriveMotors.FL, 0.5);
      while (checkBall() == 2) {  } //that's the point  
      m_driveTrain.setMotorPower(Constants.DriveMotors.FL, 0);
    }
    
   else {
    return;
   }
  }

   public int checkBall() {
    if (m_pixy.getBallX()*2 <= 295) {
      return 1;
    }
    else if (m_pixy.getBallX()*2 >= 335) {
      return 2;
    }
    else if (295 < m_pixy.getBallX()*2 && m_pixy.getBallX()*2 < 335) {
      return 3;
    }
   else {
    return 0;
   }

  }







}
