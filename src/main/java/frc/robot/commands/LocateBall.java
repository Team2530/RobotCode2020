/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveMotors;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pixy;
import frc.robot.subsystems.Shooter;

public class LocateBall extends CommandBase {
  /**
   * Creates a new LocateBall.
   */


  private int tolerance = 40;
  private DriveTrain m_driveTrain;
  private Pixy m_pixy;
  private Shooter m_shooter;
  public LocateBall(DriveTrain driveTrain, Pixy pixy, Shooter shooter) {
    m_pixy = pixy;
    m_driveTrain = driveTrain;
    m_shooter = shooter;

    addRequirements(pixy);
    addRequirements(driveTrain);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveToBall();
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



  //max X = 315;  630;   //error tolerance (tol) = 40; kP = 2?; //ajustXError(constant (increase over time?), tolerance);
  public void ajustXError(double kP, int tol) {
    float Rtol = 315 + tol;
    float Ltol = 315 - tol; 
    double Rerr = m_pixy.getX()*2 - Rtol;
    double Lerr = Ltol - m_pixy.getX()*2;

    if (Ltol > (m_pixy.getX()*2)) {
      m_driveTrain.setSingleMotorPower(DriveMotors.FR, kP * Lerr);
      //m_driveTrain.setSingleMotorPower(DriveMotors.FL, 0.3);  // to stabilize movement
    }
    else if ((m_pixy.getX()*2) > Rtol) {
      m_driveTrain.setSingleMotorPower(DriveMotors.FL, kP * Rerr);
      //m_driveTrain.setSingleMotorPower(DriveMotors.FR, 0.3);  // to stabilize movement
    }

  }
  double pixyDeadZone = 9.5; //area where it thinks ball is close enough to straigh ahead. radius
  public void driveToBall() {
    if (m_pixy.getBiggestBlock()==null){
      return;
    }
    if (m_pixy.getArea() < 900 && !(m_pixy.getBiggestBlock()==null)) {
      if (m_pixy.getX() < 157.5 - pixyDeadZone){//if ball is to far too the left
        while((m_pixy.getX() < 157.5 - pixyDeadZone)){//turn left
          m_driveTrain.turnLeft(0.5);
        }
      }
      if (m_pixy.getX() > 157.5 + pixyDeadZone){//if ball is too far too the right
        while((m_pixy.getX() > 157.5 + pixyDeadZone)){//turn right
          m_driveTrain.turnRight(0.5);
        }
      }
      while (m_pixy.getArea() < 900 && !(m_pixy.getBiggestBlock()==null)) {
        m_driveTrain.setSingleMotorPower(DriveMotors.FR, 1);
        m_driveTrain.setSingleMotorPower(DriveMotors.FL, 1);
      }
        m_driveTrain.setSingleMotorPower(DriveMotors.FR, 0);
        m_driveTrain.setSingleMotorPower(DriveMotors.FL, 0);
    } 
  }
      //gotoball
      //align X
      //ball far away? make sure ball doesn't go out of view? off screen ball tracking?
      //start motors -> drive
}
