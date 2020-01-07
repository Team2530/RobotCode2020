/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj.interfaces.Gyro;

public class AutoAlign extends Command {
  NetworkTable table;
  boolean lineFound;
  boolean hadLine = false;
  double r1;
  double t1;
  public AutoAlign() {
    // Use requires() here to declare subsystem dependencies
     requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    table = NetworkTableInstance.getDefault().getTable("datatable");
    SmartDashboard.putNumber("leftpow",table.getEntry("leftpow").getDouble(15));
    SmartDashboard.putNumber("rightpow",table.getEntry("rightpow").getDouble(15));
    lineFound = Boolean.parseBoolean(table.getEntry("lineFound").getString("false"));
    hadLine = false;
    //Robot.driveTrain.resetGryo();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    r1 = table.getEntry("r1").getDouble(-1);
    t1 = table.getEntry("t1").getDouble(-1);
    lineFound = Boolean.parseBoolean(table.getEntry("lineFound").getString("false"));
    if(lineFound){
      Robot.driveTrain.setMotorPower(0,0.2);
      Robot.driveTrain.setMotorPower(1,0.2);
      Robot.driveTrain.setMotorPower(2,0.2);
      Robot.driveTrain.setMotorPower(3,0.2);
      hadLine = true;
    }else{
      if(hadLine){
        //Robot.driveTrain.angleGyroTurn(t1, 0.3);
      }else{
        for (int i = 0; i < 3; i++) {
          Robot.driveTrain.setMotorPower(i, 0.2);
        }

      }
    }
    
    //Robot.driveTrain.SetDrivePower(table.getEntry("leftpow").getDouble(15), table.getEntry("rightpow").getDouble(15));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.Stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.Stop();
  }
}
