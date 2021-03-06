/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pixy;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
/*import frc.robot.commands.LocateBall;*/ //commented out because it is not being used yet. may or may not be used. Here anyway.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WoFSpinner extends CommandBase {
  /**
   * Creates a new WoFSpinner.
   */
  static Shooter shooter;
  private static WPI_TalonSRX motor_Left_FlyWheel = new WPI_TalonSRX(Constants.motor_Left_FlyWheel_Port);
  private static WPI_TalonSRX motor_Right_FlyWheel = new WPI_TalonSRX(Constants.motor_Right_Flywheel_Port);

  // private static Encoder encoder_Left = new
  // Encoder(Constants.encoder_Left_Flywheel_Ports[0],Constants.encoder_Left_Flywheel_Ports[1]);
  // private static Encoder encoder_Right = new
  // Encoder(Constants.encoder_Right_Flywheel_Port[0],Constants.encoder_Right_Flywheel_Port[1]);

  public void Shooter() {
    motor_Right_FlyWheel.setInverted(false);
  }
  // motor_Right_FlyWheel.set(ControlMode.Follower,
  // Constants.motor_Left_FlyWheel_Port);

  public void WoFSpinner(Shooter shooter, Pixy pixy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    DriveTrain m_driveTrain; // this variable may not be initialized correctly. Fix it
    /* final Pixy m_pixy; */ // commented out because it is not in use yet. It will be though
    addRequirements(shooter);
    addRequirements(pixy);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  public WoFSpinner() {
    double displaySpeedRads = shooter.getAvgSpeed();
    double displaySpeedRPM = displaySpeedRads * 9.549296585513702;
    SmartDashboard.putString("The Current Shooter Speed is: " + displaySpeedRPM + "", "shooter-speed");
  }

  public static void spin(String color) {
      // BELOW CODE NEEDS TO BE FINISHED
    /* switch(color){
        case "red":
          shooter.WoFspin();
          while(){}
          shooter.stopFW();
        case "yellow":
          shooter.WoFspin();
          while(){}
          shooter.stopFW();
        case "green":
          shooter.WoFspin();
          while(){}
          shooter.stopFW();
        case "blue":
          shooter.WoFspin();
          while(){}
          shooter.stopFW();
      } */
    /*while (1 == 1) {
      // run motors at 0.1 speed
      DriveTrain m_driveTrain;
      m_driveTrain.setSingleMotorPower(DriveMotors.FR, 0);
      m_driveTrain.setSingleMotorPower(DriveMotors.FL, 0);
      }*/
      //I thought this looked useless
  // Called once the command ends or is interrupted.

  // Returns true when the command should end.
}
}
