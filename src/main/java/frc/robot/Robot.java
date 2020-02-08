/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.DualLargeJoystickDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;
  private RobotContainer m_robotContainer;
  private SerialPort arduino;
  private Timer timer;
  byte[] outboxCompare = {0};

  // This variable will automatically be pushed to the Arduino when its value changes.
  // VALUE KEY:
  // 00 = nothing
  // 10 = Pixy sees a ball
  // 20 = Limelight sees a target
  // 12 = Pixy and Limelight
  public static byte[] arduinoOutbox = {00};

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Establish connection with Arduino
    try {
      arduino = new SerialPort (9600, SerialPort.Port.kUSB);
      System.out.println("Successfully connected to Arduino!");
    } catch (Exception e) {
      System.out.println("Encountered a problem connecting via kUSB. Trying kUSB 1...");

      try {
        arduino = new SerialPort (9600, SerialPort.Port.kUSB1);
        System.out.println("Successfully connected to Arduino!");
      } catch (Exception e1) {
        System.out.println("Encountered a problem connecting via kUSB1. Trying kUSB 2...");

        try {
          arduino = new SerialPort (9600, SerialPort.Port.kUSB2);
          System.out.println("Successfully connected to Arduino!");
        } catch (Exception e2) {
          System.out.println("Encountered a problem connecting via kUSB2. No connections left to test.");
        }
      }
    }

    timer = new Timer();
    timer.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Checks if data needs to be pushed to Arduino, then does it
    if (outboxCompare != arduinoOutbox) {
      if (timer.get() > 0.5  /* (in seconds) cooldown period, to prevent data spam */) {
        outboxCompare = arduinoOutbox;
        arduino.write(outboxCompare, 1);
        System.out.println("Sending " + outboxCompare + " to Arduino...");
        timer.reset();
      }
    }

    if (arduino.getBytesReceived() > 0) {
      System.out.println(arduino.readString());
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_teleopCommand = m_robotContainer.getTelopCommand();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    if (m_teleopCommand != null) {
      m_teleopCommand.schedule();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //? pit init here not command that could accendently be pressed durring match
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
