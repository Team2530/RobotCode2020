/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.commands.autonomousCommands.*;
// import frc.robot.commands.SmallJoystickElevator;
// import frc.robot.commands.XboxJoystickElevator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Pixy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // -------------------- Subsystems -------------------- \\
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Elevator elevatorSub = new Elevator();
  private final LimeLight limeLightSub = new LimeLight();
  private final Pixy m_pixy = new Pixy();
  private final Shooter m_shooter = new Shooter();

  // -------------------- Joysticks and Buttons -------------------- \\
  //Joysticks
  final Joystick stick1 = new Joystick(1); // Creates a joystick on port 1
  final Joystick stick2 = new Joystick(2); // Creates a joystick on port 2

  //Joystick buttons
  private final JoystickButton Button1 = new JoystickButton(stick1, 1); // Creates a new button for button 1 on stick1
  private final JoystickButton Button3 = new JoystickButton(stick1, 3);
  private final JoystickButton Button4 = new JoystickButton(stick1, 4);
  private final JoystickButton Button5 = new JoystickButton(stick1, 5);

  //Xbox Controller
  final XboxController xbox = new XboxController(0);

  //Xbox buttons
  private final JoystickButton XboxButton1 = new JoystickButton(xbox, 1);

  // -------------------- Autonomous Commands -------------------- \\
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DelayTest delayCommand = new DelayTest(1, m_autoCommand);
  

  // -------------------- Telop Commands -------------------- \\
  // private final XboxJoystickElevator elevatorCommand = new XboxJoystickElevator(elevatorSub, xbox);
  // private final SmallJoystickElevator elevatorCommand = new SmallJoystickElevator(elevatorSub, stick1);
  // private final EncoderTest m_telopCommand = new EncoderTest(m_driveTrain);
  private final LineUp lineUp = new LineUp(m_driveTrain, limeLightSub, elevatorSub);
  private final TestPixy pixy = new TestPixy(m_pixy);
  private final ToggleLimeLightLED toggleLED = new ToggleLimeLightLED(limeLightSub);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button1.whileHeld(lineUp);
    Button1.whenReleased(new LargeJoystickDrive(m_driveTrain, stick1));
    Button3.whenPressed(toggleLED);
    Button5.whenPressed(new LocateBall(m_driveTrain, m_pixy, m_shooter));
    // Button4.whenPressed(new InstantCommand())

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getTelopCommand() {
    return new LargeJoystickDrive(m_driveTrain, stick1);
  }

}
