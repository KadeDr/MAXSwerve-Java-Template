// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.PrintStream;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  // private Camera m_robotcCamera;
  private RobotContainer m_robotContainer;
  private final PS5Controller m_armController = new PS5Controller(OIConstants.kArmControllerPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // m_robotcCamera = new Camera();
    // m_robotcCamera.InitializeCamera();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_robotcCamera.CameraRobotPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean l1 = m_armController.getL1Button();
    boolean r1 = m_armController.getR1Button();
    boolean l2 = m_armController.getL2Button();
    boolean r2 = m_armController.getR2Button();
    boolean triangle = m_armController.getTriangleButton();
    boolean cross = m_armController.getCrossButton();
    boolean circle = m_armController.getCircleButton();
    int pov = m_armController.getPOV();

    double leftX = -m_robotContainer.m_driverController.getLeftX();
    double leftY = -m_robotContainer.m_driverController.getLeftY();
    double rightX = -m_robotContainer.m_driverController.getRightX();
    boolean a = m_robotContainer.m_driverController.getAButton();

    // m_robotcCamera.CameraTeleopPeriodic(leftY, leftX, rightX, a, m_robotContainer);

    if (l1) {
      m_robotContainer.SetAlgaeState(0.25);
    }
    else if (pov == 270) {
      m_robotContainer.SetAlgaeState(-0.25);
    }
    else {
      m_robotContainer.SetAlgaeState(0);
    }

    if (r1) {
      m_robotContainer.SetElevatorState(0.3);
    }
    else if (pov == 90) {
      m_robotContainer.SetElevatorState(-0.3);
    }
    else {
      m_robotContainer.SetElevatorState(0);
    }

    if (triangle) {
      m_robotContainer.SetElevatorSpin(0.1);
    }
    else if (cross) {
      m_robotContainer.SetElevatorSpin(-0.1);
    }
    else {
      m_robotContainer.SetElevatorSpin(0);
    }

    if (pov == 0) {
      m_robotContainer.SetAlgaeSpin(-1);
    }
    else if (pov == 180) {
      m_robotContainer.SetAlgaeSpin(1);
    }
    else {
      m_robotContainer.SetAlgaeSpin(0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
