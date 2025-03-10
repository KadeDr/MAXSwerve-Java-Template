// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;
import java.util.TimerTask;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsytem m_robotElevator = new ElevatorSubsytem();
  private final AlgaeSubsystem m_robotAlgae = new AlgaeSubsystem();

  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_armController = new CommandXboxController(OIConstants.kArmControllerPort);

  // Autonomous Vars
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    new EventTrigger("Raise").onTrue(new InstantCommand(() -> m_robotElevator.MoveElevator(-0.5)));
    new EventTrigger("Stop").onTrue(new InstantCommand(() -> m_robotElevator.MoveElevator(0)));
    new EventTrigger("Lower").onTrue(new InstantCommand(() -> m_robotElevator.MoveElevator(0.5)));
    new EventTrigger("Outtake").onTrue(new InstantCommand(() -> m_robotElevator.SpinElevator(0.1)));
    new EventTrigger("Stop Spin").onTrue(new InstantCommand(() -> m_robotElevator.SpinElevator(0)));

    // new EventTrigger("Elevator").onTrue(Commands.sequence(
    //   Commands.parallel(
    //     new InstantCommand(() -> m_robotElevator.MoveElevator(-0.5)),
    //     Commands.waitSeconds(1.05)
    //   ).andThen(
    //     Commands.parallel(
    //       new InstantCommand(() -> m_robotElevator.MoveElevator(0)),
    //       new InstantCommand(() -> m_robotElevator.SpinElevator(0.1)),
    //       Commands.waitSeconds(0.5)
    //     )
    //   ).andThen(
    //     Commands.parallel(
    //       new InstantCommand(() -> m_robotElevator.MoveElevator(0.5)),
    //       new InstantCommand(() -> m_robotElevator.SpinElevator(0)),
    //       Commands.waitSeconds(1.05)
    //     )
    //   ).andThen(
    //     new InstantCommand(() -> m_robotElevator.MoveElevator(0))
    //   )
    // ));
    // new EventTrigger("Wait").onTrue(Commands.print("Waiting at Intake"));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // if (!aprilTag) {
    //   new RunCommand(
    //     () -> m_robotDrive.drive(
    //         -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //         -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //         -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //         true),
    //     m_robotDrive);
    // }

    // m_armController.L1().onTrue(m_robotElevator.MoveElevator(0));
    // m_armController.R1().onTrue(m_robotElevator.MoveElevator(5));
  }

  public void MoveRobot(double leftYDeadband, double leftXDeadband, double rightXDeadband) {
    m_robotDrive.drive(-MathUtil.applyDeadband(leftYDeadband, OIConstants.kDriveDeadband), -MathUtil.applyDeadband(leftXDeadband, OIConstants.kDriveDeadband), -MathUtil.applyDeadband(rightXDeadband, OIConstants.kDriveDeadband), true);
  }

  public void SetElevatorState(double speed) {
    m_robotElevator.MoveElevator(speed);
  }

  public void SetElevatorSpin(double speed) {
    m_robotElevator.SpinElevator(speed);
  }

  public void SetAlgaeState(double speed) {
    m_robotAlgae.SetRotateState(speed);
  }

  public void SetAlgaeSpin(double speed) {
    m_robotAlgae.SetSpinState(speed);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // Create config for trajectory
  //   TrajectoryConfig config = new TrajectoryConfig(
  //       AutoConstants.kMaxSpeedMetersPerSecond,
  //       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DriveConstants.kDriveKinematics);

  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(),
  //       // End 88 inches meters straight ahead of where we started, facing forward
  //       new Pose2d(2.2352, 0, new Rotation2d(0)),
  //       config);

  //   var thetaController = new ProfiledPIDController(
  //       AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //       exampleTrajectory,
  //       m_robotDrive::getPose, // Functional interface to feed supplier
  //       DriveConstants.kDriveKinematics,

  //       // Position controllers
  //       new PIDController(AutoConstants.kPXController, 0, 0),
  //       new PIDController(AutoConstants.kPYController, 0, 0),
  //       thetaController,
  //       m_robotDrive::setModuleStates,
  //       m_robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false))
  //     .andThen(
  //       Commands.parallel(
  //         Commands.waitSeconds(1.05),
  //         new InstantCommand(() -> m_robotElevator.MoveElevator(-0.5))
  //       )
  //     )
  //     .andThen(() -> m_robotElevator.MoveElevator(0))
  //     .andThen(() -> m_robotElevator.SpinElevator(0.1));
  // }
}
