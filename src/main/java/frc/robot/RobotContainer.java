// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
<<<<<<< HEAD
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.limelight.AlignCommand;
=======
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
>>>>>>> df22047ee764779ca0535e960d0a4cff0f2ada11
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;
=======
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
>>>>>>> df22047ee764779ca0535e960d0a4cff0f2ada11

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
<<<<<<< HEAD
public class RobotContainer extends SubsystemBase {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        // Controller
        private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
=======
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsytem m_robotElevator = new ElevatorSubsytem();
  private final AlgaeSubsystem m_robotAlgae = new AlgaeSubsystem();

  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);

  // Autonomous Vars
  private final SendableChooser<Command> autoChooser;

  public void Arm() {
    boolean l1 = m_armController.getLeftBumperButton();
    boolean r1 = m_armController.getRightBumperButton();
    boolean triangle = m_armController.getYButton();
    boolean cross = m_armController.getAButton();
    int pov = m_armController.getPOV();

    // m_robotcCamera.CameraTeleopPeriodic(leftY, leftX, rightX, a, m_robotContainer);

    if (l1) {
      SetAlgaeState(0.25);
    }
    else if (pov == 270) {
      SetAlgaeState(-0.25);
    }
    else {
      SetAlgaeState(0);
    }

    if (r1) {
      SetElevatorState(0.3);
    }
    else if (pov == 90) {
      SetElevatorState(-0.3);
    }
    else {
      SetElevatorState(0);
    }

    if (triangle) {
      SetElevatorSpin(0.1);
    }
    else if (cross) {
      SetElevatorSpin(-0.1);
    }
    else {
      SetElevatorSpin(0);
    }

    if (pov == 0) {
      SetAlgaeSpin(-1);
    }
    else if (pov == 180) {
      SetAlgaeSpin(1);
    }
    else {
      SetAlgaeSpin(0);
    }
  }
>>>>>>> df22047ee764779ca0535e960d0a4cff0f2ada11

        // Slew rate limiters for smooth joystick input (same as your original
        // Robot.java)
        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

<<<<<<< HEAD
        // Accurate period calculation using Timer
        private double m_lastPeriodUpdate = 0.0;
        private double m_currentPeriod = 0.02;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureButtonBindings();
                m_robotDrive.zeroHeading();

                // Default command: manual field-relative swerve drive with slew limiting and
                // real period
                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> {
                                        updatePeriod();
                                        // Get TV (Exists) and TX (Angle)
                                        boolean hasTarget = LimelightHelpers.getTV("limelight-main");
                                        double tx = LimelightHelpers.getTX("limelight-main");

                                        // Rumble ONLY if we see target AND are aimed within 3 degrees
                                        if (hasTarget && Math.abs(tx) < 3.0) {
                                                // Check for CommandXboxController vs XboxController here
                                                m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
                                        } else {
                                                m_driverController.setRumble(RumbleType.kBothRumble, 0);
                                        }
                                        // Apply deadband and slew limiting
                                        double xInput = MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband);
                                        double yInput = MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband);
                                        double rotInput = MathUtil.applyDeadband(m_driverController.getRightX(),
                                                        OIConstants.kDriveDeadband);

                                        double xSpeed = -m_xspeedLimiter.calculate(xInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double ySpeed = -m_yspeedLimiter.calculate(yInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double rot = -m_rotLimiter.calculate(rotInput)
                                                        * DriveConstants.kMaxAngularSpeed;

                                        m_robotDrive.drive(xSpeed, ySpeed, rot, true, m_currentPeriod);
                                }, m_robotDrive));
        }

        private void configureButtonBindings() {
                // R1 (right bumper) → X formation (lock wheels)
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(new AlignCommand(1, m_driverController, "limelight-main", 3, m_robotDrive));

                // A button → Limelight auto-align and range (overrides forward and rotation,
                // // keeps strafing)
                // new JoystickButton(m_driverController, XboxController.Button.kA.value)
                // .whileTrue(AlignCommand());
        }

        // Helper to compute real loop period
        private void updatePeriod() {
                double now = Timer.getFPGATimestamp();
                m_currentPeriod = (m_lastPeriodUpdate == 0.0) ? 0.02 : (now - m_lastPeriodUpdate);
                m_lastPeriodUpdate = now;
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("LimelightTX", LimelightHelpers.getTX("limelight"));
                SmartDashboard.putBoolean("LimelightHasTarget", LimelightHelpers.getTV("limelight"));
        }

        // private Command limelightDriveWithAimAssist() {
        // return new RunCommand(() -> {
        // LimelightHelpers.setPriorityTagID("limelight-main", 1);
        // double xInput = MathUtil.applyDeadband(m_driverController.getLeftX(),
        // OIConstants.kDriveDeadband);
        // double yInput = MathUtil.applyDeadband(m_driverController.getLeftY(),
        // OIConstants.kDriveDeadband);

        // double xSpeed = -m_xspeedLimiter.calculate(xInput)
        // * DriveConstants.kMaxSpeedMetersPerSecond;
        // double ySpeed = -m_yspeedLimiter.calculate(yInput)
        // * DriveConstants.kMaxSpeedMetersPerSecond;

        // double tX = LimelightHelpers.getTX("limelight-main");
        // boolean hasTarget = LimelightHelpers.getTV("limelight-main");

        // double rotSpeed = 0.0;

        // if (hasTarget) {
        // double kP_Rot = 0.01;
        // rotSpeed = -tX * kP_Rot;
        // } else {
        // double rotInput = MathUtil.applyDeadband(m_driverController.getRightX(),
        // OIConstants.kDriveDeadband);

        // rotSpeed = -m_rotLimiter.calculate(rotInput) *
        // DriveConstants.kMaxAngularSpeed;
        // }

        // if (hasTarget && Math.abs(tX) < 3.0) {
        // // Check for CommandXboxController vs XboxController here
        // m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
        // } else {
        // m_driverController.setRumble(RumbleType.kBothRumble, 0);
        // }

        // m_robotDrive.drive(xSpeed, ySpeed, rotSpeed, true, 0);
        // }, m_robotDrive);
        // }

        // private Command limelightDriveToPoseCommand() {
        // return new RunCommand(() -> {

        // // 1. Check if Target is Valid
        // boolean hasTarget = LimelightHelpers.getTV("limelight-main");
        // LimelightHelpers.setPriorityTagID("limelight-main", 5);

        // if (!hasTarget) {
        // System.out.println("No Target!");
        // m_robotDrive.drive(0, 0, 0, false, 0);
        // return;
        // }

        // // 2. Get Simple 2D Data
        // double tx = LimelightHelpers.getTX("limelight-main");
        // double ta = LimelightHelpers.getTA("limelight-main");
        // double currentTagID = LimelightHelpers.getFiducialID("limelight-main");

        // if (currentTagID != 5) {
        // System.out.println("Correct tag not found!");
        // return;
        // }

        // // 3. Get Gyro Data (NEW)
        // // This assumes 0 degrees is "Straight Forward" towards the tag.
        // // If your tags are at 180 degrees (opposite side), change targetHeading to
        // 180.
        // double currentHeading = m_robotDrive.getHeading();
        // double targetHeading = 0.0;

        // // 4. Tuning
        // double desiredArea = 3.0;

        // double kP_Distance = 0.05;
        // double kP_Strafe = 0.01; // Increased slightly since it no longer fights
        // rotation
        // double kP_Gyro = 0.01; // NEW: Speed per degree of gyro error
        // double maxSpeed = 0.4;
        // double maxRotationSpeed = 0.2;

        // // 5. Calculate Speeds

        // // Forward Speed (Distance Control)
        // double driveError = desiredArea - ta;
        // double forwardSpeed = driveError * kP_Distance;

        // // Rotation Speed (GYRO LOCK)
        // // We calculate the error between where we want to face (0) and where we are.
        // // Math.IEEEremainder ensures we don't spin 360 degrees unnecessarily.
        // double headingError = Math.IEEEremainder(targetHeading - currentHeading,
        // 360);
        // double rotationSpeed = headingError * kP_Gyro;

        // // Strafe Speed (Slide to center)
        // // Now that Gyro keeps us straight, 'tx' is purely lateral offset!
        // double strafeSpeed = -tx * kP_Strafe;

        // // 6. Clamp & Drive
        // forwardSpeed = MathUtil.clamp(forwardSpeed, -maxRotationSpeed,
        // maxRotationSpeed);
        // rotationSpeed = MathUtil.clamp(rotationSpeed, -maxSpeed, maxSpeed);
        // strafeSpeed = MathUtil.clamp(strafeSpeed, -maxSpeed, maxSpeed);

        // SmartDashboard.putNumber("Debug/ForwardSpeed", forwardSpeed);
        // SmartDashboard.putNumber("Debug/StrafeSpeed", strafeSpeed);
        // SmartDashboard.putNumber("Debug/GyroError", headingError);

        // if (hasTarget && Math.abs(tx) < 3.0) {
        // // Check for CommandXboxController vs XboxController here
        // m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
        // } else {
        // m_driverController.setRumble(RumbleType.kBothRumble, 0);
        // }

        // // Drive Robot Relative (fieldRelative = false)
        // m_robotDrive.drive(forwardSpeed, strafeSpeed, rotationSpeed, false, 0);

        // }, m_robotDrive);
        // }

        // // Limelight proportional control command
        // private Command limelightAlignCommand() { // Aligns the robot to the offset
        // of the april tag the coder provides
        // return new RunCommand(() -> {
        // updatePeriod();
        // LimelightHelpers.setPriorityTagID("limelight-main", 1);

        // // 1. Check if the Limelight actually sees a target
        // boolean hasTarget = LimelightHelpers.getTV("limelight-main");
        // if (!hasTarget) {
        // m_robotDrive.drive(0, 0, 0, false, m_currentPeriod);
        // return;
        // }

        // // 2. Get Errors
        // double tx = LimelightHelpers.getTX("limelight-main"); // Horizontal error
        // double ty = LimelightHelpers.getTY("limelight-main"); // Vertical error
        // (Distance)

        // // --- CONFIGURATION ---
        // // The angle (ty) you want to reach.
        // // STEP: Drive robot to perfect shooting range, read 'ty' from dashboard, put
        // // here.
        // double desiredTargetY = 0.0;

        // double kP_Rot = 0.0015; // Tuning for Rotation
        // double kP_Strafe = 0.0015; // Tuning for Strafe
        // double kP_Range = 0.01; // NEW: Tuning for Forward/Back speed

        // // 3. Calculate Raw Outputs
        // double rotationOutput = tx * kP_Rot;
        // double strafeOutput = tx * kP_Strafe;
        // // NEW: If ty < desired, we are too far (drive forward/positive).
        // double rangeOutput = (desiredTargetY - ty) * kP_Range;

        // // 4. Apply Deadbands
        // // Stop rotation/strafe if aligned within 1.5 degrees
        // if (Math.abs(tx) < 1.5) {
        // rotationOutput = 0;
        // strafeOutput = 0;
        // }
        // // NEW: Stop ranging if distance is within 0.5 degrees of target
        // if (Math.abs(desiredTargetY - ty) < 0.5) {
        // rangeOutput = 0;
        // }

        // // 5. Safety Clamps
        // // Clamp all outputs to 10% speed for safety while testing
        // rotationOutput = MathUtil.clamp(rotationOutput, -0.1, 0.1);
        // strafeOutput = MathUtil.clamp(strafeOutput, -0.1, 0.1);
        // rangeOutput = MathUtil.clamp(rangeOutput, -0.1, 0.1); // NEW

        // if (hasTarget && Math.abs(tx) < 3.0) {
        // // Check for CommandXboxController vs XboxController here
        // m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
        // } else {
        // m_driverController.setRumble(RumbleType.kBothRumble, 0);
        // }

        // // 6. Execute Drive
        // // NOTE: We assume +X is Forward, +Y is Left (NWU standard)
        // // If Limelight is front-mounted:
        // // Forward Speed = rangeOutput
        // // Strafe Speed = -strafeOutput (Move Right to fix +TX error)
        // m_robotDrive.drive(
        // rangeOutput * DriveConstants.kMaxSpeedMetersPerSecond, // NEW: Drive
        // // Forward/Back
        // // 0,
        // // 0,
        // -strafeOutput * DriveConstants.kMaxSpeedMetersPerSecond,
        // -rotationOutput * DriveConstants.kMaxAngularSpeed,
        // false,
        // m_currentPeriod);

        // }, m_robotDrive);
        // }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Trajectory config
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(DriveConstants.kDriveKinematics);

                // Trajectory 1
                Trajectory forwardRotation = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                                new Translation2d(Units.feetToMeters(0.3), 0),
                                                new Translation2d(Units.feetToMeters(0.6), 0)),
                                new Pose2d(Units.feetToMeters(1), 0, new Rotation2d(180)),
                                config);

                var thetaController1 = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0.5,
                                AutoConstants.kThetaControllerConstraints);
                thetaController1.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
                                forwardRotation,
                                m_robotDrive::getPose,
                                DriveConstants.kDriveKinematics,
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController1,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                return new SequentialCommandGroup(
                                // Reset odometry at start
                                new InstantCommand(() -> m_robotDrive.resetOdometry(forwardRotation.getInitialPose()),
                                                m_robotDrive),

                                // Run trajectory 1
                                swerveControllerCommand1,

                                // Lock wheels in X formation at end
                                new InstantCommand(() -> m_robotDrive.setModuleStates(new SwerveModuleState[] {
                                                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                                                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                                                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                                                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                                }), m_robotDrive));

                // Your commented-out longer auto is preserved below for reference
                /*
                 * return new SequentialCommandGroup(
                 * // Reset odometry at start
                 * new InstantCommand(() ->
                 * m_robotDrive.resetOdometry(forwardRotation.getInitialPose()), m_robotDrive),
                 * 
                 * // Trajectory 1
                 * swerveControllerCommand1,
                 * 
                 * // Immediately zero drive after traj1
                 * new InstantCommand(() -> m_robotDrive.setModuleStates(new SwerveModuleState[]
                 * {
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                 * }), m_robotDrive));
                 */
        }
}
=======
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

    new EventTrigger("Raise").onTrue(m_robotElevator.MoveElevatorCommand(-0.5));
    new EventTrigger("Stop").onTrue(m_robotElevator.MoveElevatorCommand(0));
    new EventTrigger("Lower").onTrue(m_robotElevator.MoveElevatorCommand(0.5));
    new EventTrigger("Outtake").onTrue(m_robotElevator.SpinElevatorCommand(0.5));
    new EventTrigger("Stop Spin").onTrue(m_robotElevator.SpinElevatorCommand(0));

    // new EventTrigger("Raise").whileTrue(new InstantCommand(() -> m_robotElevator.MoveElevator(-0.5)));
    // new EventTrigger("Stop").whileTrue(new InstantCommand(() -> m_robotElevator.MoveElevator(0)));
    // new EventTrigger("Lower").whileTrue(new InstantCommand(() -> m_robotElevator.MoveElevator(0.5)));
    // new EventTrigger("Outtake").whileTrue(new InstantCommand(() -> m_robotElevator.SpinElevator(0.1)));
    // new EventTrigger("Stop Spin").whileTrue(new InstantCommand(() -> m_robotElevator.SpinElevator(0)));

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
>>>>>>> df22047ee764779ca0535e960d0a4cff0f2ada11
