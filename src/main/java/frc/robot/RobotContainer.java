// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.oculus.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.OculusToAutoPose;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  @AutoLogOutput(key = "Drivers/AddieDriving")
  private boolean addieBoolean;

  @AutoLogOutput(key = "Drivers/OwenDriving")
  private boolean owenBoolean;

  @AutoLogOutput(key = "Drivers/DriveFlipped")
  private boolean redFlip;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final Drive drive;
  private final Oculus oculus;
  private final Vision vision;

  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    addieBoolean = true;
    owenBoolean = false;
    redFlip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    switch (Constants.currentMode) {
      case REAL:

        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});

        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(bulldogCam1, robotToCamera1));
        // new VisionIOPhotonVision(bulldogCam2, robotToCamera2));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision = new Vision(drive::addVisionMeasurement);
        // new VisionIOPhotonVisionSim(
        //     bulldogCam1, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose),
        // new VisionIOPhotonVisionSim(
        //     bulldogCam2, robotToCamera2, driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:

        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // drive.setPose(new Pose2d(1.582, 4.034, new Rotation2d(0)));
    oculus = new Oculus(drive);

    // Set up auto routines

    AutoBuilder.configure(
        drive::getPose,
        (pose) -> {
          oculus.setPose(new Pose3d(pose).transformBy(OculusConstants.ROBOT_TO_QUEST));
          drive.setPose(pose);
        },
        drive::getChassisSpeeds,
        (speeds, feedForward) -> drive.runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // TODO us in general need to add auto commands
    // NamedCommands.registerCommand(
    //     "intake",
    //     Commands.sequence(new SlapDown(intake).withTimeout(1), new IntakeFullSpeed(intake)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "BLUE initailize to point",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(3.638, 0.496, Rotation2d.kZero));
              oculus.resetPose();
            }));
    autoChooser.addOption(
        "RED initailize to point",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(12.794, 7.641, Rotation2d.k180deg));
              oculus.resetPose();
            }));
    autoChooser.onChange(
        new OculusToAutoPose(() -> autoChooser.getSendableChooser().getSelected(), drive, oculus));

    // var teleopTab = Shuffleboard.getTab("teleop");

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .start()
        .and(() -> addieBoolean)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              redFlip ? Rotation2d.k180deg : Rotation2d.kZero));
                      oculus.resetPose();
                    },
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public void updateElastic() {}
}
