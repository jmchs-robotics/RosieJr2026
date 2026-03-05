// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.oculus.Oculus;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Vision vision;
  private final Drive drive;
  private final Intake intake;
  private final Hopper hopper;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final Shooter shooter;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Oculus oculus;

  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(bulldogCam1, robotToCamera1),
                new VisionIOPhotonVision(bulldogCam2, robotToCamera2));
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
    intake = new Intake(new IntakeIOMotors());

    hopper = new Hopper(new HopperIOMotor());
    oculus = new Oculus(drive);
    shooter = new Shooter(new ShooterIOTalonFX(), drive);

    // Set up auto routines

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // TODO add the actual commands when merged
    NamedCommands.registerCommand("intake", Commands.none());
    NamedCommands.registerCommand("shoot", Commands.none());

    // Set up SysId routines

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Configure the button bindings

    shooter = new Shooter(new ShooterIOTalonFX(), drive);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Joystick drive command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .start()
        .onTrue(
            new InstantCommand(
                    () -> {
                      drive.zeroHeading();
                    })
                .ignoringDisable(true));

    // Auto aim command example
    @SuppressWarnings("resource")
    PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               DriveCommands.autoAim(
    //                   drive, () -> aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             drive));

    driveController
        .rightStick()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.kZero));

    driveController.rightStick().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driveController.y().whileTrue(Commands.parallel(new IntakeRun(intake), new HopperRun(hopper)));

    operatorController.rightBumper().whileTrue(new HopperRun(hopper));

    operatorController.povDown().onTrue(new SlapDown(intake));

    operatorController.povUp().onTrue(new IntakeUp(intake));

    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    //driveController
    //    .y()
    //    .onTrue(
    //        new InstantCommand(() -> drive.setPose(new Pose2d(1.582, 4.034, new Rotation2d(0)))));

    driveController.povRight().whileTrue(new DriveToPose(drive, driveController));

    // driveController
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));
    // Shooter button binding
    driveController.rightTrigger().whileTrue(new ShooterRun(shooter));
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
}
