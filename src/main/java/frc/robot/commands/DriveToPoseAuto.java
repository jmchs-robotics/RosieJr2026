package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class DriveToPoseAuto extends Command {

  private static boolean isFlipped = false;

  private final double threshold = 0.5;

  private final Drive drive;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.6, 0.0, 0.05, new Constraints(8.0, 20.0));

  public DriveToPoseAuto(Drive drive) {
    this.drive = drive;
    if (drive != null) {
      addRequirements(drive);
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    thetaController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {

    Pose2d currentPose = drive.getPose();

    double thetaVelocity;
    Rotation2d currentToHubAngle;
    if (!isFlipped) {

      currentToHubAngle =
          Constants.blueHub.getTranslation().minus(currentPose.getTranslation()).getAngle();

      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(), currentToHubAngle.getRadians());
    } else {
      currentToHubAngle =
          Constants.redHub.getTranslation().minus(currentPose.getTranslation()).getAngle();

      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(), currentToHubAngle.getRadians());
    }

    if (drive != null
        && Math.abs(drive.getRotation().getDegrees() - currentToHubAngle.getDegrees())
            > threshold) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0, 0, thetaVelocity * drive.getMaxAngularSpeedRadPerSec(), drive.getRotation()));
    }
  }
}
