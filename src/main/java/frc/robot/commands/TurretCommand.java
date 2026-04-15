package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Turret;

public class TurretCommand extends Command {

  private final Turret turret;
  private final Drive drive;
  private boolean redFlip;
  private Pose2d currentHubPose;

  public TurretCommand(Turret turret, Drive drive) {

    this.turret = turret;
    addRequirements(turret);

    this.drive = drive;
  }

  @Override
  public void initialize() {
    redFlip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    currentHubPose = redFlip ? Constants.redHub : Constants.blueHub;
  }

  @Override
  public void execute() {

    Pose2d currentRobotPose = drive.getPose();
    Rotation2d robotToHub =
        currentHubPose.getTranslation().minus(currentRobotPose.getTranslation()).getAngle();
    Rotation2d currentRobotRotation = currentRobotPose.getRotation();
    Rotation2d targetTurretDegrees =
        redFlip
            ? robotToHub.plus(currentRobotRotation).minus(Rotation2d.k180deg)
            : robotToHub.plus(currentRobotRotation);

    turret.setTurretAngle(MathUtil.clamp(targetTurretDegrees.getDegrees(), -135, 135));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turret.setOpenLoop(0);
  }
}
