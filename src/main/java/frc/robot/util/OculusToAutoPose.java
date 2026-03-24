package frc.robot.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.oculus.Oculus;
import frc.robot.subsystems.oculus.OculusConstants;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class OculusToAutoPose implements Consumer<Command> {

  private final Supplier<String> selectedSupplier;
  private final Drive drive;
  private final Oculus oculus;

  public OculusToAutoPose(Supplier<String> selectedSupplier, Drive drive, Oculus oculus) {

    this.selectedSupplier = selectedSupplier;
    this.drive = drive;
    this.oculus = oculus;
  }

  @Override
  public void accept(Command t) {

    try {

      List<PathPlannerPath> pathsInAuto =
          PathPlannerAuto.getPathGroupFromAutoFile(selectedSupplier.get());
      PathPlannerPath startingPath = pathsInAuto.get(0);
      Pose2d startingPose =
          startingPath
              .getStartingHolonomicPose()
              .orElse(startingPath.getStartingDifferentialPose());

      boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

      drive.setPose(
          isFlipped
              ? startingPose.rotateAround(Constants.centerField, Rotation2d.k180deg)
              : startingPose);

      oculus.setPose(
          isFlipped
              ? new Pose3d(startingPose.rotateAround(Constants.centerField, Rotation2d.k180deg))
                  .transformBy(OculusConstants.ROBOT_TO_QUEST)
              : new Pose3d(startingPose).transformBy(OculusConstants.ROBOT_TO_QUEST));

    } catch (Exception e) {
      DriverStation.reportWarning("Failed to load selected auto path", null);
    }
  }
}
