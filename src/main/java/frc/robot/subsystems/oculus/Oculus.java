package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Oculus extends SubsystemBase {

  private final QuestNav questNav;
  private final Drive m_drive;
  private Pose3d questPose;
  private final Supplier<Pose2d> drivePose;

  public Oculus(Drive drive) {

    questNav = new QuestNav();
    m_drive = drive;
    drivePose = () -> drive.getPose();
    resetPose();

    // questNav.setPose(new Pose3d(0.058, 4.034, 0, new Rotation3d(Math.PI / 2, 0, Math.PI)));
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrame;
    poseFrame = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : poseFrame) {

      questPose = questFrame.questPose3d();
      Pose3d robotPose = questPose.transformBy(OculusConstants.ROBOT_TO_QUEST.inverse());

      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {

        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        m_drive.addVisionMeasurement(
            robotPose.toPose2d(), timestamp, OculusConstants.QUESTNAV_STD_DEVS);
      }
    }
  }

  public void resetPose() {
    questPose = new Pose3d(drivePose.get()).transformBy(OculusConstants.ROBOT_TO_QUEST);
    Logger.recordOutput("questPose", questPose);
    questNav.setPose(questPose);
  }

  public void setPose(Pose3d pose) {
    questNav.setPose(pose);
  }
}
