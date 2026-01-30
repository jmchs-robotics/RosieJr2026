package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Oculus extends SubsystemBase {

  private final QuestNav questNav;
  private final Drive m_drive;
  private Pose3d questPose;

  public Oculus(Drive drive) {

    questNav = new QuestNav();
    m_drive = drive;
    questPose = new Pose3d(drive.getPose()).transformBy(OculusConstants.ROBOT_TO_QUEST);
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrame;
    poseFrame = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : poseFrame) {

      Pose3d robotPose = questPose.transformBy(OculusConstants.ROBOT_TO_QUEST.inverse());
      questNav.setPose(questPose);

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
}
