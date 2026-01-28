package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Oculus extends SubsystemBase {

  private final QuestNav questNav;

  public Oculus() {

    questNav = new QuestNav();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrame;
    poseFrame = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : poseFrame) {

      if (poseFrame.length > 0) {

        Pose3d questPose = poseFrame[poseFrame.length - 1].questPose3d();
        Pose3d robotPose = questPose.transformBy(OculusConstants.ROBOT_TO_QUEST.inverse());
        questNav.setPose(questPose);

        // Make sure the Quest was tracking the pose for this frame
        if (questFrame.isTracking()) {

          // Get timestamp for when the data was sent
          double timestamp = questFrame.dataTimestamp();

          // You can put some sort of filtering here if you would like!

          // Add the measurement to our estimator
          // TODO combine this demo drive thing with our actual drivetrain stuff
          // DemoDrive.addVisionMeasurement(robotPose.toPose2d(), timestamp,
          // OculusConstants.QUESTNAV_STD_DEVS);
        }
      }
    }
  }
}
