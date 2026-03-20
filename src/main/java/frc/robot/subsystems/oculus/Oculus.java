package frc.robot.subsystems.oculus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
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
    List<Pose3d> observedPoses = new LinkedList<Pose3d>();
    List<Pose2d> acceptedPoses = new LinkedList<Pose2d>();
    List<Pose3d> oculusPoses = new LinkedList<>();

    if (poseFrame.length != 0) {
      var questFrame = poseFrame[poseFrame.length - 1];

      questPose = questFrame.questPose3d();
      oculusPoses.add(questPose);
      Pose3d robotPose = questPose.transformBy(OculusConstants.ROBOT_TO_QUEST.inverse());
      observedPoses.add(robotPose);

      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {

        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();
        acceptedPoses.add(robotPose.toPose2d());

        // You can put some sort of filtering here if you would like!

        if (OculusConstants.useOculus) {
          // Add the measurement to our estimator
          m_drive.addVisionMeasurement(
              robotPose.toPose2d(), timestamp, OculusConstants.QUESTNAV_STD_DEVS);
        }
      }

      Pose3d[] observedPosesArray = new Pose3d[observedPoses.size()];
      for (int i = 0; i < observedPosesArray.length; i++) {
        observedPosesArray[i] = observedPoses.get(i);
      }
      Pose2d[] acceptedPosesArray = new Pose2d[acceptedPoses.size()];
      for (int i = 0; i < acceptedPosesArray.length; i++) {
        acceptedPosesArray[i] = acceptedPoses.get(i);
      }
      Pose3d[] oculusPosesArray = new Pose3d[oculusPoses.size()];
      for (int i = 0; i < oculusPosesArray.length; i++) {
        oculusPosesArray[i] = oculusPoses.get(i);
      }

      Logger.recordOutput("Oculus/OculusPoses", oculusPosesArray);
      Logger.recordOutput("Oculus/ObservedPoses", observedPosesArray);
      Logger.recordOutput("Oculus/AcceptedPoses", acceptedPosesArray);
    }
  }

  public void resetPose() {
    questPose = new Pose3d(drivePose.get()).transformBy(OculusConstants.ROBOT_TO_QUEST);
    // Logger.recordOutput("questPose", questPose);
    questNav.setPose(questPose);
  }

  public void setPose(Pose3d pose) {
    questNav.setPose(pose);
  }
}
