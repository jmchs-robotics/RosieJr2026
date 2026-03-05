package frc.robot.subsystems.oculus;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class OculusConstants {

  public static Transform3d ROBOT_TO_QUEST = new Transform3d();

  public static Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // the pose can be within 2 cm in the x of where you want to be
          0.02, // the pose can be within 2 cm in the y of where you want to be
          0.035 // the pose can be within 2 degrees of where you want to be
          );
}
