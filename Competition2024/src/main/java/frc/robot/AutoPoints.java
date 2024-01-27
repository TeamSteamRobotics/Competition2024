package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoPoints {
    // 0 in the first dimension means blue!

    public static final Pose2d[][] bArray = new Pose2d[2][5];
    public static final Pose2d[][] nArray = new Pose2d[2][7];
    public static final Pose2d[][] sArray = new Pose2d[2][4];
    public AutoPoints() {
        bArray[0][2] = new Pose2d(0, 0, new Rotation2d());
    }


}

