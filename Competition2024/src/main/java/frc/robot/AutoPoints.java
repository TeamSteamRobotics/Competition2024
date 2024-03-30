package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoPoints {
    // 0 in the first dimension means blue!

    public static final Pose2d[][] bArray = new Pose2d[2][4];
    public static final Pose2d[][] nArray = new Pose2d[2][8];
    public static final Pose2d[][] sArray = new Pose2d[2][4];
    public AutoPoints() {
        bArray[0][0] = new Pose2d(1.42, -8.1, new Rotation2d());
        bArray[0][1] = new Pose2d(0.94, -6.14, new Rotation2d());
        bArray[0][2] = new Pose2d(1.04, -5.08, new Rotation2d());
        bArray[0][3] = new Pose2d(0.12, -4.52, new Rotation2d());

        nArray[0][0] = new Pose2d(2.89, -6.94, new Rotation2d());
        nArray[0][1] = new Pose2d(2.89, -5.52, new Rotation2d());
        nArray[0][2] = new Pose2d(2.89, -4.09, new Rotation2d());
        nArray[0][3] = new Pose2d(8.22, -7.39, new Rotation2d());
        nArray[0][4] = new Pose2d(8.22, -5.74, new Rotation2d());
        nArray[0][5] = new Pose2d(8.22, -4.09, new Rotation2d());
        nArray[0][6] = new Pose2d(8.22, -2.41, new Rotation2d());
        nArray[0][7] = new Pose2d(8.22, -0.75, new Rotation2d());

        sArray[0][0] = new Pose2d(5.01, -0.98, new Rotation2d());
        sArray[0][1] = new Pose2d(1.94, -0.97, new Rotation2d());
        sArray[0][2] = new Pose2d(0.11, -2.48, new Rotation2d());
        sArray[0][3] = new Pose2d(4.96, -8.16, new Rotation2d());
    }


}

