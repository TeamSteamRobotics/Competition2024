package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class StringParsing {

	public static String[] parsePointList(String input) {
					String noSpaces = input.replace(" ", "");
					String allCaps = noSpaces.toUpperCase();
					return allCaps.split(",");
		}
	public static Pose2d parseStringPoint(String input, boolean isBlue) {
		char letter = input.charAt(0);
		int posNum = Integer.parseInt(String.valueOf(input.charAt(1)));
		int sideNum = isBlue ? 0 : 1; // should be 0 for true
		switch (letter) {
			case 'B':
				return AutoPoints.bArray[sideNum][(posNum - 1)];
			case 'N':
				return AutoPoints.nArray[sideNum][(posNum - 1)];
			case 'S':
				return AutoPoints.sArray[sideNum][(posNum - 1)];
			default:
				return new Pose2d();
		}
	}
}
