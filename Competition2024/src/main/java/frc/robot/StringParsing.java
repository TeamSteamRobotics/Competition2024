package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TopLevelAuto;
import frc.robot.commands.TopLevelAuto.allianceColor;

public class StringParsing {

	public static String[] parsePointList(String input) {
		String noSpaces = input.replace(" ", "");
		String allCaps = noSpaces.toUpperCase();
		return allCaps.split(",");
	}

	public static Pose2d parseStringPoint(String input, allianceColor currentAlliance) {
		char letter = input.charAt(0);
		int posNum = Integer.parseInt(String.valueOf(input.charAt(1)));
		switch (letter) {
			case 'B':
				return AutoPoints.bArray[currentAlliance.getValue()][(posNum - 1)];
			case 'N':
				return AutoPoints.nArray[currentAlliance.getValue()][(posNum - 1)];
			case 'S':
				return AutoPoints.sArray[currentAlliance.getValue()][(posNum - 1)];
			default:
				return new Pose2d();
		}
	}
}
