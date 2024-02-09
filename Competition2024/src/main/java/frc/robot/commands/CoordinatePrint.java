package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;
public class CoordinatePrint extends Command{
    private AprilVisionSubsystem av;
    private int id;
    private ReturnTarget tt;
    public CoordinatePrint(AprilVisionSubsystem p_av, int targetId, ReturnTarget targetType){
        av = p_av;
        id = targetId;
        tt = targetType;
        addRequirements(av);
    }

    @Override
    public void execute() {
        System.out.println("X: " + av.getCoordinates(id, tt).x);
        System.out.println("Y: " + av.getCoordinates(id, tt).y);
        System.out.println("Z: " + av.getCoordinates(id, tt).z);
    }
}
