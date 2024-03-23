// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DigitalIOID;
import frc.robot.Constants.OdometryConsts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class DriveSubsystem extends SubsystemBase {
  private WPI_TalonSRX frontLeftMotor;
  private WPI_TalonSRX frontRightMotor;
  private WPI_VictorSPX backLeftMotor;
  private WPI_VictorSPX backRightMotor;

  private AHRS navX;

  private DifferentialDrive diffDrive;
  
  private DifferentialDriveOdometry odometry;

  private Pose2d currentRobotPose;

  private DifferentialDriveKinematics kinematics;
  
  private Field2d field = new Field2d();

  public DriveSubsystem() {

    
    frontLeftMotor = new WPI_TalonSRX(CANID.frontLeft);
    frontRightMotor = new WPI_TalonSRX(CANID.frontRight);
    backLeftMotor = new WPI_VictorSPX(CANID.backLeft);
    backRightMotor = new WPI_VictorSPX(CANID.backRight);
    
    frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    //frontLeftMotor.follow(backLeftMotor);
    //frontRightMotor.follow(backRightMotor);

    frontLeftMotor.setSensorPhase(true);
    frontRightMotor.setSensorPhase(true);

    resetEncoders();
    diffDrive = new DifferentialDrive(backLeftMotor, backRightMotor);

    navX = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftSideMeters(), getRightSideMeters(), new Pose2d(1 ,1 , new Rotation2d(0)));

    kinematics = new DifferentialDriveKinematics(DriveConstants.dConstants.kTrackWidthMeters);

    SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> resetEncoders(), this));
    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> resetGyro(), this));
    
     AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::autoDrive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );   

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  
  public ChassisSpeeds getCurrentSpeeds(){
    System.out.println("CURRENT SPEED: " + kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(encoderTicksToMeters(frontLeftMotor.getSelectedSensorVelocity(0) * 10), encoderTicksToMeters(frontRightMotor.getSelectedSensorVelocity(0) * 10))));
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(encoderTicksToMeters(frontLeftMotor.getSelectedSensorVelocity(0) * 10), -encoderTicksToMeters(frontRightMotor.getSelectedSensorVelocity(0) * 10)));
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  
  public void resetPose(Pose2d pose){
    odometry.resetPosition(navX.getRotation2d(), getLeftSideMeters(), getRightSideMeters(), pose);
  }
  public void autoDrive(ChassisSpeeds speeds) { 
    double wheelCircumferenceMeters = DriveConstants.dConstants.kCircumfrance;
    
    // Convert ChassisSpeeds to wheel speeds in meters per second.
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
   // DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
   //     speeds.vxMetersPerSecond - (speeds.omegaRadiansPerSecond * DriveConstants.dConstants.kTrackWidthMeters / 2),
   //     speeds.vxMetersPerSecond + (speeds.omegaRadiansPerSecond * DriveConstants.dConstants.kTrackWidthMeters / 2)
    //);

    // Convert wheel speeds from meters per second to RPM.
    double leftRPM = (wheelSpeeds.leftMetersPerSecond / wheelCircumferenceMeters) * 60;
    double rightRPM = (wheelSpeeds.rightMetersPerSecond / wheelCircumferenceMeters) * 60;

    System.out.println(rightRPM);

    // Assuming your motors are configured to accept speed values as a percentage of their maximum RPM,
    // convert RPM to a normalized [-1, 1] value based on the maximum RPM.
    double leftOutput = leftRPM / DriveConstants.dConstants.kMaxRPM;
    System.out.println(leftOutput);
    double rightOutput = -rightRPM / DriveConstants.dConstants.kMaxRPM;

    // Apply the normalized values to your motors. Adjust as necessary for your specific setup.
    frontLeftMotor.set(leftOutput);
    frontRightMotor.set(rightOutput);
}

  public void drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void curveDrive(double speed, double rotation) {
    diffDrive.curvatureDrive(speed, rotation, true);
  }

  public void stop(){
    frontLeftMotor.set(TalonSRXControlMode.PercentOutput, 0);
    frontRightMotor.set(TalonSRXControlMode.PercentOutput, 0);
    backLeftMotor.set(VictorSPXControlMode.PercentOutput, 0);
    backRightMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  //Average of both left side motor encoders

  //Average of both right side motor encoders


  public void resetEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }

  public double getLeftSideMeters() {
    return encoderTicksToMeters(frontLeftMotor.getSelectedSensorPosition(0));
  }

  public double getRightSideMeters() {
    return -encoderTicksToMeters(frontRightMotor.getSelectedSensorPosition(0));
  }

  public double getDistanceMeters() {
    return -(getLeftSideMeters());// +// getRightSideMeters());// / 2;
  }

  public double getRateMetersPerSecond() {
    
    return encoderTicksToMeters((frontLeftMotor.getSelectedSensorVelocity() + frontRightMotor.getSelectedSensorVelocity()) / 2);
  }

  private double encoderTicksToMeters(double ticks) {
    double rotations = ticks / DriveConstants.dConstants.encoderTicksPerRevolution;
    return rotations * DriveConstants.dConstants.kCircumfrance;
  }

  public void resetGyro() {
    navX.reset();
  }

  public double getAngleDegrees() {
    return navX.getAngle();
  }

  public Rotation2d getRotation2d() {
    return navX.getRotation2d();
  }
  
  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public Pose2d getRobotPose() {
    return currentRobotPose;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRobotPose = odometry.update(getRotation2d(), getLeftSideMeters(), getRightSideMeters());

    SmartDashboard.putNumber("Through Bore Encoder Distance", getDistanceMeters());

    SmartDashboard.putNumber("Gyro reading", getAngleDegrees());    

    field.setRobotPose(getPose());
  }
}




