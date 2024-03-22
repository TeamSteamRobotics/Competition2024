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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax backLeftMotor;
  private CANSparkMax backRightMotor;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder backLeftEncoder;
  private RelativeEncoder backRightEncoder;

  private Encoder leftThroughBoreEncoder;
  private Encoder rightThroughBoreEncoder;

  private AHRS navX;

  private DifferentialDrive diffDrive;
  
  private DifferentialDriveOdometry odometry;

  private Pose2d currentRobotPose;

  private DifferentialDriveKinematics kinematics;
  
  private Field2d field = new Field2d();

  public DriveSubsystem() {

    frontLeftMotor = new CANSparkMax(CANID.frontLeft, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(CANID.frontRight, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(CANID.backLeft, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(CANID.backRight, MotorType.kBrushless);
    
    diffDrive = new DifferentialDrive(backLeftMotor, backRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    leftThroughBoreEncoder = new Encoder(DigitalIOID.leftDriveEncoder1, DigitalIOID.leftDriveEncoder2);
    rightThroughBoreEncoder = new Encoder(DigitalIOID.rightDriveEncoder1, DigitalIOID.rightDriveEncoder2); 

    leftThroughBoreEncoder.setDistancePerPulse(OdometryConsts.wheelCircumfrenceMeters / 2048);
    rightThroughBoreEncoder.setDistancePerPulse(OdometryConsts.wheelCircumfrenceMeters / 2048);

    leftThroughBoreEncoder.setReverseDirection(true);
    rightThroughBoreEncoder.setReverseDirection(false); 

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
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftThroughBoreEncoder.getRate(), rightThroughBoreEncoder.getRate()));
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  
  public void resetPose(Pose2d pose){
    odometry.resetPosition(navX.getRotation2d(), getLeftSideMeters(), getRightSideMeters(), pose);
  }
  public void autoDrive(ChassisSpeeds speeds) {
    double wheelCircumferenceMeters = 2 * Math.PI * DriveConstants.dConstants.kWheelRadiusMeters;
    
    // Convert ChassisSpeeds to wheel speeds in meters per second.
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
   // DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
   //     speeds.vxMetersPerSecond - (speeds.omegaRadiansPerSecond * DriveConstants.dConstants.kTrackWidthMeters / 2),
   //     speeds.vxMetersPerSecond + (speeds.omegaRadiansPerSecond * DriveConstants.dConstants.kTrackWidthMeters / 2)
    //);

    // Convert wheel speeds from meters per second to RPM.
    double leftRPM = (wheelSpeeds.leftMetersPerSecond / wheelCircumferenceMeters) * 60;
    double rightRPM = (wheelSpeeds.rightMetersPerSecond / wheelCircumferenceMeters) * 60;

    // Assuming your motors are configured to accept speed values as a percentage of their maximum RPM,
    // convert RPM to a normalized [-1, 1] value based on the maximum RPM.
    double leftOutput = leftRPM / DriveConstants.dConstants.kMaxRPM;
    double rightOutput = rightRPM / DriveConstants.dConstants.kMaxRPM;

    // Apply the normalized values to your motors. Adjust as necessary for your specific setup.
    frontLeftMotor.set(leftOutput);
    frontRightMotor.set(rightOutput);
    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
}

  public void drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void curveDrive(double speed, double rotation) {
    diffDrive.curvatureDrive(speed, rotation, true);
  }

  public void stop(){
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    backLeftMotor.set(0);
    backRightMotor.set(0);
  }

  //Average of both left side motor encoders
  public double getLeftSideBuiltInRotations() {
    return (frontLeftEncoder.getPosition() + backLeftEncoder.getPosition()) / 2;
  }

  //Average of both right side motor encoders
  public double getRightSideBuiltInRotations() {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition()) / 2;
  }

  public double getLeftSideDistanceBuiltInMeters() {
    return (getLeftSideBuiltInRotations() * Constants.OdometryConsts.rotationsToMeters);
  }

  public double getRightSideDistanceBuiltInMeters() {
    return (getRightSideBuiltInRotations() * Constants.OdometryConsts.rotationsToMeters);
  }

  public double getBuiltInEncoderDistanceMeters() {
    return (getLeftSideDistanceBuiltInMeters() + getRightSideDistanceBuiltInMeters()) / 2;
  }

  public void resetEncoders() {
    rightThroughBoreEncoder.reset();
    leftThroughBoreEncoder.reset();
  }

  public double getRightSideMeters() {
    return rightThroughBoreEncoder.getDistance();
  }

  public double getLeftSideMeters() {
    return leftThroughBoreEncoder.getDistance();
  }

  public double getDistanceMeters() {
    return -(getLeftSideMeters());// +// getRightSideMeters());// / 2;
  }

  public double getRateMetersPerSecond() {
    return (rightThroughBoreEncoder.getRate() + leftThroughBoreEncoder.getRate()) / 2;
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




