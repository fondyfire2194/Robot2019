/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PathfinderReverseTrajectory;
import frc.robot.commands.PathfinderTrajectory;
import frc.robot.commands.RobotOrient;
import frc.robot.BuildTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RobotRotate;
import jaci.pathfinder.Trajectory;

import frc.robot.LimeLight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static DriveTrain driveTrain = null;
  public static RobotRotate robotRotate;
  public static SimpleCSVLogger simpleCSVLogger;
  public static LimeLight limelightCamera;
  public static String chosenFileName = "Dummy";
  public static OI m_oi;
  public static Preferences prefs;
  public static BuildTrajectory buildTrajectory;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static boolean doRobotPosition;
  public static double positionTargetFt;
  public static boolean incrPosn;
  public static boolean isPositioning;
  public static boolean stopPositioning;
  public static boolean orientRunning;
  public static boolean reverseOrient;
  public static boolean doTeleopOrient;

  public static double targetPosition;
  public static double targetRate;
  public static double xPosition;
  public static double yPosition;

  private double angleTarget = 90;
  private double orientRate = 0.5;

  public enum motionType {
    incremental, absolute
  }

  public static Trajectory[] activeTrajectory;// = new Trajectory[2];
  public static Trajectory[] bufferTrajectory;
  public static double[] activeTrajectoryGains = { 0, 0, 0, 0 };
  public static double[] activeTrajectoryTwoGains = { 0, 0, 0, 0 };
  public static boolean doTeleopTrajectory;
  public static boolean revTraj;;
  public static boolean doFileTrajectory;
  public static boolean trajectoryRunning;
  public static boolean buildOK;

  public static String[] names = { "Step", "LeftCmd", "LeftFt", "RightCmd", "RightFt", "AngleCmd", "Angle",
      "LeftSegVel", "left", "ActLeftVel", "RightSegVel", "right", "ActRightVel", "turn" };
  public static String[] units = { "Number", "FT", "FT", "FT", "FT", "Deg", "Deg", "pct", "pct", "pct", "pct", "pct",
      "pct", "pct" };
  public static boolean createTrajectoryRunFile = true;
  public static double trajectoryX;
  public static double trajectoryY;
  public static double trajectoryAngle;
  public static String trajFileName;
  public static String logName;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    robotRotate = new RobotRotate();
    m_oi = new OI();
    simpleCSVLogger = new SimpleCSVLogger();
    limelightCamera= new LimeLight();
    buildTrajectory = new BuildTrajectory();
    prefs = Preferences.getInstance();
    // Pref.deleteUnused();
    Pref.addMissing();
    SmartDashboard.putData(driveTrain);
    Timer.delay(.02);

    SmartDashboard.putNumber("Target Angle", angleTarget);
    Timer.delay(.02);
    SmartDashboard.putNumber("Orient Rate", orientRate);
    Timer.delay(.02);
    SmartDashboard.putBoolean("RevOrient", false);
    Timer.delay(.02);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateStatus();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = AutoChoosers.startPositionChooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    if (doTeleopOrient) {
      // sensors.resetGyro();

      angleTarget = SmartDashboard.getNumber("Target Angle", 90);
      if (SmartDashboard.getBoolean("ReverseOrient", false))
        angleTarget = -angleTarget;
      orientRate = SmartDashboard.getNumber("Orient Rate", .25);
      new RobotOrient(angleTarget, orientRate, true, 30).start();
      doTeleopOrient = false;
    }

    /**
     * Create trajectory from Shuffleboard x, y and angle fields put it in active
     * trajectory array. [0] is left [1] is right get the 4 trajectory loop gains
     * (P, D, A and Turn)from prefs start the trajectory notifier and data logger
     * 
     * 
     */

    if (doFileTrajectory) {

      // testTrajectory = testTrajectoryChooser.getSelected();

      logName = trajFileName;
      activeTrajectory = BuildTrajectory.buildFileName( true, trajFileName);
      SmartDashboard.putBoolean("FileOK", buildOK);

      if (!buildOK) {
        doFileTrajectory = false;
        DriverStation.reportError("Error reading file", true);
      }
    }
    if (doTeleopTrajectory || (doFileTrajectory && buildOK)) {

      driveTrain.resetEncoders();
      driveTrain.resetGyro();
      xPosition = activeTrajectory[0].get(0).x;
      yPosition = activeTrajectory[0].get(0).y - (Constants.WHEELBASE_WIDTH / 2);
      constantsFromPrefs();

      int trajectoryDirectionChooser = AutoChoosers.trajectoryDirectionChooser.getSelected();

      switch (trajectoryDirectionChooser) {

        case 0:
        new PathfinderTrajectory(true).start();
        constantsFromPrefs();
        break;
        case 1:
        new PathfinderTrajectory(false).start();
        revConstantsFromPrefs();
        break;
        case 2:
        new PathfinderReverseTrajectory(true).start();
        constantsFromPrefs();
        break;
        case 3:
        new PathfinderReverseTrajectory(false).start();
        revConstantsFromPrefs();
        break;

      }

 
      trajectoryRunning = true;
      doTeleopTrajectory = false;
      doFileTrajectory = false;
    }
    doFileTrajectory = false;

    updateStatus();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void updateStatus() {
    driveTrain.updateStatus();
    SmartDashboard.putBoolean("PosnRng", isPositioning);
    SmartDashboard.putBoolean("TrajRng", trajectoryRunning);
    SmartDashboard.putBoolean("OrientRng", orientRunning);
    SmartDashboard.putNumber("TrajLen", activeTrajectory == null ? 0 : activeTrajectory[0].length());
    SmartDashboard.putNumber("Preftest", Pref.getPref("y"));
    SmartDashboard.putString("FileChosen", chosenFileName);

  }

  private void constantsFromPrefs() {
    activeTrajectoryGains[0] = Pref.getPref("PathKp");// prefs.getDouble("PathP", 0);
    activeTrajectoryGains[1] = Pref.getPref("PathKd");// prefs.getDouble("PathD", 0);
    activeTrajectoryGains[2] = Pref.getPref("PathKa");// prefs.getDouble("PathA", 0);
    activeTrajectoryGains[3] = Pref.getPref("PathKt");// prefs.getDouble("PathTurn", 0);
  }

  private void revConstantsFromPrefs() {
    activeTrajectoryGains[0] = Pref.getPref("PathKpRev");// prefs.getDouble("PathP", 0);
    activeTrajectoryGains[1] = Pref.getPref("PathKdRev");// prefs.getDouble("PathD", 0);
    activeTrajectoryGains[2] = Pref.getPref("PathKaRev");// prefs.getDouble("PathA", 0);
    activeTrajectoryGains[3] = Pref.getPref("PathKtRev");// prefs.getDouble("PathTurn", 0);
  }
}
