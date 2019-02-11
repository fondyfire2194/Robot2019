/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoFileTrajectory;
import frc.robot.commands.DoTeleopRobotOrient;
import frc.robot.commands.DoTeleopRotateToVision;
import frc.robot.commands.DoTeleopPosition;
import frc.robot.commands.Motion.ResetEncoders;
import frc.robot.commands.Motion.ResetGyro;
import frc.robot.commands.Trajectories.*;
import frc.robot.commands.Motion.LogDriveData;
import frc.robot.commands.Elevator.LogElevatorData;
import frc.robot.commands.Limelight.*;
import frc.robot.commands.SetGyroOffset;
import frc.robot.LimelightControlMode.*;
import frc.robot.commands.Limelight.LogVisionData;;
import frc.robot.Gamepad;
import frc.robot.Constants;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.HatchPanels.*;
import frc.robot.commands.Cargo.*;
import frc.robot.commands.JoystickArcadeDriveVision;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public JoystickButton elevatorToLowerHatch;
    public JoystickButton elevatorToMidRocket;
    public JoystickButton elevatorToTopRocket;
    public JoystickButton jogElevator;

    public JoystickButton captureHatchCover;
    public JoystickButton releaseHatchCover;
    public JoystickButton extendHatchCover;
    public JoystickButton retractHatchCover;

    public JoystickButton secondExtendHatchCover;
    public JoystickButton secondRetractHatchCover;

    public JoystickButton pushHatchCover;
    public JoystickButton retractPusher;


    public JoystickButton captureAndRetractHatchCover;
    public JoystickButton presentAndReleaseHatchCoverShip;
    public JoystickButton presentAndReleaseHatchCoverRocket;  

    public JoystickButton intakeCargo;
    public JoystickButton deliverCargo;
    public JoystickButton stopCargoHandler;

    public JoystickButton driveToVision;

    public Joystick driverController = new Joystick(RobotMap.OI_DRIVER_CONTROLLER);
    public Gamepad gamepad = new Gamepad(RobotMap.OI_CO_DRIVER_CONTROLLER);
    public Gamepad gamepad_test = new Gamepad(RobotMap.OI_TEST_CONTROLLER);


    public OI() {
        Timer.delay(.02);

        SmartDashboard.putData(new ResetEncoders());
        Timer.delay(.02);
        SmartDashboard.putData(new ResetGyro());
        Timer.delay(.02);
        SmartDashboard.putData("OrientRobotVision", new DoTeleopRotateToVision());
        Timer.delay(.02);
        SmartDashboard.putData("OrientRobot", new DoTeleopRobotOrient());
        Timer.delay(.02);

        SmartDashboard.putData("DriveToTarget", new DoTeleopPosition());
        Timer.delay(.02);
        SmartDashboard.putData("File Traj", new DoFileTrajectory());
        Timer.delay(.02);
        SmartDashboard.putData("LEDS On", new SetLimelightLeds(LedMode.kforceOn));
        SmartDashboard.putData("LEDS Off", new SetLimelightLeds(LedMode.kforceOff));

        SmartDashboard.putData("Toggle View", new ToggleCamMode());
        SmartDashboard.putData("Toggle Stream", new ToggleStreamMode());

        SmartDashboard.putData("BuffToAct", new BufferToActiveTrajectory(1));

        SmartDashboard.putData("Log Drive", new LogDriveData(10));

        SmartDashboard.putData("Log Elevator", new LogElevatorData(10));

        SmartDashboard.putData("Log Vision", new LogVisionData(15));

        SmartDashboard.putData("BuffToAct", new BufferToActiveTrajectory(0));

        SmartDashboard.putData("Set Gyro Offset", new SetGyroOffset(180));
        /**
         * Co driver controller
         * 
         */

        elevatorToLowerHatch = gamepad.getButtonB();

        elevatorToLowerHatch.whenPressed(new SetElevatorTargetHeight(Constants.ALL_LOWER_HATCH_INCHES));

        elevatorToMidRocket = gamepad.getButtonY();

        elevatorToMidRocket.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_MID_HATCH_INCHES));

        elevatorToTopRocket = gamepad.getButtonX();

        elevatorToTopRocket.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_TOP_HATCH_INCHES));

        jogElevator = gamepad.getStartButton();
        jogElevator.whileHeld(new RunElevatorFromGamepad());

        /**
         * Teat Controller
         * 
         */

        captureHatchCover = gamepad_test.getLeftShoulder();
        captureHatchCover.whenPressed(new GripHatchPanel(true));

        releaseHatchCover = gamepad_test.getLeftTriggerClick();
        releaseHatchCover.whenPressed(new GripHatchPanel(false));

        extendHatchCover = gamepad_test.getRightShoulder();
        extendHatchCover.whenPressed(new ExtendHatchPanel(true));

        retractHatchCover = gamepad_test.getRightTriggerClick();
        retractHatchCover.whenPressed(new ExtendHatchPanel(false));

        secondExtendHatchCover = gamepad_test.getButtonY();
        secondExtendHatchCover.whenPressed(new ExtendHatchPanel(true));

        secondRetractHatchCover = gamepad_test.getButtonA();
        secondRetractHatchCover.whenPressed(new ExtendHatchPanel(false));

       pushHatchCover = gamepad_test.getButtonB();
       pushHatchCover.whenPressed(new PushHatchPanel(true));

       retractPusher = gamepad_test.getButtonX();
       retractPusher.whenPressed(new PushHatchPanel(false));

        captureAndRetractHatchCover = gamepad_test.getLeftTriggerClick();
        captureAndRetractHatchCover.whenPressed(new PickUpHatchPanel());

        presentAndReleaseHatchCoverShip = gamepad_test.getRightTriggerClick();
        presentAndReleaseHatchCoverShip.whenPressed(new PlaceHatchPanelShip());

        presentAndReleaseHatchCoverRocket = gamepad_test.getStartButton();
        presentAndReleaseHatchCoverRocket.whenPressed(new PlaceHatchPanelShip());

        if(gamepad_test.getDPadLeft()) new PickUpCargo(.5).start();

        if(gamepad_test.getDPadRight()) new DeliverCargo(.5).start();

        stopCargoHandler = gamepad_test.getBackButton();
        stopCargoHandler.whenPressed(new StopCargoMotor());

        driveToVision = new JoystickButton(driverController,1);
        driveToVision.whileHeld(new JoystickArcadeDriveVision());
        

    }
}
