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
import frc.robot.commands.Teleop.DoFileTrajectory;
import frc.robot.commands.Teleop.DoTeleopRobotOrient;
import frc.robot.commands.Teleop.DoTeleopPosition;
import frc.robot.commands.Motion.ResetEncoders;
import frc.robot.commands.Motion.ResetGyro;
import frc.robot.commands.Trajectories.*;
import frc.robot.commands.Motion.LogDriveData;
import frc.robot.commands.Limelight.*;
import frc.robot.LimelightControlMode.*;
import frc.robot.commands.Limelight.LogVisionData;
import frc.robot.Gamepad;
import frc.robot.ButtonBox;
import frc.robot.Constants;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.HatchPanels.*;
import frc.robot.commands.Cargo.*;
import frc.robot.commands.Teleop.JoystickArcadeDriveVision;
import frc.robot.commands.AirCompressor.*;
import frc.robot.commands.Climber.*;
import frc.robot.commands.Motion.*;
import frc.robot.Constants;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public JoystickButton elevatorToAllLowerHatch;
    public JoystickButton elevatorToMidRocketHatch;
    public JoystickButton elevatorToTopRocketHatch;

    public JoystickButton elevatorToRocketLowerHatch;
    public JoystickButton elevatorToShipCargo;
    public JoystickButton elevatorToLowerRocketCargo;
    public JoystickButton elevatorToMidRocketCargo;
    public JoystickButton elevatorToTopRocketCargo;
    public JoystickButton elevatorToCargoLoadStation;

    public JoystickButton jogElevator;

    public JoystickButton toggleHatchCoverGripper;
    public JoystickButton extendHatchCover;
    public JoystickButton retractHatchCover;
    public JoystickButton placeHatchPanelShip;
    public JoystickButton placeHatchPanelRocket;

    public JoystickButton pushHatchCover;
    public JoystickButton retractPusher;
    public JoystickButton pickUpCargo;
    public JoystickButton deliverCargo;
    public JoystickButton abortAuto;

    public JoystickButton stopCargoHandler;

    public JoystickButton driveToVision;

    public JoystickButton jogClimberArm;
    public JoystickButton jogClimberLeg;
    public JoystickButton jogClimberDrive;
    public JoystickButton prepareLevelThree;
    public JoystickButton prepareLevelTwo;
    public JoystickButton moveArmToZero;
    public JoystickButton moveLegToZero;
    public JoystickButton lowerLeg;
    public JoystickButton staggerClimb;
    public JoystickButton mmClimb;
    public JoystickButton toggleExtension;

    public JoystickButton incrementArmAngle;
    public JoystickButton decrementArmAngle;

    public Joystick driverController = new Joystick(RobotMap.OI_DRIVER_CONTROLLER);
    public Gamepad gamepad = new Gamepad(RobotMap.OI_CO_DRIVER_CONTROLLER);
    public ButtonBox buttonBox = new ButtonBox(RobotMap.BUTTON_BOX);
    // public Gamepad gamepad_test = new Gamepad(RobotMap.OI_TEST_CONTROLLER);

    public OI() {
        Timer.delay(.02);

        SmartDashboard.putData(new ResetEncoders());
        Timer.delay(.02);
        SmartDashboard.putData(new ResetGyro());
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

        SmartDashboard.putData("Log Climber", new LogClimberData(30));

        SmartDashboard.putData("Log Elevator", new LogElevatorData(10));

        SmartDashboard.putData("Log Cargo", new LogCargoMotor(30));

        SmartDashboard.putData("Log Position", new LogPositionData(10));

        SmartDashboard.putData("Log Vision", new LogVisionData(10));

        SmartDashboard.putData("BuffToAct", new BufferToActiveTrajectory(0));

        SmartDashboard.putData("Reset Elevator", new ResetElevatorPosition());

        SmartDashboard.putData("StartCompressor", new StartCompressor());

        SmartDashboard.putData("StopCompressor", new StopCompressor());

        SmartDashboard.putData("ResetArmPosition", new ResetArmPosition());

        SmartDashboard.putData("ResetLegPosition", new ResetLegPosition());

        /**
         * 
         * Driver joystick
         */

        pickUpCargo = new JoystickButton(driverController, 1);
        pickUpCargo.whileHeld(new PickUpCargo(.5));
        pickUpCargo.whenReleased(new StopCargoMotor());
        pickUpCargo.whenReleased(new ExtendHatchPanel(false));

        deliverCargo = new JoystickButton(driverController, 2);
        deliverCargo.whileHeld(new DeliverCargo(1.));
        deliverCargo.whenReleased(new StopCargoMotor());
        deliverCargo.whenReleased(new ExtendHatchPanel(false));

        driveToVision = new JoystickButton(driverController, 3);
        driveToVision.whileHeld(new JoystickArcadeDriveVision());
        driveToVision.whenReleased(new DelayOffLeds(5.));

        placeHatchPanelRocket = new JoystickButton(driverController, 4);
        placeHatchPanelRocket.whenPressed(new PlaceHatchPanelRocket());

        toggleHatchCoverGripper = new JoystickButton(driverController, 5);
        toggleHatchCoverGripper.whenPressed(new ToggleHatchGripper());

        placeHatchPanelShip = new JoystickButton(driverController, 6);
        placeHatchPanelShip.whenPressed(new PlaceHatchPanelShip());

        abortAuto = new JoystickButton(driverController, 7);

        staggerClimb = new JoystickButton(driverController, 9);
        staggerClimb.whenPressed(new StaggerClimb());

        moveArmToZero = new JoystickButton(driverController, 10);
        moveArmToZero.whenPressed(new SetClimberTargetAngle(0.1,false));

        lowerLeg = new JoystickButton(driverController, 11);
        lowerLeg.whenPressed(new SetClimberLegTargetInches(Constants.CLIMBER_LEG_START_POSITION,false));

        /**
         * Co driver controller
         * 
         */

        jogClimberDrive = gamepad.getButtonA();
        jogClimberDrive.whileHeld(new RunClimberDriveFromGamepad());

        jogClimberLeg = gamepad.getButtonB();
        jogClimberLeg.whileHeld(new RunClimberLegFromGamepad(true));

        jogClimberArm = gamepad.getButtonX();
        jogClimberArm.whileHeld(new RunClimberArmFromGamepad(true));

        jogElevator = gamepad.getButtonY();
        jogElevator.whileHeld(new RunElevatorFromGamepad());

        stopCargoHandler = gamepad.getBackButton();
        stopCargoHandler.whenPressed(new StopCargoMotor());

        incrementArmAngle = gamepad.getLeftShoulder();
        incrementArmAngle.whenPressed(new IncrementArmAngle(10));

        decrementArmAngle = gamepad.getLeftTriggerClick();
        decrementArmAngle.whenPressed(new IncrementArmAngle(-10));

        moveLegToZero = gamepad.getBackButton();
        moveLegToZero.whenPressed(new SetClimberLegTargetInches(0,false));

        staggerClimb = gamepad.getRightTriggerClick();
        staggerClimb.whenPressed(new StaggerClimb());
        

        mmClimb = gamepad.getRightShoulder();
        mmClimb.whenPressed(new MMClimb(20));

        /**
         * 
         * 
         * Button box
         */
        elevatorToRocketLowerHatch = buttonBox.getButtonB();

        elevatorToAllLowerHatch = buttonBox.getButtonRT();
        elevatorToMidRocketHatch = buttonBox.getButtonA();
        elevatorToTopRocketHatch = buttonBox.getButtonLT();

        elevatorToShipCargo = buttonBox.getButtonR1();
        elevatorToLowerRocketCargo = buttonBox.getButtonY();
        elevatorToMidRocketCargo = buttonBox.getButtonX();
        elevatorToTopRocketCargo = buttonBox.getButtonL1();
        elevatorToCargoLoadStation = buttonBox.getButtonOptions();

        elevatorToRocketLowerHatch.whenPressed(new SetElevatorTargetHeight(Constants.ALL_LOWER_HATCH_INCHES));

        elevatorToAllLowerHatch.whenPressed(new SetElevatorTargetHeight(Constants.ALL_LOWER_HATCH_INCHES));
        elevatorToMidRocketHatch.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_MID_HATCH_INCHES));
        elevatorToTopRocketHatch.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_TOP_HATCH_INCHES));

        elevatorToShipCargo.whenPressed(new SetElevatorTargetHeight(Constants.SHIP_CARGO_INCHES));
        elevatorToLowerRocketCargo.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_LOWER_CARGO_INCHES));
        elevatorToMidRocketCargo.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_MID_CARGO_INCHES));
        elevatorToTopRocketCargo.whenPressed(new SetElevatorTargetHeight(Constants.ROCKET_TOP_CARGO_INCHES));

        prepareLevelTwo = buttonBox.getBackButton();
        prepareLevelTwo.whenPressed(new SetClimberTargetAngle(Constants.LEVEL_2_START_ANGLE,false));

        prepareLevelThree = buttonBox.getStartButton();
        prepareLevelThree.whenPressed(new SetClimberTargetAngle(Constants.LEVEL_3_START_ANGLE,false));

        toggleExtension = buttonBox.getButtonOptions();
        toggleExtension.whenPressed(new ToggleExtendHatchPanel());

    }
}
