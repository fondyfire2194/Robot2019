package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SD;
import frc.robot.AutoChoosers;
import frc.robot.Pref;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Counter;

public class GamePieceHandler extends Subsystem {
	public static TalonSRX cargoMotor;
	public DoubleSolenoid hatchCoverGripper;
	public Solenoid hatchCoverExtend;
	public Solenoid hatchCoverPusher;
	private int gripperCounter;
	private boolean stopCargoIntake;
	private DigitalInput leftPusherBackSensor;
	private DigitalInput rightPusherBackSensor;
	private DigitalInput leftPusherForwardSensor;
	private DigitalInput rightPusherForwardSensor;
	private int intakeAmpsCounter;

	public static boolean hatchGripped;

	public GamePieceHandler() {

		cargoMotor = new TalonSRX(RobotMap.CARGO_MOTOR);
		cargoMotor.configFactoryDefault();
		cargoMotor.setNeutralMode(NeutralMode.Brake);
		cargoMotor.configVoltageCompSaturation(12, 0);
		cargoMotor.enableVoltageCompensation(true);

		leftPusherBackSensor = new DigitalInput(RobotMap.LEFT_PUSHER_BACK_SWITCH);
		leftPusherForwardSensor = new DigitalInput(RobotMap.LEFT_PUSHER_FWD_SWITCH);		
		rightPusherBackSensor = new DigitalInput(RobotMap.RIGHT_PUSHER_BACK_SWITCH);
		rightPusherForwardSensor = new DigitalInput(RobotMap.RIGHT_PUSHER_FWD_SWITCH);

		hatchCoverExtend = new Solenoid(2);
		hatchCoverExtend.set(false);

		hatchCoverGripper = new DoubleSolenoid(1, 0);
		hatchCoverGripper.set(DoubleSolenoid.Value.kForward);
		hatchGripped = true;

		hatchCoverPusher = new Solenoid(6);
		hatchCoverPusher.set(false);


	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {

	}

	public void stopCargoMotor() {
		cargoMotor.set(ControlMode.PercentOutput, 0);
	}

	public void pickUpCargo(double speed) {
		stopCargoIntake = checkCargoIntakeAmps();
		if (!stopCargoIntake)
			cargoMotor.set(ControlMode.PercentOutput, speed);
		else {
			cargoMotor.set(ControlMode.PercentOutput, 0);
			Robot.elevator.holdPositionInches = Constants.ROCKET_LOWER_CARGO_INCHES;
		}
	}

	public void deliverCargo(double speed) {
		stopCargoIntake = false;
		cargoMotor.set(ControlMode.PercentOutput, -speed);
	}

	public void gripHatchPanel() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kForward);
		gripperCounter = 0;
		hatchGripped = true;
	}

	public void releaseHatchPanel() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kReverse);
		gripperCounter = 0;
		hatchGripped = false;
	}

	private void gripHatchPanelOff() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kOff);
	}

	public void extendHatchPanel() {
		hatchCoverExtend.set(true);
	}

	public void retractHatchPanel() {
		hatchCoverExtend.set(false);
	}

	public void pushHatchPanel() {
		hatchCoverPusher.set(true);
	}

	public void retractPusher() {
		hatchCoverPusher.set(false);

	}

	public boolean getLeftSideDetected() {
		return !leftPusherBackSensor.get();
	}

	public boolean getRightSideDetected() {
		return !rightPusherBackSensor.get();
	}

	public boolean eitherSideDetected() {
		return getLeftSideDetected() || getRightSideDetected();
	}

	public boolean bothSidesDetected() {
		return getLeftSideDetected() && getRightSideDetected();
	}

	public boolean getLeftSideExtended() {
		return !leftPusherForwardSensor.get();
	}

	public boolean getRightSideExtended() {
		return !rightPusherForwardSensor.get();
	}

	public boolean eitherSideExtended() {
		return getLeftSideExtended() || getRightSideExtended();
	}

	public boolean bothSidesExtended() {
		return getLeftSideExtended() && getRightSideExtended();
	}

	public void updateStatus() {

		if (hatchCoverGripper.get() != DoubleSolenoid.Value.kOff)
			gripperCounter++;
		if (gripperCounter > 2) {
			gripHatchPanelOff();
		}

		SD.putN1("CargoMotorAmps", cargoMotor.getOutputCurrent());
		SD.putN1("CargoMotorPct", cargoMotor.getMotorOutputPercent());
		SmartDashboard.putBoolean("LeftDetected", getLeftSideDetected());

		SmartDashboard.putBoolean("RightDetected", getRightSideDetected());

		if (AutoChoosers.debugChooser.getSelected() == 4) {
			SD.putN1("CargoMotorVolts", cargoMotor.getBusVoltage());

		}
	}

	private boolean checkCargoIntakeAmps() {
		if (cargoMotor.getOutputCurrent() > Pref.getPref("CargoIntakeAmpsLimit")) {
			intakeAmpsCounter++;
		} else {
			intakeAmpsCounter = 0;
		}
		return  intakeAmpsCounter > 25;
	}

}
