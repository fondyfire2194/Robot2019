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
	private DigitalInput leftPusherSensor;
	private DigitalInput rightPusherSensor;
	private int intakeAmpsCounter;

	public static boolean hatchGripped;

	public GamePieceHandler() {

		cargoMotor = new TalonSRX(RobotMap.CARGO_MOTOR);
		cargoMotor.configFactoryDefault();
		cargoMotor.setNeutralMode(NeutralMode.Brake);
		cargoMotor.configVoltageCompSaturation(12, 0);
		cargoMotor.enableVoltageCompensation(true);

		leftPusherSensor = new DigitalInput(0);

		rightPusherSensor = new DigitalInput(1);

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

	public boolean getLeftHatchDetected() {
		return !leftPusherSensor.get();
	}

	public boolean getRightHatchDetected() {
		return !rightPusherSensor.get();
	}

	public boolean eitherHatchDetected() {
		return !leftPusherSensor.get() || !rightPusherSensor.get();
	}

	public boolean bothHatchesDetected() {
		return !leftPusherSensor.get() && !rightPusherSensor.get();
	}

	public void updateStatus() {

		if (hatchCoverGripper.get() != DoubleSolenoid.Value.kOff)
			gripperCounter++;
		if (gripperCounter > 2) {
			gripHatchPanelOff();
		}

		SD.putN1("CargoMotorAmps", cargoMotor.getOutputCurrent());
		SD.putN1("CargoMotorPct", cargoMotor.getMotorOutputPercent());
		SmartDashboard.putBoolean("LeftDetected", getLeftHatchDetected());

		SmartDashboard.putBoolean("RightDetected", getRightHatchDetected());

		if (AutoChoosers.debugChooser.getSelected() == 4) {
			SD.putN1("CargoMotorVolts", cargoMotor.getBusVoltage());

		}
	}

	private boolean checkCargoIntakeAmps() {
		if (cargoMotor.getOutputCurrent() > Pref.getPref("CargoIntakeAmpsLimit") * .8) {
			intakeAmpsCounter++;
		} else {
			intakeAmpsCounter = 0;
		}
		return cargoMotor.getOutputCurrent() > Pref.getPref("CargoIntakeAmpsLimit") || intakeAmpsCounter > 10;
	}

}
