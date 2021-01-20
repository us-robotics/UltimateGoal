package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;

public class Launcher extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method
	 *
	 * @param opMode
	 */
	public Launcher(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		launcher = hardwareMap.dcMotor.get("launcher");
		trigger = hardwareMap.servo.get("trigger");

		launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		setLauncherPower();
		setTriggerPosition();

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN);
	}

	private DcMotor launcher;
	private Servo trigger;

	private float power;
	private boolean hit;

	private float maxPower = 0.7375f;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.getIsAuto())
		{
			Input input = opMode.getHelper(Input.class);

			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER)) power = 1f - power;
			hit = input.getButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

			float MaxPowerChangeRate = 0.0025f;

			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP)) maxPower += MaxPowerChangeRate;
			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN)) maxPower -= MaxPowerChangeRate;

			opMode.getHelper(Telemetry.class).addData("Launcher Max Power", maxPower);
		}

		setLauncherPower();
		setTriggerPosition();
	}

	private void setLauncherPower()
	{
		launcher.setPower(power * maxPower);
	}

	private void setTriggerPosition()
	{
		trigger.setPosition(hit ? 0d : 0.45d);
	}

	public void setPower(float power)
	{
		this.power = power;
	}

	public void setHit(boolean hit)
	{
		this.hit = hit;
	}
}
