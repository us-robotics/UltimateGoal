package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;
import FTCEngine.Math.Mathf;

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
		jeff = hardwareMap.servo.get("jeff");

		launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN);

		setLauncherPower();
		setTriggerPosition();
		setJeffJeffing();
	}

	private DcMotor launcher;
	private Servo trigger;
	private Servo jeff;

	private float power;
	private boolean hit;
	private float jeffing;

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
			jeffing = input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.LEFT_TRIGGER);

			float MaxPowerChangeRate = 0.0025f;

			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP)) maxPower += MaxPowerChangeRate;
			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN)) maxPower -= MaxPowerChangeRate;

			opMode.getHelper(Telemetry.class).addData("Launcher Max Power", maxPower);
		}

		setLauncherPower();
		setTriggerPosition();
		setJeffJeffing();
	}

	private void setLauncherPower()
	{
		launcher.setPower(power * maxPower);
	}

	private void setTriggerPosition()
	{
		trigger.setPosition(hit ? 0d : 0.52d);
	}

	private void setJeffJeffing()
	{
		jeff.setPosition(Mathf.lerp(0.4f, 0f, jeffing));
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
