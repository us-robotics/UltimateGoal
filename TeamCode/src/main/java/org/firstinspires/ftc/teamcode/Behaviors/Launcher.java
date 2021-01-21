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

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT);

		setLauncherPower();
		setTriggerPosition();
		setJeffJeffing();
	}

	private DcMotor launcher;
	private Servo trigger;
	private Servo jeff;

	private boolean primed;
	private boolean hit;
	private float jeffing;

	private static final float HIGH_POWER = 0.735f; //Power for high goal
	private static final float SHOT_POWER = 0.695f; //Power for power shots

	private float power = HIGH_POWER;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.getIsAuto())
		{
			Input input = opMode.getHelper(Input.class);

			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER)) primed = !primed;

			hit = input.getButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);
			jeffing = input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.LEFT_TRIGGER);

			final float POWER_CHANGE_RATE = 0.0025f;

			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP)) power += POWER_CHANGE_RATE;
			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN)) power -= POWER_CHANGE_RATE;

			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT)) power = HIGH_POWER;
			if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT)) power -= SHOT_POWER;

			opMode.getHelper(Telemetry.class).addData("Launcher Power", power);
		}

		setLauncherPower();
		setTriggerPosition();
		setJeffJeffing();
	}

	private void setLauncherPower()
	{
		launcher.setPower(primed ? power : 0d);
	}

	private void setTriggerPosition()
	{
		trigger.setPosition(hit ? 0d : 0.52d);
	}

	private void setJeffJeffing()
	{
		jeff.setPosition(Mathf.lerp(0.4f, 0f, jeffing));
	}

	public void setPrimed(boolean primed)
	{
		this.primed = primed;
	}

	public void setHit(boolean hit)
	{
		this.hit = hit;
	}
}
