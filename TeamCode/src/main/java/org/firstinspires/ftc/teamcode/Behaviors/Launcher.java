package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Debug;
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

		launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT);

		apply();
	}

	private DcMotor launcher;
	private Servo trigger;

	public static final float HIGH_POWER = 0.7325f; //Power for high goal
	public static final float SHOT_POWER = 0.6975f; //Power for power shots

	private float power = HIGH_POWER;

	private boolean primed;
	private boolean hit;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER)) primed = !primed;
			hit = opMode.input.getButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

			final float POWER_CHANGE_RATE = 0.0025f;

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP)) power += POWER_CHANGE_RATE;
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN)) power -= POWER_CHANGE_RATE;

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT)) power = HIGH_POWER;
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT)) power = SHOT_POWER;

			opMode.debug.addData("Launcher Power", power);
		}

		apply();
	}

	private void apply()
	{
		launcher.setPower(primed ? power : 0d);
		trigger.setPosition(hit ? 0d : 0.52d);
	}

	public void setPower(float power)
	{
		this.power = power;
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
