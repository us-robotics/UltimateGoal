package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class WobbleGrabber extends AutoBehavior<WobbleGrabber.Job>
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public WobbleGrabber(OpModeBase opMode)
	{
		super(opMode);
	}

	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		arm = hardwareMap.dcMotor.get("arm");
		grabber = hardwareMap.servo.get("grabber");
		touch = hardwareMap.touchSensor.get("touch");

		arm.setDirection(DcMotorSimple.Direction.REVERSE);
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	@Override
	public void awakeUpdate()
	{
		super.awakeUpdate();
		apply();
	}

	private DcMotor arm;
	private Servo grabber;
	private TouchSensor touch;

	private Mode targetMode = Mode.FOLD;
	private boolean isReleased;

	private static final float MOVE_POWER = 0.38f;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			isReleased = opMode.input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.RIGHT_TRIGGER) > 0.1f;

			float magnitude = opMode.input.getMagnitude(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK);
			Vector2 direction = opMode.input.getDirection(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK);

			if (magnitude > 0.1f)
			{
				if (Math.abs(direction.x) > Math.abs(direction.y)) targetMode = Mode.HIGH;
				else targetMode = direction.y > 0f ? Mode.RESET : Mode.GRAB;
			}
		}

		apply();
	}

	@Override
	protected void updateJob()
	{
		WobbleGrabber.Job job = getCurrentJob();

		if (job instanceof WobbleGrabber.Move)
		{
			WobbleGrabber.Move move = (WobbleGrabber.Move)job;

			targetMode = move.mode;
			apply(); //We need to actually set the target to check if the motor is busy or not

			if (move.mode != Mode.RESET && !arm.isBusy()) move.finishJob();
		}

		if (job instanceof WobbleGrabber.Grab)
		{
			WobbleGrabber.Grab grab = (WobbleGrabber.Grab)job;

			isReleased = !grab.grab;
			grab.finishJob();
		}
	}

	private void apply()
	{
		if (targetMode == Mode.FOLD)
		{
			if (touch.isPressed())
			{
				arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				arm.setPower(0f);

				targetMode = Mode.FOLD;
			}
			else
			{
				arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				arm.setPower(MOVE_POWER);
			}

			isReleased = false;
		}

		grabber.setPosition(isReleased ? 1f : 0f);

		if (targetMode)
		{

		}
		else
		{
			if (targetMode == Mode.FOLD)
			{

			}
			else
			{
				int position = 0;

				switch (targetMode)
				{
					case GRAB:
					{
						position = -760;
						break;
					}
					case HIGH:
					{
						position = -400;
						break;
					}
				}

				arm.setTargetPosition(position);
				arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				arm.setPower(MOVE_POWER);
			}
		}
	}

	public enum Mode
	{
		FOLD,
		GRAB,
		HIGH,
		UP,
		DOWN
	}

	abstract static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Move extends WobbleGrabber.Job
	{
		public Move(Mode mode)
		{
			this.mode = mode;
		}

		public final Mode mode;
	}

	public static class Grab extends WobbleGrabber.Job
	{
		public Grab(boolean grab)
		{
			this.grab = grab;
		}

		public final boolean grab;
	}
}
