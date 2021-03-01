package org.firstinspires.ftc.teamcode.Behaviors;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
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

	private Mode mode = Mode.FOLD;
	private boolean isReleased;

	private static final float MOVE_POWER = 0.43f;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			isReleased = opMode.input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.RIGHT_TRIGGER) > 0.4f;

			float magnitude = opMode.input.getMagnitude(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK);
			Vector2 direction = opMode.input.getDirection(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK);

			if (magnitude > 0.3f)
			{
				if (Math.abs(direction.x) > Math.abs(direction.y)) mode = direction.x > 0f ? Mode.GRAB : Mode.HIGH;
				else mode = direction.y > 0f ? Mode.FOLD : Mode.DOWN;
			}
			else if (mode == Mode.FOLD || mode == Mode.DOWN) mode = Mode.IDLE;
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

			mode = move.mode;
			apply(); //We need to actually set the target to check if the motor is busy or not

			if (move.mode == Mode.DOWN || move.mode == Mode.IDLE) move.finishJob();
			else if (move.mode == Mode.FOLD && Mathf.almostEquals((float)arm.getPower(), 0f)) move.finishJob();
			else if (Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) < 25f) move.finishJob();
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
		switch (mode)
		{
			case FOLD:
			{
				if (touch.isPressed())
				{
					arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					arm.setPower(0f);
				}
				else
				{
					arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
					arm.setPower(MOVE_POWER);
				}

				break;
			}
			case GRAB:
			{
				arm.setTargetPosition(-740);
				arm.setPower(MOVE_POWER);
				arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				break;
			}
			case HIGH:
			{
				arm.setTargetPosition(-360);
				arm.setPower(MOVE_POWER);
				arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				break;
			}
			case DOWN:
			{
				arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				arm.setPower(-MOVE_POWER);
				break;
			}
			case IDLE:
			{
				arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				arm.setPower(0f);
				break;
			}
		}

		grabber.setPosition(isReleased ? 1f : 0f);
	}

	public enum Mode
	{
		FOLD,
		GRAB,
		HIGH,
		DOWN,
		IDLE
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
