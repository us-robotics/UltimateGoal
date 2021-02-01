package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Autos.DrivetrainAuto;

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

	private Position targetPosition = Position.FOLD;
	private boolean releasing;

	private static final float RESET_POWER = -0.32f;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			releasing = opMode.input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.RIGHT_TRIGGER) > 0.1f;

			float magnitude = opMode.input.getMagnitude(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK);
			Vector2 direction = opMode.input.getDirection(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK);

			if (magnitude > 0.5f)
			{
				if (Math.abs(direction.x) > Math.abs(direction.y)) targetPosition = Position.GRAB;
				else targetPosition = direction.y > 0f ? Position.HIGH : Position.FOLD;
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
		}

		if (job instanceof WobbleGrabber.Grab)
		{
			WobbleGrabber.Grab move = (WobbleGrabber.Grab)job;
		}
	}

	private void apply()
	{
		grabber.setPosition(releasing ? 0.45f : 0f);

		if (targetPosition == Position.FOLD)
		{
			if (touch.isPressed())
			{
				arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				arm.setPower(RESET_POWER / 3f);
			}
			else
			{
				arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				arm.setPower(RESET_POWER);
			}
		}
		else
		{
			int position;

			switch (targetPosition)
			{
				case GRAB:
				{
					position = 335;
					break;
				}
				case HIGH:
				{
					position = 620;
					break;
				}
				default:
				{
					throw new IllegalArgumentException(targetPosition.toString());
				}
			}

			arm.setPower(0.65d);
			arm.setTargetPosition(position);
			arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
	}

	public enum Position
	{
		FOLD,
		GRAB,
		HIGH
	}

	abstract static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Move extends WobbleGrabber.Job
	{
		public Move(Position position)
		{
			this.position = position;
		}

		public final Position position;
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
