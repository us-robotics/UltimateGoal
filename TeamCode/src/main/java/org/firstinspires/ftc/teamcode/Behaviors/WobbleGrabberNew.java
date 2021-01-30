package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

public class WobbleGrabberNew extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public WobbleGrabberNew(OpModeBase opMode)
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

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.X);
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

	private Position targetPosition;
	private boolean releasing;
	private boolean resetting = true;

	private static final float RESET_POWER = -0.38f;

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
				targetPosition = direction.y > 0f ? Position.HIGH : Position.FOLD;
			}
		}

		apply();
	}

	private void apply()
	{
		grabber.setPosition(releasing ? 0.45f : 0f);

		if (resetting)
		{
			if (touch.isPressed())
			{
				arm.setPower(0d);
				arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				resetting = false;
			}
			else
			{
				arm.setPower(RESET_POWER);
				arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			}
		}
		else
		{
			int position;

			switch (targetPosition)
			{
				case FOLD:
				{
					position = 0;
					break;
				}
				case GRAB:
				{
					position = 335;
					break;
				}
				case HIGH:
				{
					position = 500;
					break;
				}
				default:
				{
					throw new IllegalArgumentException(targetPosition.name());
				}
			}

			arm.setPower(1d);
			arm.setTargetPosition(position);
			arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
	}

	public void setTargetPosition(Position targetPosition)
	{
		this.targetPosition = targetPosition;
	}

	public void setReleasing(boolean releasing)
	{
		this.releasing = releasing;
	}

	public void setResetting(boolean resetting)
	{
		this.resetting = resetting;
	}

	public boolean isResetting()
	{
		return resetting;
	}

	public enum Position
	{
		FOLD,
		GRAB,
		HIGH
	}
}
