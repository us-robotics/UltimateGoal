package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

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

		apply();

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.X);
	}

	private DcMotor arm;
	private Servo grabber;
	private TouchSensor touch;

	private boolean releasing;
	private boolean resetting = true;

	private static final float RESET_POWER = -0.38f;

	@Override
	public void update()
	{
		super.update();

		releasing = opMode.input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.RIGHT_TRIGGER) > 0.1f;

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
				arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				setResetting(false);
			}
			else
			{

			}

			arm.setPower(RESET_POWER);
			arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		else
		{

		}
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
		IDLE,
		HIGH
	}
}
