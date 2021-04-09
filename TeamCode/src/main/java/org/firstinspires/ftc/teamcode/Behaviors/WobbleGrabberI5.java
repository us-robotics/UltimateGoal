package org.firstinspires.ftc.teamcode.Behaviors;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class WobbleGrabberI5 extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public WobbleGrabberI5(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		arm = hardwareMap.dcMotor.get("arm");
		grabber = hardwareMap.servo.get("grabber");
		potentiometer = hardwareMap.analogInput.get("angle");

		arm.setDirection(DcMotorSimple.Direction.REVERSE);
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private DcMotor arm;
	private Servo grabber;
	private AnalogInput potentiometer;

	private Mode mode = Mode.GRAB;
	private boolean isReleased;

	private float currentP;
	private float currentI;
	private float currentD;

	public float constantP = 4.5f;
	public float constantI = 3.7f;
	public float constantD = 0.1f;

	private float previousError;

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
				boolean x = Math.abs(direction.x) > Math.abs(direction.y);

				if (x) mode = direction.x > 0 ? Mode.HIGH : Mode.DROP;
				else mode = direction.y > 0f ? Mode.FOLD : Mode.GRAB;
			}
		}

		apply();
	}

	private void apply()
	{
		float targetAngle = 0f;

		switch (mode)
		{
			case FOLD:
			{
				targetAngle = 35f;
				break;
			}
			case HIGH:
			{
				targetAngle = 132f;
				break;
			}
			case DROP:
			{
				targetAngle = 174f;
				break;
			}
			case GRAB:
			{
				targetAngle = 227f;
				break;
			}
		}

		float currentError = targetAngle - getAngle();
		float power = updatePID(currentError, opMode.time.getDeltaTime());

		arm.setPower(power * 0.001f);
		grabber.setPosition(isReleased ? 1f : 0f);

		opMode.telemetry.addData("angle", getAngle());
		opMode.telemetry.addData("target", targetAngle);
		opMode.telemetry.addData("power", power);
		opMode.telemetry.addData("p", currentP);
		opMode.telemetry.addData("i", currentI);
		opMode.telemetry.addData("d", currentD);
	}

	private float updatePID(float currentError, float deltaTime)
	{
		currentP = currentError;
		currentI += currentP * deltaTime;
		currentD = (currentP - previousError) / deltaTime;

		previousError = currentError;

		return currentP * constantP + currentI * constantI + currentD * constantD;
	}

	private float getAngle()
	{
		double voltage = potentiometer.getVoltage();
		double x = 2700d * voltage + 4455d - Math.sqrt(21.87e6 * voltage * voltage - 24.057e6 * voltage + 19847025);
		return (float)(x / (20d * voltage)); //https://www.desmos.com/calculator/wswiucb3hc
	}

	public enum Mode
	{
		FOLD,
		HIGH,
		DROP,
		GRAB
	}
}
