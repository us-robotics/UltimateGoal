package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

public class Drivetrain extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 */
	public Drivetrain(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		frontRight = hardwareMap.dcMotor.get("frontRight");
		frontLeft = hardwareMap.dcMotor.get("frontLeft");
		backRight = hardwareMap.dcMotor.get("backRight");
		backLeft = hardwareMap.dcMotor.get("backLeft");

		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		backRight.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = opMode.getBehavior(InertialMeasurementUnit.class);

		resetMotorPositions();
		setRawVelocities(Vector2.zero, 0f);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_1, Input.Button.X);
	}

	private DcMotor frontRight;
	private DcMotor frontLeft;
	private DcMotor backRight;
	private DcMotor backLeft;

	private InertialMeasurementUnit imu;
	private float targetAngle;

	private final float[] powers = new float[4];

	private Vector2 positionalInput = Vector2.zero;
	private float rotationalInput = 0f;

	@Override
	public void start()
	{
		super.start();

		if (imu == null) return;
		targetAngle = getAngle();
	}

	@Override
	public void update()
	{
		super.update();
		Input input = opMode.getHelper(Input.class);

		if (!getIsAuto())
		{
			//Process input if is not in auto
			positionalInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.LEFT_JOYSTICK);
			rotationalInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.RIGHT_JOYSTICK).x;

			//Process input for smoother control by interpolating a polynomial curve
			final float exponent = 0.72f;

			positionalInput = positionalInput.normalize().mul((float)Math.pow(positionalInput.getMagnitude(), exponent));
			rotationalInput = Mathf.normalize(rotationalInput) * (float)Math.pow(Math.abs(rotationalInput), exponent) * 0.72f;
		}

		//If no rotational input, then IMU is used to counterbalance hardware inaccuracy to drive straight
		if (imu == null) setRawVelocities(positionalInput, rotationalInput);
		else
		{
			if (!Mathf.almostEquals(rotationalInput, 0f) || positionalInput.equals(Vector2.zero))
			{
				targetAngle = getAngle();
				setRawVelocities(positionalInput, rotationalInput);
			}
			else setRawVelocities(positionalInput, Mathf.toSignedAngle(getAngle() - targetAngle) / 25f);
		}

		if (input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.X)) resetMotorPositions();
//		opMode.getHelper(Telemetry.class).addData("Motor Average", getAveragePosition());
	}

	private void setRawVelocities(Vector2 localDirection, float angularDelta)
	{
		localDirection = localDirection.clampMagnitude(0f, 1f);
		angularDelta = Mathf.clamp(angularDelta, -1f, 1f);

		powers[0] = localDirection.y - localDirection.x - angularDelta;
		powers[1] = localDirection.y + localDirection.x + angularDelta;
		powers[2] = localDirection.y + localDirection.x - angularDelta;
		powers[3] = localDirection.y - localDirection.x + angularDelta;

		float max = -Float.MAX_VALUE;

		for (int i = 0; i < powers.length; i++)
		{
			float power = Math.abs(powers[i]);

			if (power < 0.05f) powers[i] = 0f;
			else max = Math.max(max, power);
		}

		if (max > 1f)
		{
			//Scales all powers down by a multiplier if one power is higher than 1
			for (int i = 0; i < powers.length; i++) powers[i] /= max;
		}

		frontRight.setPower(powers[0]);
		frontLeft.setPower(powers[1]);
		backRight.setPower(powers[2]);
		backLeft.setPower(powers[3]);

//		opMode.getHelper(Telemetry.class).addData("Powers", Arrays.toString(powers));
	}

	public void setDirectInputs(Vector2 positionalInput, float rotationalInput)
	{
		this.positionalInput = positionalInput;
		this.rotationalInput = rotationalInput;
	}

	public void resetMotorPositions()
	{
		setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	private void setMotorModes(DcMotor.RunMode mode)
	{
		frontRight.setMode(mode);
		frontLeft.setMode(mode);
		backRight.setMode(mode);
		backLeft.setMode(mode);
	}

	public void setTargetAngle(float targetAngle)
	{
		this.targetAngle = targetAngle;
	}

	public float getAveragePosition()
	{
		return (Math.abs(frontRight.getCurrentPosition()) + Math.abs(frontLeft.getCurrentPosition()) +
		        Math.abs(backRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition())) / 4f;
	}

	public float getAngle()
	{
		return Mathf.toSignedAngle(imu.getAngles().z);
	}
}
