package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;
import FTCEngine.Math.Vector3;

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

		frontLeft = hardwareMap.dcMotor.get("frontLeft");
		frontRight = hardwareMap.dcMotor.get("frontRight");
		backLeft = hardwareMap.dcMotor.get("backLeft");
		backRight = hardwareMap.dcMotor.get("backRight");

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
			final float exponent = 2.2f; //Can use a higher exponent power if more precision is needed

			positionalInput = positionalInput.normalize().mul((float)Math.pow(positionalInput.getMagnitude(), exponent));
			rotationalInput = Mathf.normalize(rotationalInput) * (float)Math.pow(Math.abs(rotationalInput), exponent);
		}

		//If no rotational input, then IMU is used to counterbalance hardware inaccuracy to drive straight
		if (imu != null && Mathf.almostEquals(rotationalInput, 0f))
		{
			float deviation = Mathf.toSignedAngle(getAngle() - targetAngle) / 45f;
			setRawVelocities(positionalInput, deviation);
		}
		else
		{
			setRawVelocities(positionalInput, rotationalInput);
			if (imu != null) targetAngle = getAngle();
		}

		if (input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.X)) resetMotorPositions();
		//opMode.getHelper(Telemetry.class).addData("Motor Average", averagePosition());
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
			max = Math.max(max, power);
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

	public float averagePosition()
	{
		return (Math.abs(frontRight.getCurrentPosition()) + Math.abs(frontLeft.getCurrentPosition()) +
		        Math.abs(backRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition())) / 4f;
	}

	private float getAngle()
	{
		return Mathf.toSignedAngle(imu.getAngles().z);
	}
}
