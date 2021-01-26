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

public class WobbleGrabber extends Behavior
{
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
		arm.setPower(-MAX_POWER);
		grabber.setPosition(0);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.X);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.Y);

		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private DcMotor arm;
	private Servo grabber;
	private TouchSensor touch;

	private boolean released;
	private int pushing; //Zero = no pushing, one = pushing up, negative one = pushing down

	final float MAX_POWER = 0.38f;
	final int GRAB_TARGET_POSITION = 335;

	@Override
	public void start()
	{
		super.start();
		resetEncoder();
		arm.setPower(0f);
	}

	public void update()
	{
		super.update();

		if (opMode.hasSequence())
		{
			final float PushDownPower = 0.1f;

			if (pushing > 0) arm.setPower(MAX_POWER);
			else if (pushing < 0) arm.setPower(-PushDownPower);
			else arm.setPower(0f);
		}
		else
		{
			boolean positionToggle = opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.X);

			if (arm.isBusy())
			{
				if (positionToggle) arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				arm.setPower(MAX_POWER);
			}
			else
			{
				float armInput = opMode.input.getVector(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK).y;

				//Process input for smoother/better control
				int direction = Mathf.normalize(armInput);
				armInput = (float)Math.sin(Math.abs(armInput) * Math.PI - Math.PI / 2d) / 2f + 0.5f;

				if (positionToggle)
				{
					arm.setTargetPosition(GRAB_TARGET_POSITION);
					arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				}
				else arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

				if (touch.getValue() > 0.2d && direction >= 0) arm.setPower(MAX_POWER);
				else arm.setPower(armInput * direction * MAX_POWER);
			}

			setReleased(opMode.input.getButton(Input.Source.CONTROLLER_2, Input.Button.Y));
			if (opMode.input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.RIGHT_TRIGGER) > 0.2f) resetEncoder();
		}

		grabber.setPosition(released ? 0.45f : 0f);
		//opMode.getHelper(Telemetry.class).addData("Wobble Arm Position", arm.getCurrentPosition());
	}

	public void setReleased(boolean released)
	{
		this.released = released;
	}

	public void clearPush()
	{
		pushing = 0;
	}

	public void setPushUp()
	{
		pushing = 1;
	}

	public void setPushDown()
	{
		pushing = -1;
	}

	public void resetEncoder()
	{
		arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
}
