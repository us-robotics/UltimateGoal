package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;
import FTCEngine.Core.Time;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

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

		arm.setPower(0d);
		arm.setDirection(DcMotorSimple.Direction.REVERSE);
		grabber.setPosition(0);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.X);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.Y);

		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	private DcMotor arm;
	private Servo grabber;
	private TouchSensor touch;

	public void update()
	{
		super.update();
		Input input = opMode.getHelper(Input.class);

		final float MaxPower = 0.38f;
		final int GrabTargetPosition = 335;

		boolean positionToggle = input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.X);

		if (arm.isBusy())
		{
			if (positionToggle) arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			arm.setPower(MaxPower);
		}
		else
		{
			float armInput = input.getVector(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK).y;

			//Process input for smoother/better control
			int direction = Mathf.normalize(armInput);
			armInput = (float)Math.sin(Math.abs(armInput) * Math.PI - Math.PI / 2d) / 2f + 0.5f;

			if (positionToggle)
			{
				arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				arm.setTargetPosition(GrabTargetPosition);
			}
			else arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			if (touch.getValue() > 0.2d && direction >= 0) arm.setPower(MaxPower);
			else arm.setPower(armInput * direction * MaxPower);
		}

		grabber.setPosition(input.getButton(Input.Source.CONTROLLER_2, Input.Button.Y) ? 0.45f : 0f);
		opMode.getHelper(Telemetry.class).addData("Wobble Arm Position", arm.getCurrentPosition());
	}
}
