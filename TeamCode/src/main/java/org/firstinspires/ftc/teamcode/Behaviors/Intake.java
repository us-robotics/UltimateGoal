package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

public class Intake extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public Intake(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		intake = hardwareMap.dcMotor.get("intake");
		apply();
	}

	private DcMotor intake;
	private float power;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			setPower(opMode.input.getVector(Input.Source.CONTROLLER_2, Input.Button.RIGHT_JOYSTICK).y);
		}

		apply();
	}

	public void setPower(float power)
	{
		this.power = power;
	}

	private void apply()
	{
		intake.setPower(power);
	}
}
