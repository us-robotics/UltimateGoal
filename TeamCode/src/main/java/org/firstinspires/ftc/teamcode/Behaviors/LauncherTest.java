package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

public class LauncherTest extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public LauncherTest(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_1, Input.Button.X);

		launcher = hardwareMap.dcMotor.get("launcher");
		launcher.setPower(0f);
	}

	DcMotor launcher;

	@Override
	public void update()
	{
		super.update();

		float power = opMode.getHelper(Input.class).getButton(Input.Source.CONTROLLER_1, Input.Button.X) ? -1f : 1f;
		launcher.setPower(power);
	}
}
