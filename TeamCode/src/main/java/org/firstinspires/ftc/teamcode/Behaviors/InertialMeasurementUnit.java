package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector3;

public class InertialMeasurementUnit extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public InertialMeasurementUnit(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

		imu.initialize(parameters);
		initialAngles = getAnglesRaw();
	}

	private BNO055IMU imu;
	private Vector3 initialAngles;

	public Vector3 getAngles()
	{
		return getAnglesRaw().sub(initialAngles);
	}

	private Vector3 getAnglesRaw()
	{
		Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
		return new Vector3(orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
	}
}
