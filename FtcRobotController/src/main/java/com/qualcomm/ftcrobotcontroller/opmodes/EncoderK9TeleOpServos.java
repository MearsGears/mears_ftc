/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


public class EncoderK9TeleOpServos extends OpMode {
    // creating the variables  and formulas to use the encoders
	final static int ENCODER_CPR = 1440;
	final static double GEAR_RATIO = 2;
	final static int WHEEL_DIAMETER = 4;
	final static int DISTANCE = 180;


	final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
	final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
	final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;


	DcMotor motorRight;
	DcMotor motorLeft;
	DcMotor arm_motor;
    DcMotor arm_extend;
	Servo claw;
	Servo armservo;
	/**
	 * Constructor
	 */
	public EncoderK9TeleOpServos() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "left_drive" and "right_drive"
		 *   "left_drive" is on the right side of the bot.
		 *   "right_drive" is on the left side of the bot and reversed.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the armservo joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		arm_motor = hardwareMap.dcMotor.get("left_arm");
		motorRight = hardwareMap.dcMotor.get("right_drive");
		motorLeft = hardwareMap.dcMotor.get("left_drive");
		motorRight.setDirection(DcMotor.Direction.REVERSE);
		arm_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_extend = hardwareMap.dcMotor.get("arm_drive");

		armservo = hardwareMap.servo.get("servo_1");
		claw = hardwareMap.servo.get("servo_6");



        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

	}
	/*
         * Code to run when the op mode is first disabled goes here
         *
         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
         */
	@Override
	public void start() {
		motorLeft.setTargetPosition((int) COUNTS);
		motorRight.setTargetPosition((int) COUNTS);

		motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

		motorLeft.setPower(1);
		motorRight.setPower(1);

	





	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */


	@Override
	public void loop() {




		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */


        telemetry.addData("Motor Target", COUNTS);
        telemetry.addData("Left Position", motorLeft.getCurrentPosition());
        telemetry.addData("Right Position", motorRight.getCurrentPosition());


	}





	@Override
	public void stop() {

	}

    	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.009, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.12, 0.14, 0.15, 0.18, 0.19, 0.21, 0.22, 0.24, 0.25, 0.28,
				0.30, 0.36, 0.43, 0.47, 0.50, 0.55, 0.60, 0.66, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		
		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}


		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
