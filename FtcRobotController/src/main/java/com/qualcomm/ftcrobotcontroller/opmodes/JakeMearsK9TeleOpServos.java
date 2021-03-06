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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**				--Jake!!!! JAKE!!!! JAKE!!!
 CREATE ANOTHER JAKE FILE CALLED : JakeK9AUtoTime.....and adapt it to our program...so we have an autonomous program
 MrsM think you can do it!!!!!
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class JakeMearsK9TeleOpServos extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the armservo servo approaches 0, the armservo position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.
	final static double armservo_MIN_RANGE  = 0.20;
	final static double armservo_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.90;

	// position of the armservo servo.

    double armservoPosition;

	// amount to change the armservo servo position.
	double armservoDelta = 0.2;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.2;

	DcMotor motorRight;
	DcMotor motorLeft;
	DcMotor leftArm;
    DcMotor arm_extend;
	Servo claw;
	Servo armservo;
	/**
	 * Constructor
	 */
	public JakeMearsK9TeleOpServos() {

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
		leftArm = hardwareMap.dcMotor.get("left_arm");
		motorRight = hardwareMap.dcMotor.get("right_drive");
		motorLeft = hardwareMap.dcMotor.get("left_drive");
		motorRight.setDirection(DcMotor.Direction.REVERSE);
		leftArm.setDirection(DcMotor.Direction.REVERSE);
        arm_extend = hardwareMap.dcMotor.get("arm_drive");
		arm_extend.setDirection(DcMotor.Direction.REVERSE);

		armservo = hardwareMap.servo.get("servo_1");
		claw = hardwareMap.servo.get("servo_6");

		// assign the starting position of the wrist and claw
		armservoPosition = 0.2;
		clawPosition = 0.2;

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		float left_drive = -gamepad1.left_stick_y;
		float right_drive = -gamepad1.right_stick_y;
		float left_arm = gamepad2.right_stick_y;
        float arm_drive = gamepad2.left_stick_y;


		// clip the right/left values so that the values never exceed +/- 1
		right_drive = Range.clip(right_drive, -1, 1);
		left_drive = Range.clip(left_drive, -1, 1);
		left_arm = Range.clip(left_arm, -1, 1);
        arm_drive = Range.clip(arm_drive, -1, 1);
		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right_drive = (float)scaleInput(right_drive);
		left_drive =  (float)scaleInput(left_drive);
		
		// write the values to the motors
		motorRight.setPower(right_drive);
		motorLeft.setPower(left_drive);
        leftArm.setPower(left_arm);
		arm_extend.setPower(arm_drive);


		// update the position of the armservo.
		if (gamepad2.b) {
			// if the A button is pushed on gamepad1, increment the position of
			// the armservo servo.
			armservoPosition += armservoDelta;
		}

		if (gamepad2.x) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the armservo servo.
			armservoPosition -= armservoDelta;
		}

		// update the position of the claw
		if (gamepad2.x) {
			clawPosition += clawDelta;
		}

		if (gamepad2.b) {
			clawPosition -= clawDelta;
		}

        // clip the position values so that they never exceed their allowed range. hello
        armservoPosition = Range.clip(armservoPosition, armservo_MIN_RANGE, armservo_MAX_RANGE);
        clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
		armservo.setPosition(armservoPosition);
		claw.setPosition(clawPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("armservo", "armservo:  " + String.format("%.2f", armservoPosition));
        telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left_drive tgt pwr",  "left_drive  pwr: " + String.format("%.2f", left_drive));
        telemetry.addData("right_drive tgt pwr", "right_drive pwr: " + String.format("%.2f", right_drive));

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
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
				0.30, 0.36, 0.43, 0.47, 0.50, 0.55, 0.60, 0.66, 0.72, 0.85, 1.00, 2.00, 3.00 };
		
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
