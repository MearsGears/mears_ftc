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

public class Ticus_MearsK9TeleOpServos extends OpMode {

    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    DcMotor leftPulley;
    DcMotor rightPulley;
    DcMotor leftEx;
    DcMotor rightEx;
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo leftS;
    Servo rightS;


    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        leftS = hardwareMap.servo.get("leftS");
        rightS = hardwareMap.servo.get("rightS");
        leftEx = hardwareMap.dcMotor.get("leftE");
        rightEx = hardwareMap.dcMotor.get("rightE");
        leftPulley = hardwareMap.dcMotor.get("LPulley");
        rightPulley = hardwareMap.dcMotor.get("RPulley");


        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }


    @Override
    public void loop() {


        float right = gamepad2.right_stick_y;           //added
        float left = gamepad2.right_stick_y;           //added




        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        float rightP = gamepad2.left_stick_y;
        float leftP = gamepad2.left_stick_y;

       // float rightX = gamepad2.right_stick_y;
        //float leftX = gamepad2.right_stick_y;         took out


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);           //added
        left =  (float)scaleInput(left);            //added

        leftP = (float)scaleInput(leftP);
        rightP = (float)scaleInput(rightP);

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        rightEx.setPower(right);
        leftEx.setPower(left);

        leftPulley.setPower(leftP);
        rightPulley.setPower(rightP);

        if (gamepad2.left_bumper) {
            leftS.setPosition(LEFT_OPEN_POSITION);
            rightS.setPosition(RIGHT_OPEN_POSITION);
        }
        if(gamepad2.right_bumper) {
            leftS.setPosition(LEFT_CLOSED_POSITION);
            rightS.setPosition(RIGHT_CLOSED_POSITION);
        }
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
        double[] scaleArray = {  0.0, 0.01, 0.02, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.14, 0.15, 0.18, 0.21, 0.22, 0.25, 0.28,
                0.30, 0.36, 0.43, 0.47, 0.50, 0.55, 0.60, 0.66, 0.72, 0.85, 1.00 };

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







