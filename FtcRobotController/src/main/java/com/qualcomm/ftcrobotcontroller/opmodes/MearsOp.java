package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;




/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class MearsOp extends OpMode {


    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
// TETRIX VALUES.
    final static double ARMSERV_MIN_RANGE  = 0.20;
    final static double ARMSERV_MAX_RANGE  = 0.90;
    final static double CLAW_MIN_RANGE  = 0.20;
    final static double CLAW_MAX_RANGE  = 0.7;

    // position of the arm servo.
    double armservPosition;

    // amount to change the arm servo position.
    double armservDelta = 0.1;

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.1;


    DcMotor right_motor;
    DcMotor left_motor;
    DcMotor arm_motor;
    Servo claw;
    Servo armserv;

    /**
     * Constructor
     */
    public MearsOp() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        arm_motor = hardwareMap.dcMotor.get("arm");
        right_motor = hardwareMap.dcMotor.get("right_drive");
        left_motor = hardwareMap.dcMotor.get("left_drive");
        armserv = hardwareMap.servo.get("servo_1");
        claw = hardwareMap.servo.get("servo_6");

        left_motor.setDirection(DcMotor.Direction.REVERSE);
        //right_motor.setDirection(DcMotor.Direction.REVERSE);

        // assign the starting position of the wrist and claw
        armservPosition = 0.2;
        clawPosition = 0.2;

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */

    @Override
    public void loop()
    {



        // float throttle = -gamepad1.right_stick_y; //this is for 1joy
        //float direction = gamepad1.right_stick_x; //this is for 1joy
        //float throttle = -Test;
        //float direction = Test;
        float left = gamepad1.right_stick_y;
        float right = gamepad1.left_stick_y;
        float arm = -gamepad2.right_stick_y;


        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        arm = Range.clip(arm, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);
        arm = (float) scaleInput(arm);
        // write the values to the motors
        right_motor.setPower(right);
        left_motor.setPower(left);
        arm_motor.setPower(arm);


 // get the corresponding index for the scaleInput array.
       // update the position of the arm.
		if (gamepad2.a) {
			// if the A button is pushed on gamepad2, increment the position of
			// the arm servo.
			armservPosition += armservDelta;
		}

		if (gamepad2.y) {
			// if the Y button is pushed on gamepad2, decrease the position of
			// the arm servo.
			armservPosition -= armservDelta;
		}

		// update the position of the claw
		if (gamepad2.x) {
			clawPosition += clawDelta;
		}

		if (gamepad2.b) {
			clawPosition -= clawDelta;
		}

        // clip the position values so that they never exceed their allowed range.
        armservPosition = Range.clip(armservPosition, ARMSERV_MIN_RANGE, ARMSERV_MAX_RANGE);
        clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
		armserv.setPosition(armservPosition);
		claw.setPosition(clawPosition);

 int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}

