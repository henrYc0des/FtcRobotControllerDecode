/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Just Leave Short Bro", group="Robot")

public class JustLeaveShortBro extends LinearOpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    private DcMotor bottomShooter = null;
    private DcMotor topShooter = null;

    DcMotor inhale;
    DcMotor indixer;

    public Servo servo67;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;



    public ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        telemetry.addLine("▒▒▒▒▒▄██████████▄▒▒▒▒▒");
        telemetry.addLine("▒▒▒▄██████████████▄▒▒▒");
        telemetry.addLine("▒▒██████████████████▒▒");
        telemetry.addLine("▒▐███▀▀▀▀▀██▀▀▀▀▀███▌▒");
        telemetry.addLine("▒███▒▒▌■▐▒▒▒▒▌■▐▒▒███▒");
        telemetry.addLine("▒▐██▄▒▀▀▀▒▒▒▒▀▀▀▒▄██▌▒");
        telemetry.addLine("▒▒▀████▒▄▄▒▒▄▄▒████▀▒▒");
        telemetry.addLine("▒▒▐███▒▒▒▀▒▒▀▒▒▒███▌▒▒");
        telemetry.addLine("▒▒███▒▒▒▒▒▒▒▒▒▒▒▒███▒▒");
        telemetry.addLine("▒▒▒██▒▒▀▀▀▀▀▀▀▀▒▒██▒▒▒");
        telemetry.addLine("▒▒▒▐██▄▒▒▒▒▒▒▒▒▄██▌▒▒▒");
        telemetry.addLine("▒▒▒▒▀████████████▀▒▒▒▒");

        // Initialize the drive system variables.
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        bottomShooter = hardwareMap.get(DcMotor.class, "bottom_shooter");
        topShooter = hardwareMap.get(DcMotor.class, "top_shooter");
        indixer = hardwareMap.get(DcMotor.class, "indixer");
        inhale = hardwareMap.get(DcMotor.class, "inhale");
        servo67 = hardwareMap.get(Servo.class, "servo67");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        topShooter.setDirection(DcMotor.Direction.REVERSE);
        bottomShooter.setDirection(DcMotor.Direction.REVERSE);
        indixer.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward
        frontLeftDrive.setPower(-1);
         backLeftDrive.setPower(-1);
        frontRightDrive.setPower(-1);
        backRightDrive.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        frontLeftDrive.setPower(1);
        backLeftDrive.setPower(1);
        frontRightDrive.setPower(-1);
        backRightDrive.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive forward for 1 Second
        frontLeftDrive.setPower(1);
        backLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        backRightDrive.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
