package org.firstinspires.ftc.teamcode.Subsystems.Utils.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
        public RevBlinkinLedDriver blinkin;
        private RevBlinkinLedDriver.BlinkinPattern pattern;

        private Telemetry.Item patternName;
        private Telemetry.Item display;
        private LEDTest.DisplayKind displayKind;

        /**
         * constructor for LED
         * @param hardwareMap
         * @param name
         */
        public LED (HardwareMap hardwareMap, String name){
            displayKind = LEDTest.DisplayKind.MANUAL; //allows us to switch LEDs from code

            blinkin = hardwareMap.get(RevBlinkinLedDriver.class, name);
            blueYellow();
            blinkin.setPattern(pattern);
        }

        /**
         * auto or manual
         * @return
         */
        public String getDisplayKind(){
            return displayKind.toString();
        }
        /**
         * returns pattern name
         * @return
         */
        public String getPatternName(){
            return pattern.toString();
        }

        /**
         * sets LED pattern
         * @param pattern
         */
        public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
            blinkin.setPattern(pattern);
        }

        /**
         * makes LEDs red
         */
        public void red(){
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            setPattern(pattern);
        }

        /**
         * makes LEDs yellow
         */
        public void yellow(){
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            setPattern(pattern);
        }

        /**
         * makes LEDs green
         */
        public void green(){
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            setPattern(pattern);
        }

        /**
         * makes LEDs blue
         */
        public void blue(){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            setPattern(pattern);
        }

        /**
         * turns LEDs off
         */
        public void black(){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            setPattern(pattern);
        }

        /**
         * makes LEDs and flashing blue-yellow
         */
        public void blueYellow(){
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON;
            setPattern(pattern);
        }
}
