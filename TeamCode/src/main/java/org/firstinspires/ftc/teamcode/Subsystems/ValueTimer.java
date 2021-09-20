package org.firstinspires.ftc.teamcode.Subsystems;

/**
 * Restricts update times of a value to happen
 * every x milliseconds.
 * Helps with loop times.
 * @param <T> Value return type
 */
public abstract class ValueTimer<T> {

    private T value; //value type
    private double iTime = 0; //initial time used for delay
    private double delayTime = 500; //amount of time (in milliseconds) that value is delayed
    private boolean isPaused = false; //if reading is paused

    /**
     * main phase of subsystem
     * delays reads of value
     * must be in begging of loop
     */
    public void update() {
        //delay for reads
       if((System.currentTimeMillis() - iTime > delayTime && !isPaused)){
            value = readValue(); //reads inputted value
            iTime = System.currentTimeMillis(); //resets iTime for next loop
       }
    }

    /**
     * reads value (subsystem input)
     * @return returns inputted value to subsystem
     */
    public abstract T readValue();

    /**
     * out of subsystem to get delayed value
     * @return delayed value
     */
    public T getValue(){
        return value;
    }

    /**
     * sets delay time
     * (default is 500 mil secs)
     * must be before update function
     * @param delayTime inputted delay time in mil secs
     */
    public void setDelayTime (double delayTime){
        this.delayTime = delayTime;
    }

    /**
     * keeps getValue at a constant value
     */
    public void pause(){
        isPaused = true;
    }

    /**
     * resumes the pause function
     */
    public void resume(){
        isPaused = false;
    }
}
