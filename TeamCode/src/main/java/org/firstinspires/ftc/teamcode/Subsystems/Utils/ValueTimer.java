package org.firstinspires.ftc.teamcode.Subsystems.Utils;

/**
 * Restricts update times of a value to happen
 * every x milliseconds.
 * Helps with loop times.
 * @param <T> Value return type
 */
public abstract class ValueTimer<T> implements Input {

    private T value; //value type
    private long iTime = 0; //initial time used for delay
    private long delayTime; //amount of time (in milliseconds) that value is delayed
    private boolean isPaused = true; //if reading is paused

    public ValueTimer(long delayTime) {
        this.delayTime = delayTime;
    }

    /**
     * main phase of subsystem
     * delays reads of value
     * must be in begging of loop
     */
    @Override
    public void updateInput() {
        //delay for reads
       if(!isPaused && System.currentTimeMillis() - iTime > delayTime){
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
    public void setDelayTime(long delayTime){
        this.delayTime = delayTime;
    }

    /**
     * resumes the pause function
     */
    @Override
    public void startInput(){
        isPaused = false;
    }

    /**
     * keeps getValue at a constant value
     */
    @Override
    public void stopInput(){
        isPaused = true;
    }
}
