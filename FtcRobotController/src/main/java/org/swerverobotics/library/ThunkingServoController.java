package org.swerverobotics.library;

import com.qualcomm.robotcore.hardware.*;

/**
 * An implementation of ServoController that talks to a non-thunking target implementation
 * by thunking all calls over to the loop thread and back gain.
 */
class ThunkingServoController implements ServoController
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    ServoController target;   // can only talk to him on the loop thread

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    private ThunkingServoController(ServoController target)
        {
        this.target = target;
        }

    static public ThunkingServoController Create(ServoController target)
        {
        return target instanceof ThunkingServoController ? (ThunkingServoController)target : new ThunkingServoController(target);
        }

    //----------------------------------------------------------------------------------------------
    // ServoController interface
    //----------------------------------------------------------------------------------------------

    @Override public String getDeviceName()
        {
        class Thunk extends SynchronousOpMode.ResultableAction<String>
            {
            @Override public void actionOnLoopThread()
                {
                this.result = target.getDeviceName();
                }
            }
        Thunk thunk = new Thunk();
        thunk.dispatch();
        return thunk.result;
        }

    @Override public int getVersion()
        {
        class Thunk extends SynchronousOpMode.ResultableAction<Integer>
            {
            @Override public void actionOnLoopThread()
                {
                this.result = target.getVersion();
                }
            }
        Thunk thunk = new Thunk();
        thunk.dispatch();
        return thunk.result;
        }

    @Override public void close()
        {
        class Thunk extends SynchronousOpMode.WaitableAction
            {
            @Override public void actionOnLoopThread()
                {
                target.close();
                }
            }
        Thunk thunk = new Thunk();
        thunk.dispatch();
        }

    @Override public void pwmEnable()
        {
        class Thunk extends SynchronousOpMode.WaitableAction
            {
            @Override public void actionOnLoopThread()
                {
                target.pwmEnable();
                }
            }
        Thunk thunk = new Thunk();
        thunk.dispatch();
        }

    @Override public void pwmDisable()
        {
        class Thunk extends SynchronousOpMode.WaitableAction
            {
            @Override public void actionOnLoopThread()
                {
                target.pwmDisable();
                }
            }
        Thunk thunk = new Thunk();
        thunk.dispatch();
        }

    @Override public ServoController.PwmStatus getPwmStatus()
        {
        class Thunk extends SynchronousOpMode.ResultableAction<ServoController.PwmStatus>
            {
            @Override public void actionOnLoopThread()
                {
                this.result = target.getPwmStatus();
                }
            }
        Thunk thunk = new Thunk();
        thunk.dispatch();
        return thunk.result;
        }

    @Override public void setServoPosition(int channel, double position)
        {
        class Thunk extends SynchronousOpMode.WaitableAction
            {
            int channel;
            double position;
            @Override public void actionOnLoopThread()
                {
                target.setServoPosition(channel, position);
                }
            }
        Thunk thunk = new Thunk();
        thunk.channel = channel;
        thunk.position = position;
        thunk.dispatch();
        }

    @Override public double getServoPosition(int channel)
        {
        class Thunk extends SynchronousOpMode.ResultableAction<Double>
            {
            int channel;
            @Override public void actionOnLoopThread()
                {
                this.result = target.getServoPosition(channel);
                }
            }
        Thunk thunk = new Thunk();
        thunk.channel = channel;
        thunk.dispatch();
        return thunk.result;
        }
    }