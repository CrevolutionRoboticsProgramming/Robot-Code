package org.frc2851.crevolib.motion;


import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.concurrent.locks.ReentrantLock;

public class PID implements Sendable
{
    private double p, i, d, f;
    private ReentrantLock mMutex = new ReentrantLock();
    public PID(double p, double i, double d, double f)
    {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public double getP()
    {
        mMutex.lock();
        try {
            return p;
        } finally {
            mMutex.unlock();
        }
    }

    public double getI()
    {
        mMutex.lock();
        try {
            return i;
        } finally {
            mMutex.unlock();
        }
    }

    public double getD()
    {
        mMutex.lock();
        try {
            return d;
        } finally {
            mMutex.unlock();
        }
    }

    public double getF()
    {
        mMutex.lock();
        try {
            return f;
        } finally {
            mMutex.unlock();
        }
    }

    public void setP(double val)
    {
        mMutex.lock();
        try {
            p = val;
        } finally {
            mMutex.unlock();
        }
    }

    public void setI(double val)
    {
        mMutex.lock();
        try {
            i = val;
        } finally {
            mMutex.unlock();
        }
    }

    public void setD(double val)
    {
        mMutex.lock();
        try {
            d = val;
        } finally {
            mMutex.unlock();
        }
    }

    public void setF(double val)
    {
        mMutex.lock();
        try {
            f = val;
        } finally {
            mMutex.unlock();
        }
    }

    public void setPID(double p, double i, double d, double f)
    {
        mMutex.lock();
        try {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        } finally {
            mMutex.unlock();
        }
    }

    public void setPID(double p, double i, double d)
    {
        setPID(p, i, d, 0);
    }

    public void reset()
    {

    }

    @Override
    public String toString()
    {
        return "PID[" + p + ", " + i + ", " + d + ", " + f + "]";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PID");
        builder.setSafeState(this::reset);
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
    }
}
