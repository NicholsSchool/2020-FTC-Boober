package org.firstinspires.ftc.teamcode.util.pid;

public class MyPidControl {

    private double m_p, m_i, m_d;

    private double currentI, maxI;
    private boolean isEnabled;
    private double target;

    private long previousTime;
    private double previousError;

    public MyPidControl(double p, double i, double d)
    {
        m_p = p;
        m_i = i;
        m_d = d;
        isEnabled = false;
        previousError = 0;

    }

    public double getValue(double currentPosition)
    {
        if(!isEnabled)
            return 0;
        long currentTime = System.currentTimeMillis();
        double currentError = target - currentPosition;


        double p = m_p * currentError;
        currentI += m_i * (currentError * (currentTime - previousTime));
        if(maxI != 0)
        {
            if(currentI > maxI)
                currentI = maxI;
            else if(currentI < -maxI)
                currentI = -maxI;
        }
        double d = m_d * (currentError - previousError) / (currentTime - previousTime);
        previousError = currentError;
        previousTime = currentTime;
        return p + currentI + d;


    }

    public void setMaxI(double maxI)
    {
        this.maxI = maxI;
    }

    public void setTarget(double target)
    {
        this.target = target;
    }

    public void enable()
    {
        isEnabled = true;
        previousTime = System.currentTimeMillis();
    }

    public void disable()
    {
        isEnabled = false;
    }


    public double getP() {
        return m_p;
    }

    public void setP(double p) {
        this.m_p = p;
    }

    public double getI() {
        return m_i;
    }

    public void setI(double i) {
        this.m_i = i;
    }

    public double getD() {
        return m_d;
    }

    public void setD(double d) {
        this.m_d = d;
    }
}
