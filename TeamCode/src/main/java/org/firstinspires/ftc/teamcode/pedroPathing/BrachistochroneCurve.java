package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.Arrays;

/**
 * A Brachistochrone (Cycloid) curve implementation for Pedro Pathing.
 * This represents the path of quickest descent between two points.
 */
public class BrachistochroneCurve extends CustomCurve {

    private Pose start;
    private Pose end;
    private double R;
    private double thetaMax;
    private double offsetX, offsetY;

    public BrachistochroneCurve(Pose start, Pose end) {
        super((FuturePose) Arrays.asList(start, end));
        initialize();
    }

    @Override
    public void initialize() {
        this.start = getControlPoints().get(0);
        this.end = getControlPoints().get(1);

        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();

        // Numerically solve for thetaMax: dy/dx = (1 - cos(t)) / (t - sin(t))
        this.thetaMax = solveThetaMax(dy / dx);
        this.R = dy / (1.0 - Math.cos(thetaMax));

        this.offsetX = start.getX();
        this.offsetY = start.getY();
    }

    @Override
    public Pose getPose(double t) {
        double theta = t * thetaMax;
        double x = R * (theta - Math.sin(theta));
        double y = R * (1.0 - Math.cos(theta));
        return new Pose(offsetX + x, offsetY + y);
    }

    @Override
    public Vector getDerivative(double t) {
        // dx/dt = R * (1 - cos(theta)) * dTheta/dt
        // dy/dt = R * sin(theta) * dTheta/dt
        // Since theta = t * thetaMax, dTheta/dt = thetaMax
        double theta = t * thetaMax;
        double dx = R * (1.0 - Math.cos(theta)) * thetaMax;
        double dy = R * Math.sin(theta) * thetaMax;
        return new Vector(dx, dy);
    }

    @Override
    public Vector getSecondDerivative(double t) {
        // d2x/dt2 = R * sin(theta) * (thetaMax^2)
        // d2y/dt2 = R * cos(theta) * (thetaMax^2)
        double theta = t * thetaMax;
        double d2x = R * Math.sin(theta) * Math.pow(thetaMax, 2);
        double d2y = R * Math.cos(theta) * Math.pow(thetaMax, 2);
        return new Vector(d2x, d2y);
    }

    @Override
    public String pathType() {
        return "Brachistochrone Curve";
    }

    @Override
    public CustomCurve getReversed() {
        return new BrachistochroneCurve(end, start);
    }

    private double solveThetaMax(double ratio) {
        double theta = Math.PI; // Initial guess
        for (int i = 0; i < 100; i++) {
            double f = (1.0 - Math.cos(theta)) / (theta - Math.sin(theta)) - ratio;
            double df = (Math.sin(theta) * (theta - Math.sin(theta)) - Math.pow(1.0 - Math.cos(theta), 2))
                    / Math.pow(theta - Math.sin(theta), 2);
            double nextTheta = theta - f / df;
            if (Math.abs(nextTheta - theta) < 1e-7) return nextTheta;
            theta = nextTheta;
        }
        return theta;
    }
}