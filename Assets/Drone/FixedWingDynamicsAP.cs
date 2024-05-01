using System.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class FixedWingDynamicsAP
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double U { get; set; }
    public double V { get; set; }
    public double W { get; set; }
    public double Phi { get; set; }
    public double Theta { get; set; }
    public double Psi { get; set; }
    public double P { get; set; }
    public double Q { get; set; }
    public double R { get; set; }
    public double Ua { get; set; }
    public double Ue { get; set; }
    public double Ur { get; set; }
    public double Ut { get; set; }

    // Parameters
    public double g=9.81;

    // Navion 
    public double c=1.679;
    public double b=10.18;
    public double sw=17.09;

    public double rho=1.225;
    public double uinf=53.75;
    public double m=1247.0;
    public double Ix=1421.0;
    public double Iy=4068.0;
    public double Iz=4787.0;
    public double Ixz=0.0;
    public double Ixy=0.0;

    public double xu=-0.1;
    public double xw=0.073;
    public double zu=-0.806;
    public double zw=-4.57;
    public double zwd=-1.153;
    public double zq=-2.5;
    public double mu=0.0;
    public double mw=-1.159;
    public double mwd=-3.102;
    public double mq=-6.752;
    public double yv=-0.564;
    public double lv=-0.074;
    public double lp=-0.205;
    public double lr=0.0535;
    public double nv=0.0701;
    public double np=-0.02875;
    public double nr=-0.0625;

    public double xelv=0.0;
    public double zelv=-0.5;
    public double melv=-1.35;
    public double xthr=1.0;
    public double zthr=0.0;
    public double mthr=0.0;
    public double yrud=0.156;
    public double lrud=0.0118;
    public double nrud=-0.0717;
    public double yail=0.0;
    public double lail=-0.1352;
    public double nail=-0.00346;

    public FixedWingDynamicsAP(
        double _x, double _y, double _z,
        double _u, double _v, double _w,
        double _phi, double _theta, double _psi,
        double _p, double _q, double _r,
        double _Ua, double _Ue, double _Ur, double _Ut
    ){
        X = _x;
        Y = _y;
        Z = _z;
        U = _u;
        V = _v;
        W = _w;
        Phi = _phi;
        Theta = _theta;
        Psi = _psi;
        P = _p;
        Q = _q;
        R = _r;
        Ua = _Ua;
        Ue = _Ue;
        Ur = _Ur;
        Ut = _Ut;
    }

    public double[] GetDerivatives()
    {
        double dX = Math.Cos(Theta) * Math.Cos(Psi) * U + 
            (Math.Sin(Phi) * Math.Sin(Theta) * Math.Cos(Psi) - Math.Cos(Phi) * Math.Sin(Psi)) * V + 
            (Math.Cos(Phi) * Math.Sin(Theta) * Math.Cos(Psi) + Math.Sin(Phi) * Math.Sin(Psi)) * W;

        double dY = Math.Cos(Theta) * Math.Sin(Psi) * U +
            (Math.Sin(Phi) * Math.Sin(Theta) * Math.Sin(Psi) + Math.Cos(Phi) * Math.Cos(Psi)) * V + 
            (Math.Cos(Phi) * Math.Sin(Theta) * Math.Sin(Psi) - Math.Sin(Phi) * Math.Cos(Psi)) * W;

        double dZ = -Math.Sin(Theta) * U + 
            Math.Sin(Phi) * Math.Cos(Theta) * V + 
            Math.Cos(Phi) * Math.Cos(Theta) * W;

            
        double dPhi = P + Math.Sin(Phi) * Math.Tan(Theta) * Q + Math.Cos(Phi) * Math.Tan(Theta) * R;
        double dTheta = Math.Cos(Phi) * Q - Math.Sin(Phi) * R;
        double dPsi = Math.Sin(Phi) * (1 / Math.Cos(Theta)) * Q + Math.Cos(Phi) * (1 / Math.Cos(Theta)) * R;

        double[] longitudinal = getLongitudinal();
        double dU = longitudinal[0];
        double dW = longitudinal[1];
        double dQ = longitudinal[2];
        dTheta = longitudinal[3];
        
        double[] lateral = getLateral();
        double dV = lateral[0];
        double dP = lateral[1];
        double dR = lateral[2];
        dPhi = lateral[3];

        double[] ret = {dX, dY, dZ, dU, dV, dW, dPhi, dTheta, dPsi, dP, dQ, dR};

        return ret;
    }

    private double[] getLongitudinal()
    {
        double gamma0 = Phi;

        double xudim=0.5*rho*uinf*sw*xu;
        double xwdim=0.5*rho*uinf*sw*xw;
        
        double zudim=0.5*rho*uinf*sw*zu;
        double zwdim=0.5*rho*uinf*sw*zw;
        double zwddim=0.5*rho*sw*c*zwd;
        double zqdim=.5*rho*uinf*sw*c*zq;
        
        double mudim=0.5*rho*uinf*sw*c*mu;
        double mwdim=0.5*rho*uinf*sw*c*mw;
        double mwddim=0.5*rho*sw*Math.Pow(c, 2)*mwd;
        double mqdim=0.5*rho*uinf*sw*Math.Pow(c, 2)*mq;
        
        double xelvdim=0.5*rho*Math.Pow(uinf, 2)*sw*xelv;
        double zelvdim=0.5*rho*Math.Pow(uinf, 2)*sw*zelv;
        double melvdim=0.5*rho*Math.Pow(uinf, 2)*sw*c*melv;
        
        double xthrdim=0.5*rho*Math.Pow(uinf, 2)*sw*xthr;
        double zthrdim=0.5*rho*Math.Pow(uinf, 2)*sw*zthr;
        double mthrdim=0.5*rho*Math.Pow(uinf, 2)*sw*c*mthr;

        double [][] mp = new double[4][];
        mp[0] = new double[] { m, 0,         0,  0};
        mp[1] = new double[] { 0, m-zwddim,  0,  0};
        mp[2] = new double[] { 0, -mwddim,   Iy, 0};
        mp[3] = new double[] { 0, 0,         0,  1};
        double[][] mpinv = MatrixControl.MatrixInverse(mp);

        double[][] ap = new double[4][];
        ap[0] = new double[] {xudim, xwdim,  0.0,           -m*g*Math.Cos(gamma0)};
        ap[1] = new double[] {zudim, zwdim,  zqdim+m*uinf,  m*g*Math.Sin(gamma0)};
        ap[2] = new double[] {mudim, mwdim,  mqdim,         0.0};
        ap[3] = new double[] {0.0,   0.0,    1.0,           0.0};
        
        double[][] x = new double[4][];
        x[0] = new double[] {U};
        x[1] = new double[] {W};
        x[2] = new double[] {Q};
        x[3] = new double[] {Theta};

        double[][] bp = new double[4][];
        bp[0] = new double[] {xelvdim,  xthrdim,  xudim,  xwdim};
        bp[1] = new double[] {zelvdim,  zthrdim,  zudim,  zwdim};
        bp[2] = new double[] {melvdim,  mthrdim,  mudim,  mwdim};
        bp[3] = new double[] {0.0,      0.0,      0.0,    0.0};
        
        double[][] u = new double[4][];
        u[0] = new double[] {Ue};
        u[1] = new double[] {Ut};
        u[2] = new double[] {0};
        u[3] = new double[] {0};

        double[][] aa = MatrixControl.MatrixProduct(mpinv, ap);
        double[][] bb = MatrixControl.MatrixProduct(mpinv, bp);

        double[][] xdot = MatrixControl.MatrixAdd(
            MatrixControl.MatrixProduct(aa, x), 
            MatrixControl.MatrixProduct(bb, u)
        );

        double[] ret = {xdot[0][0], xdot[1][0], xdot[2][0], xdot[3][0]};

        return ret;
    }

    private double[] getLateral()
    {
        double yvdim=0.5*rho*uinf*sw*yv;
        double lvdim=0.5*rho*uinf*sw*b*lv;
        double lpdim=0.5*rho*uinf*sw*Math.Pow(b, 2)*lp;
        double lrdim=0.5*rho*uinf*sw*Math.Pow(b, 2)*lr;
        double nvdim=0.5*rho*uinf*sw*b*nv;
        double npdim=0.5*rho*uinf*sw*Math.Pow(b, 2)*np;
        double nrdim=0.5*rho*uinf*sw*Math.Pow(b, 2)*nr;

        double yaildim=0.5*rho*Math.Pow(uinf, 2)*sw*yail;
        double laildim=0.5*rho*Math.Pow(uinf, 2)*sw*b*lail;
        double naildim=0.5*rho*Math.Pow(uinf, 2)*sw*b*nail;

        double yruddim=0.5*rho*Math.Pow(uinf, 2)*sw*yrud;
        double lruddim=0.5*rho*Math.Pow(uinf, 2)*sw*b*lrud;
        double nruddim=0.5*rho*Math.Pow(uinf, 2)*sw*b*nrud;


        double [][] mp = new double[4][];
        mp[0] = new double[] { m, 0,     0,   0};
        mp[1] = new double[] { 0, Ix,   -Ixz, 0};
        mp[2] = new double[] { 0, -Ixz,  Iz,  0};
        mp[3] = new double[] { 0, 0,     0,   1};
        double[][] mpinv = MatrixControl.MatrixInverse(mp);

        double[][] ap = new double[4][];
        ap[0] = new double[] {yvdim, 0.0,    -m*uinf, m*g};
        ap[1] = new double[] {lvdim, lpdim,  lrdim,   0};
        ap[2] = new double[] {nvdim, npdim,  nrdim,   0.0};
        ap[3] = new double[] {0.0,   1.0,    0.0,     0.0};
        
        double[][] x = new double[4][];
        x[0] = new double[] {V};
        x[1] = new double[] {P};
        x[2] = new double[] {R};
        x[3] = new double[] {Phi};

        double[][] bp = new double[4][];
        bp[0] = new double[] {yaildim,  yruddim,  yvdim};
        bp[1] = new double[] {laildim,  lruddim,  lvdim};
        bp[2] = new double[] {naildim,  nruddim,  nvdim};
        bp[3] = new double[] {0.0,      0.0,      0.0};
        
        double[][] u = new double[3][];
        u[0] = new double[] {0};
        u[1] = new double[] {Ua};
        u[2] = new double[] {Ur};

        double[][] xdot = MatrixControl.MatrixAdd(
            MatrixControl.MatrixProduct(MatrixControl.MatrixProduct(mpinv, ap), x), 
            MatrixControl.MatrixProduct(MatrixControl.MatrixProduct(mpinv, bp), u)
        );

        double[] ret = {xdot[0][0], xdot[1][0], xdot[2][0], xdot[3][0]};

        return ret;
    }
}

// Constants from Slides
// X_U  = -0.1;
// X_W  =  0.073;
// X_Ue =  0;
// X_Ut =  1;

// Z_U  = -0.806;
// Z_W  = -4.57;
// Z_dW = -1.153;
// Z_Q  = -2.5;
// Z_Ue = -0.5;

// M_U  =  0;
// M_W  = -1.159;
// M_dW = -3.102;
// M_Q  = -6.752;
// M_Ue = -1.35;
// Y_V  = -0.564;
// Y_Ur =  0.156;
// Y_Ua =  0;

// L_V  = -0.074;
// L_P  = -0.205;
// L_R  =  0.0535;
// L_Ur =  0.0118;
// L_Ua = -0.1352;

// N_V  =  0.0701;
// N_P  = -0.02875;
// N_R  = -0.0625;
// N_Ur = -0.0717;
// N_Ua = -0.00326;