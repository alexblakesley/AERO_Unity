using System;

public class HybridDynamicsMarta
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
    public double U1 { get; set; }
    public double U2 { get; set; }
    public double U3 { get; set; }
    public double U4 { get; set; }

    // Parameters
    public double g = 9.81;
    public double m = 21.5;
    public double rho = 1.225;
    public double ws = 3.3; // Wing Span (m)
    public double t = 1; // Tail Offset (m)
    public double c = 0.2; // Wing Chord (m)
    public double S = 0.784; // Wing Area (m^2)
    public double l1 = 0.4; // CoG to motor offset in x (m)
    public double l2 = 0.4; // CoG to motor offset in y (m)
    public double Ix = 0.08; // Inertia in X
    public double Iy = 0.05; // Inertia in Y
    public double Iz = 0.12; // Inertia in Z
    public double Ixz = 0; // Inertia in X - Z 

    public double Cd = 0.08; // dimensionless drag coefficient in xyz axis.

    public double b = 2.20 * Math.Pow(10, -4); // drag coefficient (𝑁/𝑠^2)
    public double d = 5.58 * Math.Pow(10, -6); // torque coefficient (𝑁m/𝑠^2)

    public double omega_max = 736.81; // (rad/s)
    public double thrust_max = 125; // (N)
    public double torque_max = 3.26; // (Nm)


    public HybridDynamicsMarta(
        double _x, double _y, double _z,
        double _u, double _v, double _w,
        double _phi, double _theta, double _psi,
        double _p, double _q, double _r,
        double _Ua, double _Ue, double _Ur, double _Ut,
        double _U1, double _U2, double _U3, double _U4
    ){
        X = _x;
        Y = _y;
        Z = -_z;
        U = _u;
        V = _v;
        W = -_w;
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
        U1 = _U1;
        U2 = _U2;
        U3 = _U3;
        U4 = _U4;
    }

    public double[] GetDerivatives()
    {
        double[] position = getPositionDerivatives();
        double dX = position[0];
        double dY = position[1];
        double dZ = position[2];
        
        double[] velocity = getVelocityDerivatives();
        double dU = velocity[0];
        double dV = velocity[1];
        double dW = velocity[2];
        
        double[] rotation = getRotationDerivatives();
        double dPhi = rotation[0];
        double dTheta = rotation[1];
        double dPsi = rotation[2];
        
        double[] angular_velocity = getAngularVelocityDerivatives();
        double dP = angular_velocity[0];
        double dQ = angular_velocity[1];
        double dR = angular_velocity[2];

        double[] ret = {dX, dY, -dZ, dU, dV, -dW, dPhi, dTheta, dPsi, dP, dQ, dR};
        return ret;
    }

    private double[] getPositionDerivatives()
    {
        double[][] RVb = new double[3][];
        RVb[0] = new double[] {
            Math.Cos(Theta)*Math.Cos(Psi), 
            Math.Sin(Phi)*Math.Sin(Theta)*Math.Cos(Psi) - Math.Cos(Phi)*Math.Sin(Psi), 
            Math.Cos(Phi)*Math.Sin(Theta)*Math.Cos(Psi) + Math.Sin(Phi)*Math.Sin(Psi)
        };
        RVb[1] = new double[] {
            Math.Cos(Theta)*Math.Sin(Psi), 
            Math.Sin(Phi)*Math.Sin(Theta)*Math.Sin(Psi) + Math.Cos(Phi)*Math.Cos(Psi), 
            Math.Cos(Phi)*Math.Sin(Theta)*Math.Sin(Psi) - Math.Sin(Phi)*Math.Cos(Psi)
        };
        RVb[2] = new double[] {
            -Math.Sin(Theta), 
            Math.Sin(Phi)*Math.Cos(Theta), 
            Math.Cos(Phi)*Math.Cos(Theta)
        };

        double[][] x1 = new double[3][];
        x1[0] = new double[] {U};
        x1[1] = new double[] {V};
        x1[2] = new double[] {W};

        double[][] x1dot = MatrixControl.MatrixProduct(RVb, x1);
        
        double[] ret = {x1dot[0][0], x1dot[1][0], x1dot[2][0]};

        return ret;
    }

    private double[] getVelocityDerivatives()
    {
        double Fb_g_x = - m * g * Math.Sin(Theta);
        double Fb_g_y =   m * g * Math.Cos(Theta) * Math.Sin(Phi);
        double Fb_g_z =   m * g * Math.Cos(Theta) * Math.Cos(Phi);

        double Fb_a_x = - 0.5 * rho * U * Math.Abs(U) * S * Cd; // (Cd + Math.Sin(Ue) + Math.Sin(Ua) + Math.Sin(Ur));
        double Fb_a_y = - 0.5 * rho * V * Math.Abs(V) * S * Cd;
        double Fb_a_z = - 0.5 * rho * W * Math.Abs(W) * S * Cd;
        
        double F_eng = Ut;
        double Fb_eng_x = F_eng;
        double Fb_eng_y = 0;
        double Fb_eng_z = 0;
        
        double Fb_mtr_x = 0;
        double Fb_mtr_y = 0;
        double Fb_mtr_z = -b * (Math.Pow(U1, 2) + Math.Pow(U2, 2) + Math.Pow(U3, 2) + Math.Pow(U4, 2));

        double Fx = Fb_g_x + Fb_a_x + Fb_eng_x + Fb_mtr_x;
        double Fy = Fb_g_y + Fb_a_y + Fb_eng_y + Fb_mtr_y;
        double Fz = Fb_g_z + Fb_a_z + Fb_eng_z + Fb_mtr_z;

        double dU = Fx / m - P * U;
        double dV = Fy / m - Q * V;
        double dW = Fz / m - R * W;

        double[] ret = {dU, dV, dW};

        return ret;
    }

    private double[] getRotationDerivatives()
    {
        double dPhi = P + (R * Math.Cos(Phi) + Q * Math.Sin(Phi)) * Math.Tan(Theta);
        double dTheta = Q * Math.Cos(Phi) - R * Math.Sin(Phi);
        double dPsi = (Q * Math.Sin(Phi) + R * Math.Cos(Phi)) * (1 / Math.Cos(Theta));
        
        double[] ret = {dPhi, dTheta, dPsi};

        return ret;
    }

    private double[] getAngularVelocityDerivatives()
    {
        double Mb_a_x = rho * Math.Pow(U, 2) * S * Math.Sin(Ua) * ws;
        double Mb_a_y = rho * Math.Pow(V, 2) * S * Math.Sin(Ue) * t;
        double Mb_a_z = rho * Math.Pow(W, 2) * S * Math.Sin(Ur) * t;
        
        double Mb_mtr_x = b * l1 * (Math.Pow(U1, 2) + Math.Pow(U4, 2) - Math.Pow(U2, 2) - Math.Pow(U3, 2));
        double Mb_mtr_y = b * l2 * (Math.Pow(U1, 2) + Math.Pow(U2, 2) - Math.Pow(U3, 2) - Math.Pow(U4, 2));
        double Mb_mtr_z = d * (Math.Pow(U1, 2) + Math.Pow(U3, 2) - Math.Pow(U2, 2) - Math.Pow(U4, 2));

        double[][] M = new double[3][];
        M[0] = new double[] { Mb_a_x + Mb_mtr_x };
        M[1] = new double[] { Mb_a_y + Mb_mtr_y };
        M[2] = new double[] { Mb_a_z + Mb_mtr_z };

        double[][] PQR = new double[3][];
        PQR[0] = new double[] { P };
        PQR[1] = new double[] { Q };
        PQR[2] = new double[] { R };

        double[][] I = new double[3][];
        I[0] = new double[] { Ix,    0,    -Ixz };
        I[1] = new double[] { 0,     Iy,   0    };
        I[2] = new double[] { -Ixz,  0,    Iz  };
        
        double[][] I_inv = MatrixControl.MatrixInverse(I);

        double[][] temp = MatrixControl.MatrixProduct(I, PQR);
        double[][] temp2 = MatrixControl.MatrixProduct(PQR, MatrixControl.MatrixTranspose(temp));
        double[][] temp3 = MatrixControl.MatrixAdd(M, temp2, true);
        double[][] x4dot = MatrixControl.MatrixProduct(I_inv, temp3);

        double[] ret = {x4dot[0][0], x4dot[1][0], x4dot[2][0]};

        return ret;
    }
}
