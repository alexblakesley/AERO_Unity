using System;
using Unity.VisualScripting.Dependencies.Sqlite;

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
    public double alpha { get; set; }
    public double beta { get; set; }
    public double Va { get; set; }
    public double DragTerms { get; set; }

    // Wind Vector
    double Uw = 0;
    double Vw = 0;
    double Ww = 0;

    // Parameters
    public double g = 9.81;
    public double rho = 1.225;
    public double m = 11; // Mass (kg)
    public double b = 2.9; // Wing Span (m)
    public double t = 1; // Tail Offset (m)
    public double c = 0.19; // Wing Chord (m)
    public double S = 0.55; // Wing Area (m^2)
    public double l1 = 0.4; // CoG to motor offset in x (m)
    public double l2 = 0.4; // CoG to motor offset in y (m)
    public double Ix = 0.824; // Inertia in X (kg m^2)
    public double Iy = 1.135; // Inertia in Y (kg m^2)
    public double Iz = 1.759; // Inertia in Z (kg m^2)
    public double Ixz = 0.12; // Inertia in X - Z (kg m^2)

    public double S_prop = 0.2027; // propellor area (m^2)
    public double C_prop = 1; // propellor efficiency
    public double k_motor = 80; // motor efficiency
    public double n_eff = Math.PI / 4; // motor efficiency
    public double CT_q = 2.20 * Math.Pow(10, -4); // drag coefficient (𝑁/𝑠^2)
    public double CM_q = 5.58 * Math.Pow(10, -6); // torque coefficient (𝑁m/𝑠^2)

    public double omega_max = 736.81; // (rad/s)
    public double thrust_max = 125; // (N)
    public double torque_max = 3.26; // (Nm)

    // Longitudinal Aerodynamic coefficients (no unit)
    public double CL_0 = 0.23;
    public double CD_0 = 0.043;
    public double Cm_0 = 00;
    public double CL_alpha = 5.61;
    public double CD_alpha = 0.030;
    public double Cm_alpha = 0;
    public double CL_q = 7.95;
    public double CD_q = 0;
    public double Cm_q = -38.21;
    public double CL_Ue = 0.13;
    public double CD_Ue = 0.0135;
    public double Cm_Ue = -0.99;
    public double alpha0 = 0.47;
    public double CD_p = 0.043;


    // Lateral Aerodynamic coefficients (no unit)
    public double CY_0 = 0;
    public double Cl_0 = 0;
    public double Cn_0 = 0;
    public double CY_beta = -0.83;
    public double Cl_beta = -0.13;
    public double Cn_beta = 0.073;
    public double CY_p = 0;
    public double Cl_p = -0.51;
    public double Cn_p = -0.069;
    public double CY_r = 0;
    public double Cl_r = 0.25;
    public double Cn_r = -0.095;
    public double CY_Ua = 0.075;
    public double Cl_Ua = 0.17;
    public double Cn_Ua = -0.011;
    public double CY_Ur = 0.19;
    public double Cl_Ur = 0.0024;
    public double Cn_Ur = -0.069;


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

        if (U == 0 && V == 0 && W == 0){
            U = 0.00000000000000001;
        }
        
        double U_res = U - Uw;
        double V_res = Math.Round(V - Vw, 4);
        double W_res = Math.Round(W - Ww, 4);

        Va = Math.Sqrt(Math.Pow(U_res, 2) + Math.Pow(V_res, 2) + Math.Pow(W_res, 2));

        alpha = Math.Atan(W_res / U_res) * sigmoid(U_res);
        beta = Math.Asin(V_res / Va);

        DragTerms = 0.5 * rho * Math.Pow(Va, 2) * S;
    }

    public double sigmoid(double term, double widthFactor = 10)
    {   
        return 2 / (1 + Math.Exp(-widthFactor * term)) - 1;
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
        // Gravity
        double grav = m * g * sigmoid(-Z, 10);
        double Fb_g_x = - grav * Math.Sin(Theta);
        double Fb_g_y =   grav * Math.Cos(Theta) * Math.Sin(Phi);
        double Fb_g_z =   grav * Math.Cos(Theta) * Math.Cos(Phi);

        // Aero
        double Fs_drag = DragTerms * ((CD_0 + CD_alpha) * alpha + CD_q * c / (2 * U) * Q + CD_Ue * Ue);
        double Fs_lift = DragTerms * ((CL_0 + CL_alpha) * alpha + CL_q * c / (2 * U) * Q + CL_Ue * Ue);

        double Fb_a_x = Math.Cos(alpha) * - Fs_drag    + - Math.Sin(alpha) * - Fs_lift;
        double Fb_a_y = DragTerms * b * (CY_0 + CY_beta * beta + CY_p * b / (2 * U) * P  + CY_r * b / (2 * U) * R + CY_Ua * Ua + CY_Ur * Ur);
        double Fb_a_z = Math.Sin(alpha) * - Fs_drag    +   Math.Cos(alpha) * - Fs_lift;
        
        // Engine 
        double Fb_eng_x = 0.5 * rho * S_prop * C_prop * (Math.Pow(k_motor * Ut, 2) - Math.Pow(U, 2));
        double Fb_eng_y = 0;
        double Fb_eng_z = 0;
        
        // Motors
        double Fb_mtr_x = 0;
        double Fb_mtr_y = 0;
        double Fb_mtr_z = - CT_q * (Math.Pow(U1, 2) + Math.Pow(U2, 2) + Math.Pow(U3, 2) + Math.Pow(U4, 2));

        // Sum
        double Fx = Fb_g_x + Fb_a_x + Fb_eng_x + Fb_mtr_x;
        double Fy = Fb_g_y + Fb_a_y + Fb_eng_y + Fb_mtr_y;
        double Fz = Fb_g_z + Fb_a_z + Fb_eng_z + Fb_mtr_z;

        double dU = Fx / m + (R * V - Q * W);
        double dV = Fy / m + (P * W - R * U);
        double dW = Fz / m + (Q * U - P * V);

        double[] ret = {dU, dV, dW};

        return ret;
    }

    private double[] getRotationDerivatives()
    {
        double dPhi =    P + (Q * Math.Sin(Phi) + R * Math.Cos(Phi)) * Math.Tan(Theta);
        double dTheta =       Q * Math.Cos(Phi) - R * Math.Sin(Phi);
        double dPsi =        (Q * Math.Sin(Phi) + R * Math.Cos(Phi)) * (1 / Math.Cos(Theta));
        
        double[] ret = {dPhi, dTheta, dPsi};

        return ret;
    }

    private double[] getAngularVelocityDerivatives()
    {   
        // Aero
        double Mb_a_phi = DragTerms * b * (Cl_0 + Cl_beta * beta + Cl_p * b / (2 * U) * P + Cl_r * b / (2 * U) * R + Cl_Ua * Ua + Cl_Ur * Ur);
        double Mb_a_theta = DragTerms * c * (Cm_0 + Cm_alpha * alpha + Cm_q * c / (2 * U) * Q + Cm_Ue * Ue);
        double Mb_a_psi = DragTerms * b * (Cn_0 + Cn_beta * beta + Cn_p * b / (2 * U) * P + Cn_r * b / (2 * U) * R + Cn_Ua * Ua + Cn_Ur * Ur);
        
        // Motors
        double Mb_mtr_phi = CT_q * Math.Cos(n_eff) * (Math.Pow(U1, 2) - Math.Pow(U2, 2) - Math.Pow(U3, 2) + Math.Pow(U4, 2));
        double Mb_mtr_theta = CT_q * Math.Sin(n_eff) * (Math.Pow(U1, 2) + Math.Pow(U2, 2) - Math.Pow(U3, 2) - Math.Pow(U4, 2));
        double Mb_mtr_psi = CM_q                   * (Math.Pow(U1, 2) - Math.Pow(U2, 2) + Math.Pow(U3, 2) - Math.Pow(U4, 2));

        double[][] M = new double[3][];
        M[0] = new double[] { Mb_a_phi + Mb_mtr_phi };
        M[1] = new double[] { Mb_a_theta + Mb_mtr_theta };
        M[2] = new double[] { Mb_a_psi + Mb_mtr_psi };

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
