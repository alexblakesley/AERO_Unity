using System.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class FixedWingDynamics
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
    public double g = 9.81;
    public double m = 1.01;
    public double rho = 1.293;
    public double S = 0.228;
    public double b = 1.31;
    public double chord = 0.175;
    public double Ix = 0.02;
    public double Iy = 0.026;
    public double Iz = 0.053;
    public double Ixz = 0;
    public double Ixy = 0;
    public double rotor_r = 0.0993;
    public double rotor_A = 0.031;
    public double C_T = 0.12;
    public double k_motor = 11.39;
    public double q_motor = 239;
    public double Uw = 0;
    public double Vw = 0;
    public double Ww = 0;

    private readonly double UrFactor = 0.0000000000000001f;

    public FixedWingDynamics(
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

        // FORCES
        //  forces = (FG_x, FA_x, FT_x, FG_y, FA_y, FT_y, FG_z, FA_z, FT_z)
        double[] forces = FixedWingDynamics_getForces();
        double FG_x = forces[0];
        double FA_x = forces[1];
        double FT_x = forces[2];
        double FG_y = forces[3];
        double FA_y = forces[4];
        double FT_y = forces[5];
        double FG_z = forces[6];
        double FA_z = forces[7];
        double FT_z = forces[8];

        double Fx = FG_x + FA_x + FT_x;
        double Fy = FG_y + FA_y + FT_y;
        double Fz = FG_z + FA_z + FT_z;
        
        double dU = R * V - Q * W + Fx / m;
        double dV = P * W - R * U + Fy / m;
        double dW = Q * U - P * V + Fz / m;
        
        // Moments
        // moments = (
        //    L, M, N, 
        //    Cl_beta, Cl_p, Cl_ail, Cl_r, Cl_rud, 
        //    Cm_0, Cm_alpha, Cm_q, Cm_ele,
        //    Cn_beta, Cn_p, Cn_ail, Cn_r, Cn_rud
        // )
        double[] moments = FixedWingDynamics_getMoments();

        double L = moments[0];
        double M = moments[1];
        double N = moments[2];
                
        double C = Ix * Iz - Math.Pow(Ixz, 2);
        double C1 = Ixz * (Ix - Iy + Iz) / C;
        double C2 = (Iz * (Iz - Iy) + Math.Pow(Ixz, 2)) / C;
        double C3 = Iz / C;
        double C4 = Ixz / C;
        double C5 = (Iz - Ix) / Iy;
        double C6 = Ixy / Iy;
        double C7 = ((Ix - Iy) * Ix + Math.Pow(Ixy, 2)) / C;
        double C8 = Ix / C;
        
        double dP = C1 * P * Q - C2 * Q * R + C3 * L + C4 * N;
        double dQ = C5 * P * R - C6 * (Math.Pow(P, 2) - Math.Pow(R, 2)) + M / Iy;
        double dR = C7 * P * Q - C1 * Q * R + C4 * L + C8 * N;
        
        // double dP = ( -(Iz - Iy) * q * r + L ) / Ix;
        // double dQ = ( -(Ix - Iz) * r * p + M ) / Iy;
        // double dR = ( -(Iy - Ix) * p * q + N ) / Iz;
        Debug.Log("L: "+L);
        Debug.Log("dP: "+dP);

        double[] ret = {dX, dY, dZ, dU, dV, dW, dPhi, dTheta, dPsi, dP, dQ, dR};

        return ret;
    }

    private double[] FixedWingDynamics_getForces()
    {
        double Ubar = U - Uw;
        double Vbar = V - Vw;
        double Wbar = W - Ww;

        double V_a = Math.Sqrt(Math.Pow(Ubar, 2) + Math.Pow(Vbar, 2) + Math.Pow(Wbar, 2));

        double alpha = Math.Atan(Wbar / Ubar);
        double beta = Math.Asin(Vbar / V_a);

        //// FORCES
        // Gravitational
        double FG_x = m * g * -Math.Sin(Theta);
        double FG_y = m * g * Math.Sin(Phi) * Math.Cos(Theta);
        double FG_z = m * g * Math.Cos(Phi) * Math.Cos(Theta);

        // Thrust
        double throttle = k_motor * Ut + q_motor;
        double Ft = 0.5 * rho * rotor_A * C_T * (Math.Pow(rotor_r * throttle, 2) - Math.Pow(V_a, 2));

        double FT_x = Ft;
        double FT_y = 0;
        double FT_z = 0;

        // Aerodynamic
        double CD_0 = 0;
        double CD_alpha = getCD_Basic(alpha) * alpha;
        double CD_q = 0 * chord / (2 * V_a) * Q;
        double CD_ele = getCD_Elevator(alpha, Ue) * Ue;

        double CY_beta = -0.3073 * beta;
        double CY_p = getCy_RollRate(alpha) * b * P / (2 * V_a);

        double CL_0 = 0;
        double CL_alpha = getCL_Basic(alpha) * alpha;
        double CL_q = 7.9520 * chord / (2 * V_a) * Q;
        double CL_ele = getCL_Elevator(Ue) * Ue;

        double CD = CD_0 + CD_alpha + CD_q + CD_ele;
        double CY = CY_beta + CY_p;
        double CL = CL_0 + CL_alpha + CL_q + CL_ele;

        double F_D = 0.5 * rho * Math.Pow(V_a, 2) * S * CD;
        double F_Y = 0.5 * rho * Math.Pow(V_a, 2) * S * CY;
        double F_L = 0.5 * rho * Math.Pow(V_a, 2) * S * CL;

        double FA_x = Math.Cos(beta) * Math.Cos(alpha) * -F_D + -Math.Sin(alpha) * -F_L;
        double FA_y = F_Y;
        double FA_z = Math.Cos(beta) * Math.Sin(alpha) * -F_D + Math.Cos(alpha) * -F_L;

        
        double[] ret = {FG_x, FA_x, FT_x, FG_y, FA_y, FT_y, FG_z, FA_z, FT_z};

        return ret;

    }

    private double getCD_Basic(double alpha) { 
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data     
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};     
        double[] cd_basic_values = {0.02612, 0.02615, 0.02705, 0.02890, 0.03168, 0.03544, 0.04017, 0.04599, 0.05291, 0.06097,                  
            0.07022, 0.08072, 0.09255, 0.11490, 0.13790, 0.15980, 0.17540, 0.17630, 0.15880, 0.09811};     
        // Perform linear interpolation    
        return interp1(alpha_values, cd_basic_values, alpha); 
    }

    private double getCD_Elevator(double alpha, double Ue) {
        alpha = Mathf.Rad2Deg*alpha;
        Ue = Mathf.Rad2Deg*Ue;
        // Define the table data for Cn_Aileron
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] Ue_values = {-20, -10, -5, -1, 0, 1, 5, 10, 20};
        
        // Define the matrix of Cn_Aileron values as per the table
        double[,] cd_elevator_values = {
            { 0.0035,  0.0017,  0.0005,  0.0001, 0.0000, 0.0000, 0.0000, 0.0007, 0.0020}, // alpha -2
            { 0.0031,  0.0014,  0.0004,  0.0000, 0.0000, 0.0000, 0.0002, 0.0009, 0.0024}, // alpha -1
            { 0.0027,  0.0011,  0.0003,  0.0000, 0.0000, 0.0000, 0.0003, 0.0012, 0.0028}, // alpha 0
            { 0.0024,  0.0009,  0.0002,  0.0000, 0.0000, 0.0000, 0.0004, 0.0014, 0.0032}, // alpha 1
            { 0.0020,  0.0006,  0.0000,  0.0000, 0.0000, 0.0001, 0.0006, 0.0017, 0.0036}, // alpha 2
            { 0.0016,  0.0004, -0.0001, -0.0001, 0.0000, 0.0001, 0.0007, 0.0019, 0.0040}, // alpha 3
            { 0.0012,  0.0001, -0.0002, -0.0001, 0.0000, 0.0001, 0.0008, 0.0022, 0.0044}, // alpha 4
            { 0.0008, -0.0001, -0.0004, -0.0001, 0.0000, 0.0001, 0.0009, 0.0024, 0.0047}, // alpha 5
            { 0.0004, -0.0004, -0.0005, -0.0001, 0.0000, 0.0002, 0.0011, 0.0027, 0.0051}, // alpha 6
            { 0.0000, -0.0006, -0.0006, -0.0002, 0.0000, 0.0002, 0.0012, 0.0030, 0.0055}, // alpha 7
            {-0.0004, -0.0009, -0.0007, -0.0002, 0.0000, 0.0002, 0.0013, 0.0032, 0.0059}, // alpha 8
            {-0.0008, -0.0012, -0.0009, -0.0002, 0.0000, 0.0002, 0.0014, 0.0035, 0.0063}, // alpha 9
            {-0.0012, -0.0014, -0.0010, -0.0002, 0.0000, 0.0003, 0.0016, 0.0037, 0.0068}, // alpha 10
            {-0.0021, -0.0020, -0.0013, -0.0003, 0.0000, 0.0003, 0.0019, 0.0043, 0.0077}, // alpha 12
            {-0.0032, -0.0027, -0.0016, -0.0004, 0.0000, 0.0004, 0.0022, 0.0050, 0.0088}, // alpha 14
            {-0.0044, -0.0035, -0.0020, -0.0005, 0.0000, 0.0005, 0.0026, 0.0058, 0.0099}, // alpha 16
            {-0.0058, -0.0044, -0.0025, -0.0005, 0.0000, 0.0006, 0.0030, 0.0068, 0.0113}, // alpha 18
            {-0.0066, -0.0049, -0.0027, -0.0006, 0.0000, 0.0006, 0.0033, 0.0072, 0.0122}, // alpha 19
            {-0.0076, -0.0056, -0.0031, -0.0007, 0.0000, 0.0007, 0.0037, 0.0079, 0.0132}, // alpha 20
            {-0.0148, -0.0102, -0.0054, -0.0011, 0.0000, 0.0011, 0.0060, 0.0125, 0.0203}, // alpha 25
        };
        int idx = findNearestNeighbourIndex( alpha, alpha_values );

        double[] row = Enumerable.Range(0, cd_elevator_values.GetLength(0))
                .Select(x => cd_elevator_values[x, idx])
                .ToArray();

        return interp1(Ue_values, row, Ue); 

        // Perform bilinear interpolation
        // ret = interp2(Ue_values, alpha_values, cd_elevator_values, Ue, alpha, "linear");
        // ret(isNan(ret))=100;
        // return ret;
    }

    private double getCL_Basic(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for CL_Basic     
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};     
        double[] cl_basic_values = {-0.0381, 0.0516, 0.1432, 0.2372, 0.3338, 0.4327, 0.5335, 0.6360, 0.7399, 0.8451,
            0.9515, 1.0590, 1.1680, 1.3480, 1.5050, 1.6320, 1.6990, 1.6810, 1.5210, 0.0707};     
        // Perform linear interpolation     
        return interp1(alpha_values, cl_basic_values, alpha); 
    }

    private double getCL_Elevator(double Ue) {
        Ue = Mathf.Rad2Deg*Ue;
        // Define the table data for CL_Basic     
        double[] Ue_values = {-20, -10, -5, -1, 0, 1, 5, 10, 20};     
        // double[] cl_elevator_values = {-0.0924, -0.0598, -0.0299, -0.0060, 0.0001, 0.0060, 0.0299, 0.0598, 0.0924};    
        double[] cl_elevator_values = {-0.0924, -0.0598, -0.0299, -0.0060, -0.0001, -0.0060, -0.0299, -0.0598, -0.0924}; // Made all negative values
        
        // Perform linear interpolation     
        return interp1(Ue_values, cl_elevator_values, Ue); 
    }

    private double getCy_RollRate(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for Cy_RollRate
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] cy_rollrate_values = {-0.0059, -0.0042, -0.0026, -0.0010, 0.0006, 0.0022, 0.0038, 0.0054, 0.0070, 0.0085,
                            0.0101, 0.0116, 0.0130, 0.0163, 0.0200, 0.0239, 0.0268, 0.0512, 0.0434, 0.0806};
        
        // Perform linear interpolation
        return interp1(alpha_values, cy_rollrate_values, alpha);
    }


    private double[] FixedWingDynamics_getMoments() {
        double Ubar = U - Uw;
        double Vbar = V - Vw;
        double Wbar = W - Ww;

        double V_a = Math.Sqrt(Math.Pow(Ubar, 2) + Math.Pow(Vbar, 2) + Math.Pow(Wbar, 2));

        double alpha = Math.Atan(Wbar / Ubar);
        double beta = Math.Asin(Vbar / V_a);

        // Aerodynamic
        double Cl_beta = getCl_Beta(alpha) * beta;
        double Cl_p = getCl_RollRate(alpha) * b / (2 * V_a) * P;
        double Cl_r = getCl_YawRate(alpha) * b / (2 * V_a) * R;
        double Cl_ail = getCl_Aileron(Ua) * Ua;
        double Cl_rud = UrFactor * Ur;
        Debug.Log("p: "+P);
        Debug.Log("Cl_p: "+Cl_p);
        
        double Cm_0 = 0;
        double Cm_alpha = getCm_Basic(alpha) * alpha;
        double Cm_q = -16.58 * chord / (2 * V_a) * Q;
        double Cm_ele = getCm_Elevator(Ue) * Ue;

        double Cn_beta = 0.0709 * beta;
        double Cn_p = getCn_RollRate(alpha) * b / (2 * V_a) * P;
        double Cn_r = getCn_YawRate(alpha) * b / (2 * V_a) * R;
        double Cn_ail = getCn_Aileron(alpha, Ua) * Ua;
        double Cn_rud = UrFactor * Ur;

        double Cl = Cl_beta + Cl_p + Cl_ail + Cl_r + Cl_rud;
        double Cm = Cm_0 + Cm_alpha + Cm_q + Cm_ele;
        double Cn = Cn_beta + Cn_p + Cn_ail + Cn_r + Cn_rud;

        double L = 0.5 * rho * S * b * Math.Pow(V_a, 2) * Cl;
        double M = 0.5 * rho * S * chord * Math.Pow(V_a, 2) * Cm;
        double N = 0.5 * rho * S * b * Math.Pow(V_a, 2) * Cn;

        double[] ret = {L, M, N, 
            Cl_beta, Cl_p, Cl_ail, Cl_r, Cl_rud, 
            Cm_0, Cm_alpha, Cm_q, Cm_ele, 
            Cn_beta, Cn_p, Cn_ail, Cn_r, Cn_rud 
        };

        return ret;
    }

    private double getCm_Basic(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for Cm_Basic     
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};     
        double[] cm_basic_values = {0.0862, 0.0598, 0.0337, 0.0068, -0.0225, -0.0538, -0.0863, -0.1200, -0.1550, -0.1914,   
            -0.2291, -0.2682, -0.3095, -0.3952, -0.5032, -0.6163, -0.6163, -0.6163, -0.6163, -0.6163};     
        // Perform linear interpolation     
        return interp1(alpha_values, cm_basic_values, alpha);
    }


    private double getCm_Elevator(double Ue) {
        Ue = Mathf.Rad2Deg*Ue;
        // Define the table data for Cm_Basic     
        double[] Ue_values = {-20, -10, -5, -1, 0, 1, 5, 10, 20};     
        // double[] cm_elevator_values = {0.2636, 0.1695, 0.0847, 0.0170, -0.0002, -0.0170, -0.0847, -0.1695, -0.2638};   
        double[] cm_elevator_values = {-0.2636, -0.1695, -0.0847, -0.0170, -0.0002, -0.0170, -0.0847, -0.1695, -0.2638}; // Made all negative values    
        // Perform linear interpolation     
        return interp1(Ue_values, cm_elevator_values, Ue);
    }


    private double getCl_Beta(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for Cl_Beta
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] cl_beta_values = {-0.0397, -0.0415, -0.0434, -0.0454, -0.0474, -0.0495, -0.0517, -0.0540, -0.0563, -0.0586, 
                        -0.0610, -0.0634, -0.0659, -0.0694, -0.0718, -0.0729, -0.0715, -0.0688, -0.0609, 0.0015};

        // Perform linear interpolation
        return interp1(alpha_values, cl_beta_values, alpha);
    }

    private double getCl_RollRate(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for Cl_RollRate
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] cl_rollrate_values = {-0.4666, -0.4802, -0.4934, -0.5059, -0.5175, -0.5280, -0.5374, -0.5458, -0.5530, -0.5591, 
                            -0.5639, -0.5676, -0.5278, -0.4056, -0.3176, -0.1707, 0.1513, 0.6986, 1.2260, 2.1430};

        // Perform linear interpolation
        return interp1(alpha_values, cl_rollrate_values, alpha);
    }

    private double getCl_YawRate(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for Cl_YawRate
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] cl_yawrate_values = {0.0187, 0.0361, 0.0541, 0.0726, 0.0966, 0.1109, 0.1307, 0.1508, 0.1717, 0.1917, 
                            0.2125, 0.2334, 0.2544, 0.2864, 0.3120, 0.3299, 0.3320, 0.3197, 0.2727, -0.0692};

        // Perform linear interpolation
        return interp1(alpha_values, cl_yawrate_values, alpha);
    }

    private double getCl_Aileron(double Ua) {
        Ua = Mathf.Rad2Deg*Ua;
        // Define the table data for Cl_YawRate
        double[] Ua_values = {0, 2, 6, 10, 16, 20, 30, 40, 50};
        double[] cl_aileron_values = {0.0000, 0.0043, 0.0130, 0.0217, 0.0348, 0.0435, 0.0636, 0.0706, 0.0740};

        // Perform linear interpolation
        return interp1(Ua_values, cl_aileron_values, Ua);
    }

    private double getCn_YawRate(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        // Define the table data for Cn_YawRate
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] cn_yawrate_values = {-0.0737, -0.0744, -0.0752, -0.0762, -0.0772, -0.0784, -0.0798, -0.0813, -0.0829, -0.0847, 
                            -0.0867, -0.0888, -0.0910, -0.0949, -0.0982, -0.1007, -0.1010, -0.0994, -0.0937, -0.0769};

        // Perform linear interpolation
        return interp1(alpha_values, cn_yawrate_values, alpha);
    }

    private double getCn_Aileron(double alpha, double Ua) {
        alpha = Mathf.Rad2Deg*alpha;
        Ua = Mathf.Rad2Deg*Ua;
        // Define the table data for Cn_Aileron
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] Ua_values = {0, 2, 6, 10, 16, 20, 30, 40, 50}; // Assuming the columns are beta values
        
        // Define the matrix of Cn_Aileron values as per the table
        // You will need to fill in the matrix with the values from your table. This is an example.
        double[,] cn_aileron_values = {
            {.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000}, // alpha -2
            {.0000,  0.0000, -0.0001, -0.0002, -0.0003, -0.0004, -0.0006, -0.0006, -0.0007}, // alpha -1
            {.0000, -0.0001, -0.0002, -0.0004, -0.0007, -0.0008, -0.0012, -0.0013, -0.0014}, // alpha 0
            {.0000, -0.0001, -0.0004, -0.0006, -0.0010, -0.0013, -0.0019, -0.0021, -0.0022}, // alpha 1
            {.0000, -0.0002, -0.0005, -0.0009, -0.0014, -0.0017, -0.0025, -0.0028, -0.0029}, // alpha 2
            {.0000, -0.0002, -0.0007, -0.0011, -0.0018, -0.0022, -0.0032, -0.0035, -0.0037}, // alpha 3
            {.0000, -0.0003, -0.0008, -0.0013, -0.0021, -0.0027, -0.0039, -0.0043, -0.0045}, // alpha 4
            {.0000, -0.0003, -0.0009, -0.0016, -0.0025, -0.0031, -0.0046, -0.0051, -0.0053}, // alpha 5
            {.0000, -0.0004, -0.0011, -0.0018, -0.0029, -0.0036, -0.0053, -0.0059, -0.0061}, // alpha 6
            {.0000, -0.0004, -0.0012, -0.0021, -0.0033, -0.0041, -0.0060, -0.0067, -0.0070}, // alpha 7
            {.0000, -0.0005, -0.0014, -0.0023, -0.0037, -0.0046, -0.0068, -0.0075, -0.0078}, // alpha 8
            {.0000, -0.0005, -0.0015, -0.0026, -0.0041, -0.0051, -0.0075, -0.0083, -0.0087}, // alpha 9
            {.0000, -0.0006, -0.0017, -0.0028, -0.0045, -0.0056, -0.0082, -0.0091, -0.0095}, // alpha 10
            {.0000, -0.0006, -0.0019, -0.0032, -0.0051, -0.0064, -0.0094, -0.0103, -0.0108}, // alpha 12
            {.0000, -0.0007, -0.0021, -0.0035, -0.0056, -0.0070, -0.0103, -0.0114, -0.0119}, // alpha 14
            {.0000, -0.0007, -0.0022, -0.0037, -0.0060, -0.0075, -0.0110, -0.0121, -0.0127}, // alpha 16
            {.0000, -0.0008, -0.0023, -0.0038, -0.0061, -0.0076, -0.0111, -0.0123, -0.0129}, // alpha 18
            {.0000, -0.0007, -0.0022, -0.0037, -0.0059, -0.0074, -0.0108, -0.0119, -0.0125}, // alpha 19
            {.0000, -0.0006, -0.0019, -0.0032, -0.0051, -0.0063, -0.0093, -0.0102, -0.0107}, // alpha 20
            {.0000,  0.0001,  0.0004,  0.0006,  0.0006,  0.0012,  0.0018,  0.0020,  0.0021}, // alpha 25
        };

        int idx = findNearestNeighbourIndex( alpha, alpha_values );

        double[] row = Enumerable.Range(0, cn_aileron_values.GetLength(0))
                .Select(x => cn_aileron_values[x, idx])
                .ToArray();

        return interp1(Ua_values, row, Ua); 
    }

    private double getCn_RollRate(double alpha) {
        alpha = Mathf.Rad2Deg*alpha;
        double[] alpha_values = {-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 19, 20, 25};
        double[] Cn_rollrate_values = {0.0027, -0.0047, -0.0121, -0.0197, -0.0274, -0.0351, -0.0429, -0.0508, -0.0588, -0.0669, -0.0750, -0.0833, -0.0940, -0.1150, -0.1329, -0.1517, -0.1698, -0.2573, -0.2137, -0.0583};

        // Perform linear interpolation
        return interp1(alpha_values, Cn_rollrate_values, alpha);
    }

    private double interp1( double[] a, double[] b, double a_new )
    {   
        List<double> dX = new();
        List<double> dY = new();
        List<double> slope = new();
        List<double> intercept = new();

        for( int i = 0; i < a.Length; ++i ){
            if( i < a.Length-1 ) {
                dX.Add( a[i+1] - a[i] );
                dY.Add( b[i+1] - b[i] );
                slope.Add( dY[i] / dX[i] );
                intercept.Add( b[i] - a[i] * slope[i] );
            } else {
                dX.Add( dX[i-1] );
                dY.Add( dY[i-1] );
                slope.Add( slope[i-1] );
                intercept.Add( intercept[i-1] );
            }
        }

        int idx = findNearestNeighbourIndex( a_new, a );
        return slope[idx] * a_new + intercept[idx];

    }

    int findNearestNeighbourIndex( double value, double[] a )
    {
        double dist = double.MaxValue;
        int idx = -1;

        for ( int i = 0; i < a.Length; ++i ) {
            double newDist = value - a[i];
            if ( newDist >= 0 && newDist < dist ) {
                dist = newDist;
                idx = i;
            }
        }

        if (idx == -1){
            Debug.Log("Index Error Found: "+value);
        }

        return idx;
    }
}
