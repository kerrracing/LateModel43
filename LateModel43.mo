package LateModel43
  "Advanced Work/Power/Delta-T Correlator for Oswego Late Model 43"

  // ===================================================================
  //  Engine Dyno Function (Coyote 707hp)
  // ===================================================================
  function engineTorqueLookup
    input  Real rpm;
    output Real torque "N*m";
  protected
    constant Real rpm_tab[9] = {2000, 3000, 4000, 5000, 6000, 6500, 6800, 7200, 7500};
    constant Real pwr_tab[9] = {180000, 250000, 350000, 450000, 510000, 527000, 527000, 500000, 460000};
    Real tau_tab[9];
  algorithm
    for j in 1:9 loop
      tau_tab[j] := pwr_tab[j] / (rpm_tab[j]*2*Modelica.Constants.pi/60);
    end for;
    if rpm <= rpm_tab[1] then
      torque := tau_tab[1];
    elseif rpm >= rpm_tab[9] then
      torque := tau_tab[9];
    else
      torque := tau_tab[1];
      for j in 1:8 loop
        if rpm >= rpm_tab[j] and rpm <= rpm_tab[j+1] then
          torque := tau_tab[j] + (tau_tab[j+1]-tau_tab[j])*(rpm-rpm_tab[j])/(rpm_tab[j+1]-rpm_tab[j]);
        end if;
      end for;
    end if;
  end engineTorqueLookup;

  // ===================================================================
  //  Main Model
  // ===================================================================
  model Correlator
    import SI = Modelica.Units.SI;
    import S  = Modelica.Blocks.Types.Smoothness;
    import E  = Modelica.Blocks.Types.Extrapolation;

    // -------------------------------------------------------------------
    // 1. TELEMETRY INPUT (13 Columns from Python Script)
    // -------------------------------------------------------------------
    parameter String fileName = "oswego_table.txt";
    
    Modelica.Blocks.Sources.CombiTimeTable mo(
      tableOnFile=true, tableName="oswego", fileName=fileName,
      columns={2,3,4,5,6,7,8,9,10,11,12,13}, // 1-based index (skips Time)
      smoothness=S.LinearSegments, extrapolation=E.HoldLastPoint);
    
    Real brake_pct    = mo.y[1];  // %
    Real lat_g        = mo.y[2];  // G
    Real lon_g        = mo.y[3];  // G
    Real v_kmh        = mo.y[4];  // km/h
    Real gear_meas    = mo.y[5];  // Gear
    Real rpm_meas     = mo.y[6];  // RPM
    Real steer_deg    = mo.y[7];  // deg
    Real throttle_pct = mo.y[8];  // %
    Real w_fl         = mo.y[9];  // rad/s
    Real w_fr         = mo.y[10]; // rad/s
    Real w_rl         = mo.y[11]; // rad/s
    Real w_rr         = mo.y[12]; // rad/s

    // -------------------------------------------------------------------
    // 2. ADJUSTABLE VEHICLE PARAMETERS
    // -------------------------------------------------------------------
    parameter SI.Mass m_veh       = 1280.0;
    parameter SI.Mass m_driver    = 75.0;
    SI.Mass m_total = m_veh + m_driver;
    
    parameter SI.Density rho_air  = 1.202;
    parameter Real       Cd       = 0.35;
    parameter Real       Cl       = 0.45; 
    parameter SI.Area    A_front  = 2.0;
    
    parameter SI.Radius  r_wheel  = 0.3315;
    parameter Real       Crr      = 0.015;
    
    parameter Real gear_ratios[4] = {2.66, 1.78, 1.30, 1.00};
    parameter Real final_drive    = 4.875;
    parameter Real eta_drive      = 0.92; 

    // -------------------------------------------------------------------
    // 3. DRIVER / TELEMETRY DECODING
    // -------------------------------------------------------------------
    Real throttle = max(0, min(1, throttle_pct/100.0));
    Real brake    = max(0, min(1, brake_pct/100.0));
    SI.Velocity v_meas = v_kmh/3.6;
    SI.Acceleration a_lon = lon_g*9.81;

    SI.Velocity v_rear_wheels = (w_rl + w_rr)/2 * r_wheel;
    Real slip_ratio_rear = (v_rear_wheels - v_meas) / max(0.1, v_meas);

    // -------------------------------------------------------------------
    // 4. POWER & FORCE GENERATION
    // -------------------------------------------------------------------
    // Convert floating gear (4.0) to integer (4)
    Integer gear_idx = max(1, min(4, integer(gear_meas)));
    Real current_gear = gear_ratios[gear_idx];

    SI.AngularVelocity omega_eng = rpm_meas*2*Modelica.Constants.pi/60;
    SI.Torque tau_eng = engineTorqueLookup(rpm_meas);
    SI.Power P_engine = tau_eng * omega_eng * throttle;

    SI.Force F_tractive = (tau_eng * throttle * current_gear * final_drive * eta_drive) / r_wheel;
    
    SI.Force F_drag = 0.5*rho_air*Cd*A_front*v_meas^2;
    SI.Force F_down = 0.5*rho_air*Cl*A_front*v_meas^2;
    SI.Force F_roll = Crr*(m_total*9.81 + F_down);
    
    SI.Force F_scrub = m_total * abs(a_lon) * abs(sin(steer_deg * Modelica.Constants.pi/180));
    SI.Force F_brake = brake * (P_engine / max(0.1, v_meas));

    SI.Force F_net = F_tractive - F_drag - F_roll - F_scrub - F_brake;

    // -------------------------------------------------------------------
    // 5. WORK AND POWER
    // -------------------------------------------------------------------
    SI.Power P_net_theoretical = F_net * v_meas;
    SI.Power P_kin_actual = m_total * a_lon * v_meas;

    SI.Energy W_engine(start=0, fixed=true);
    SI.Energy W_kin_actual(start=0, fixed=true);
    SI.Energy W_loss(start=0, fixed=true);

    // -------------------------------------------------------------------
    // 6. DELTA T & SENSITIVITY
    // -------------------------------------------------------------------
    SI.Time DeltaT_calc(start=0, fixed=true);
    
    SI.Energy W_ref = abs(W_engine) + abs(W_loss);
    SI.Time DeltaT_simple = W_ref / max(100.0, P_engine);

    SI.Time dt_from_mass;
    SI.Time dt_from_Cd;
    SI.Time dt_from_downforce;
    SI.Time dt_from_slip;

  equation
    der(W_engine)     = P_engine;
    der(W_kin_actual) = P_kin_actual;
    der(W_loss)       = F_drag*v_meas + F_roll*v_meas + F_scrub*v_meas;

    der(DeltaT_calc) = if abs(P_engine) > 100.0 then (1.0/abs(P_engine)) * der(W_engine) else 0;

    dt_from_mass = if P_engine > 1000 then -(W_engine / (P_engine^2)) * ( - (Crr*9.81 + a_lon) * v_meas ) else 0;
    dt_from_Cd = if P_engine > 1000 then -(W_engine / (P_engine^2)) * ( - 0.5*rho_air*A_front*v_meas^3 * 0.01 ) else 0;
    dt_from_downforce = if P_engine > 1000 then -(W_engine / (P_engine^2)) * ( - Crr*0.5*rho_air*A_front*v_meas^3 * 0.01 ) else 0;
    dt_from_slip = if P_engine > 1000 then (F_tractive * (v_rear_wheels - v_meas)) / P_engine else 0;

  end Correlator;
end LateModel43;