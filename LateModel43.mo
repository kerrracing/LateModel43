package LateModel43
  "Work/Power/Delta-T Correlator for LateModel43 (2026 Snowball Derby / Five Flags)"

  // ===================================================================
  //  Engine Torque Curve (Matched 1:1 to AC power.lut - 662 hp SLM)
  // ===================================================================
  function engineTorqueLookup
    input  Real rpm;
    output Real torque "N*m";
  protected
    constant Real rpm_tab[12] = {
      2000, 3000, 4000, 5000, 5500, 6000,
      6500, 7000, 7400, 7600, 7800, 7900
    };
    constant Real tau_tab[12] = {
      210, 360, 475, 565, 605, 635,
      650, 645, 635, 620, 595, 0
    };
  algorithm
    if rpm <= rpm_tab[1] then
      torque := tau_tab[1];
    elseif rpm >= rpm_tab[12] then
      torque := tau_tab[12];
    else
      torque := tau_tab[1];
      for j in 1:11 loop
        if rpm >= rpm_tab[j] and rpm <= rpm_tab[j+1] then
          torque := tau_tab[j] + (tau_tab[j+1] - tau_tab[j]) * (rpm - rpm_tab[j]) / (rpm_tab[j+1] - rpm_tab[j]);
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

    // -----------------------------------------------------------------
    // 1. TELEMETRY INPUT (13 Columns from clean_lap.py)
    // -----------------------------------------------------------------
    parameter String fileName = "telemetry_table.txt";

    Modelica.Blocks.Sources.CombiTimeTable mo(
      tableOnFile=true,
      tableName="telemetry",
      fileName=fileName,
      columns={2,3,4,5,6,7,8,9,10,11,12,13},
      smoothness=S.LinearSegments,
      extrapolation=E.HoldLastPoint);

    Real brake_pct    = mo.y[1];  // %
    Real lat_g        = mo.y[2];  // G
    Real lon_g        = mo.y[3];  // G
    Real v_kmh        = mo.y[4];  // km/h
    Real gear_meas    = mo.y[5];  // Gear
    Real rpm_meas     = mo.y[6];  // RPM
    Real steer_deg    = mo.y[7];  // deg (steering wheel angle)
    Real throttle_pct = mo.y[8];  // %
    Real w_fl         = mo.y[9];  // rad/s
    Real w_fr         = mo.y[10]; // rad/s
    Real w_rl         = mo.y[11]; // rad/s
    Real w_rr         = mo.y[12]; // rad/s

    // -----------------------------------------------------------------
    // 2. VEHICLE PARAMETERS (Synced to car.ini, aero.ini, tyres.ini)
    // -----------------------------------------------------------------
    // AC car.ini TOTALMASS (1270 kg = 2800 lb) ALREADY includes driver
    parameter SI.Mass m_veh       = 1270.0;
    parameter SI.Mass m_fuel      = 4.0;
    SI.Mass m_total = m_veh + m_fuel;

    parameter SI.Density rho_air  = 1.156;
    parameter Real       Cd       = 0.431;  // Derived from aero.ini + LUTs
    parameter Real       Cl       = 0.666;  // Recalibrated aero.ini (CL_GAIN=3)
    parameter SI.Area    A_front  = 2.0;

    parameter SI.Radius  r_wheel     = 0.3315;
    parameter Real       steer_ratio = 12.0;   // From car.ini STEER_RATIO
    parameter Real       Crr_0       = 0.015;
    parameter Real       Crr_v2      = 1.023e-6;

    parameter Real gear_ratios[4] = {2.66, 1.78, 1.30, 1.00};
    parameter Real final_drive    = 4.625;  // Set 4.75 when analyzing lunch CSV
    parameter Real eta_drive      = 0.92;

    // -----------------------------------------------------------------
    // 3. TELEMETRY DECODING
    // -----------------------------------------------------------------
    Real throttle = max(0.0, min(1.0, throttle_pct / 100.0));
    Real brake    = max(0.0, min(1.0, brake_pct / 100.0));
    SI.Velocity     v_meas = max(0.1, v_kmh / 3.6);
    SI.Acceleration a_lon  = lon_g * 9.81;
    SI.Acceleration a_lat  = lat_g * 9.81;

    SI.Velocity v_rear_wheels = ((w_rl + w_rr) / 2.0) * r_wheel;
    Real slip_ratio_rear = (v_rear_wheels - v_meas) / max(0.5, v_meas);
    Real Crr_dyn = Crr_0 + Crr_v2 * v_meas^2;

    // -----------------------------------------------------------------
    // 4. POWER & FORCES
    // -----------------------------------------------------------------
    Integer gear_idx = max(1, min(4, integer(gear_meas + 0.5)));
    Real current_gear = gear_ratios[gear_idx];

    SI.AngularVelocity omega_eng = rpm_meas * 2.0 * Modelica.Constants.pi / 60.0;
    SI.Torque          tau_eng   = engineTorqueLookup(rpm_meas);
    SI.Power           P_engine  = tau_eng * omega_eng * throttle;
    SI.Power           P_wheel   = P_engine * eta_drive;

    SI.Force F_tractive = (tau_eng * throttle * current_gear * final_drive * eta_drive) / r_wheel;
    SI.Force F_drag     = 0.5 * rho_air * Cd * A_front * v_meas^2;
    SI.Force F_down     = 0.5 * rho_air * Cl * A_front * v_meas^2;
    SI.Force F_roll     = Crr_dyn * (m_total * 9.81 + F_down);
    SI.Force F_scrub    = (0.505 * m_total * abs(a_lat)) * abs(sin((steer_deg / steer_ratio) * Modelica.Constants.pi / 180.0));
    SI.Force F_brake    = if (brake > 0.01 and a_lon < 0.0) then m_total * (-a_lon) else 0.0;

    SI.Force F_net             = F_tractive - F_drag - F_roll - F_scrub - F_brake;
    SI.Force F_kin_actual      = m_total * a_lon;
    SI.Power P_net_theoretical = F_net * v_meas;
    SI.Power P_kin_actual      = F_kin_actual * v_meas;

    // -----------------------------------------------------------------
    // 5. INTEGRATED WORK SINKS & DELTA-T = WORK / POWER BREAKDOWN
    // -----------------------------------------------------------------
    SI.Energy W_engine(start=0, fixed=true);
    SI.Energy W_wheel(start=0, fixed=true);
    SI.Energy W_aero(start=0, fixed=true);
    SI.Energy W_roll(start=0, fixed=true);
    SI.Energy W_scrub(start=0, fixed=true);
    SI.Energy W_brake(start=0, fixed=true);
    SI.Energy W_slip(start=0, fixed=true);
    SI.Energy W_kin_actual(start=0, fixed=true);

    SI.Power  P_wheel_avg  = max(1000.0, W_wheel / max(0.1, time));
    SI.Energy W_loss_total = W_aero + W_roll + W_scrub + W_brake + W_slip;

    // Lap-time contribution (seconds) attributed to each work sink (Delta t = W / P)
    SI.Time dt_aero_total  = W_aero  / P_wheel_avg;
    SI.Time dt_roll_total  = W_roll  / P_wheel_avg;
    SI.Time dt_scrub_total = W_scrub / P_wheel_avg;
    SI.Time dt_brake_total = W_brake / P_wheel_avg;
    SI.Time dt_slip_total  = W_slip  / P_wheel_avg;

    // Sensitivities for kerrracing.com sliders (per +10% Cd, +10 kg mass, +10% Power)
    SI.Time dt_plus10pct_Cd    = 0.10 * dt_aero_total;
    SI.Time dt_plus10kg_mass   = (10.0 / m_total) * (dt_roll_total + dt_scrub_total + dt_brake_total);
    SI.Time dt_plus10pct_power = -0.10 * (W_wheel / P_wheel_avg) * ((W_aero + 0.15 * W_wheel) / max(1.0, W_wheel));

  equation
    der(W_engine)     = P_engine;
    der(W_wheel)      = P_wheel;
    der(W_aero)       = F_drag  * v_meas;
    der(W_roll)       = F_roll  * v_meas;
    der(W_scrub)      = F_scrub * v_meas;
    der(W_brake)      = F_brake * v_meas;
    der(W_slip)       = max(0.0, F_tractive * (v_rear_wheels - v_meas));
    der(W_kin_actual) = m_total * a_lon * v_meas;

  end Correlator;
end LateModel43;