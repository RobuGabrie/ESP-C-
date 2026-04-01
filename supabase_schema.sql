-- Run with:
-- psql "postgresql://postgres:gabigay1524@db.imdevwnnqmrnmxlwqxzn.supabase.co:5432/postgres" -f supabase_schema.sql

create table if not exists public.sensor_readings (
  id bigint generated always as identity primary key,
  created_at timestamptz not null default now(),
  device_ts text not null,
  uptime_s integer not null,
  temp_c real,
  therm_raw integer,
  gyro_x_dps real,
  gyro_y_dps real,
  gyro_z_dps real,
  gyro_abs_dps real,
  accel_x_g real,
  accel_y_g real,
  accel_z_g real,
  roll_deg real,
  pitch_deg real,
  yaw_deg real,
  heading_deg real,
  quat_w real,
  quat_x real,
  quat_y real,
  quat_z real,
  mag_x real,
  mag_y real,
  mag_z real,
  lin_ax_mps2 real,
  lin_ay_mps2 real,
  lin_az_mps2 real,
  vel_x_mps real,
  vel_y_mps real,
  vel_z_mps real,
  pos_x_m real,
  pos_y_m real,
  pos_z_m real,
  mag_present boolean,
  imu_addr integer,
  fusion_mode text,
  coord_frame text,
  axis_convention text,
  angles_unit text,
  quat_order text,
  rssi_dbm integer,
  cpu_pct real,
  voltage_v real,
  current_ma real,
  power_mw real,
  used_mah real,
  battery_pct real,
  battery_min integer,
  ina_bus_raw integer,
  ina_curr_raw integer,
  ina_pow_raw integer
);

-- Quick setup for device inserts over REST.
alter table public.sensor_readings disable row level security;

grant usage on schema public to anon, authenticated;
grant insert, select on public.sensor_readings to anon, authenticated;
