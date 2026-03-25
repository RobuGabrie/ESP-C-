-- Run with:
-- psql "postgresql://postgres:gabigay1524@db.imdevwnnqmrnmxlwqxzn.supabase.co:5432/postgres" -f supabase_schema.sql

create table if not exists public.sensor_readings (
  id bigint generated always as identity primary key,
  created_at timestamptz not null default now(),
  device_ts text not null,
  uptime_s integer not null,
  temp_c real,
  ldr_raw integer,
  ldr_pct real,
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
  ina_pow_raw integer,
  pcf_therm_raw integer,
  pcf_ext_raw integer,
  pcf_pot_raw integer
);

-- Quick setup for device inserts over REST.
alter table public.sensor_readings disable row level security;

grant usage on schema public to anon, authenticated;
grant insert, select on public.sensor_readings to anon, authenticated;
