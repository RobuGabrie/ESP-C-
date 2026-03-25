# ESP32-monitoring
# ESP-C-

## Supabase SQL setup

Connection string (with credentials provided):

postgresql://postgres:gabigay1524@db.imdevwnnqmrnmxlwqxzn.supabase.co:5432/postgres

Create table and permissions:

```bash
psql "postgresql://postgres:gabigay1524@db.imdevwnnqmrnmxlwqxzn.supabase.co:5432/postgres" -f supabase_schema.sql
```

Important for ESP upload to Supabase REST:

- In `src/main.cpp`, set `SUPABASE_API_KEY` to your Supabase anon key (or service role key).
- The ESP now calls Supabase on every sensor read (1 second interval) using `POST /rest/v1/sensor_readings`.
