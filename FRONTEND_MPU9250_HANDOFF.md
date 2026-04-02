# Handoff Frontend - Miscare in spatiu (MPU9250)

Acest document descrie exact ce trebuie consumat in frontend pentru tracking 3D, pe baza implementarii curente din firmware.

## 1) Transport recomandat pentru frontend

- Endpoint realtime: `ws://<ESP_IP>:8001`
- Frecventa target cand exista client WS conectat: ~60 Hz (interval ~16 ms)
- Cand nu exista client WS: loop intern la ~50 ms fara broadcast
- Payload construit in `buildImuJSON`

Referinte:
- `src/main.cpp` (constante interval): `IMU_PUBLISH_INTERVAL_MS=16`, `IMU_PUBLISH_IDLE_INTERVAL_MS=50`
- `src/main.cpp` (publicare WS): blocul din `loop()` care apeleaza `buildImuJSON` + `imuWs.broadcastTXT`

## 2) Cadrul de coordonate si conventii

Campuri meta trimise in payload:
- `frame = "right-handed"`
- `axes = "x:right,y:forward,z:up"`
- `angles = "degrees"`
- `quat_order = "wxyz"`

Interpretare:
- X pozitiv: dreapta
- Y pozitiv: inainte
- Z pozitiv: sus
- Quaternion: (`q0`,`q1`,`q2`,`q3`) = (`w`,`x`,`y`,`z`)

## 3) Campuri de miscare relevante (WS)

### Orientare (foloseste acestea in prim-plan)
- `q0,q1,q2,q3` - quaternion normalizat (unitless)
- `roll,pitch,yaw` - unghiuri in grade

### Input inertial brut filtrat
- `gx,gy,gz` - viteze unghiulare in deg/s
- `ax,ay,az` - acceleratie in g (dupa bias)
- `gravity_x,gravity_y,gravity_z` - vector gravitate estimat in g

### Miscare translationala estimata (efect vizual)
- `lin_ax,lin_ay,lin_az` - acceleratie liniara in m/s^2, in world frame
- `vel_x,vel_y,vel_z` - viteza estimata in m/s
- `pos_x,pos_y,pos_z` - pozitie relativa estimata in m
- `dt_ms` - pasul de integrare folosit
- `stationary` / `zupt` - true cand detecteaza stationar

### Stare algoritm
- `mode`:
  - `fusion` = fuziune quaternion + accel + optional magnetometru
  - `mpu_dmp` = orientare din DMP (daca activ)

## 4) Ce calculeaza firmware-ul (pe scurt)

1. Citeste gyro + accel (si magnetometru daca exista AK8963).
2. Estimeaza orientarea cu quaternion integration, corectata cu accelerometru.
3. Daca exista magnetometru, corecteaza drift-ul de yaw (heading blend dinamic).
4. Separa gravitatia din acceleratie.
5. Roteaza acceleratia liniara in world frame.
6. Integreaza in timp pentru viteza si pozitie.
7. Activeaza ZUPT cand e aproape stationar, forteaza viteza spre 0 si stabilizeaza pozitia.

## 5) Recomandare clara pentru frontend

- Pentru orientare camera/obiect 3D: foloseste direct quaternion (`q0..q3`).
- Pentru HUD/telemetrie: afiseaza `roll/pitch/yaw`, `gx/gy/gz`, `stationary`.
- Pentru translatie (`pos_*`): trateaz-o ca estimare relativa cu drift (nu GPS-grade).
- Daca ai nevoie de heading stabil: prefera `yaw` cand `mag_present` e disponibil in fluxul MQTT 1 Hz (`hardandsoft/esp32/data`) pentru validare context.

## 6) Contract JSON minim pentru frontend

```json
{
  "up": 1234,
  "q0": 0.9981,
  "q1": 0.0124,
  "q2": -0.0582,
  "q3": 0.0129,
  "roll": 1.5,
  "pitch": -6.7,
  "yaw": 271.1,
  "gx": 2.1,
  "gy": -0.4,
  "gz": 0.7,
  "lin_ax": 0.013,
  "lin_ay": -0.026,
  "lin_az": 0.004,
  "vel_x": 0.091,
  "vel_y": -0.002,
  "vel_z": 0.000,
  "pos_x": 0.341,
  "pos_y": -0.043,
  "pos_z": 0.010,
  "dt_ms": 16.65,
  "stationary": false,
  "mode": "fusion",
  "frame": "right-handed",
  "axes": "x:right,y:forward,z:up",
  "angles": "degrees",
  "quat_order": "wxyz"
}
```

## 7) Snippet frontend (Three.js) - quaternion direct

```js
// q0..q3 vin in ordinea w,x,y,z
function applyImuQuaternion(object3D, imu) {
  const w = imu.q0 ?? 1;
  const x = imu.q1 ?? 0;
  const y = imu.q2 ?? 0;
  const z = imu.q3 ?? 0;

  // Three.js foloseste (x,y,z,w)
  object3D.quaternion.set(x, y, z, w);
}
```

## 8) Observatii importante

- In cod exista structura/functie Kalman (`KalmanAxis`, `kalmanUpdateAxis`), dar in fluxul curent nu este apelata.
- Pozitia (`pos_*`) este integrare inertiala pura cu ZUPT, deci are drift inerent.
- Pentru UX bun, filtreaza vizual translatiile mici in frontend (de ex. deadzone + smoothing usor).
