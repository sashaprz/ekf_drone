# EKF suite comparison

baseline `4d25c07` dirty (filter {'state estimation/FINAL_gps.py': 'caecea3fbb6f', 'state estimation/calibration.py': 'f891872e7efd'}) vs candidate `6074c75` dirty (filter {'state estimation/FINAL_gps.py': 'c14058582dcd', 'state estimation/calibration.py': 'f891872e7efd'})

**4 metric improvements, 3 metric regressions, 0 overall-grade changes** across 331 matched runs.

> runs only in baseline: 0, only in candidate: 30

## Regressions

- **circle_fast / imu_gap(gap_s=1.0)** `pos_h_rms_m`: 0.0715 -> 0.179 (PASS -> PASS)
- **circle_fast / imu_gap(gap_s=1.0)** `recovery_s`: 0.808 -> 1.41 (PASS -> PASS)
- **stops / imu_gap(gap_s=1.0)** `pos_h_rms_m`: 0.0593 -> 0.207 (PASS -> PASS)

## Overall grade changes

none

## Improvements

- circle_fast / imu_gap(gap_s=1.0) `tilt_max_deg`: 2.52 -> 0.875 (PASS -> PASS)
- circle_fast / imu_gap(gap_s=1.0) `nees_pos`: 0.0176 -> 0.0741 (None -> None)
- stops / imu_gap(gap_s=1.0) `nees_pos`: 0.00979 -> 0.0901 (None -> None)
- patrol_long / combined_realistic `tilt_max_deg`: 3 -> 2.31 (PASS -> PASS)

## All matched runs (metrics that changed at all)

| mission | fault | overall | changes |
|---|---|---|---|
| hover | none | FAIL | - |
| hover | gps_dropout(duration_s=5) | FAIL | - |
| hover | gps_dropout(duration_s=15) | FAIL | - |
| hover | gps_dropout(duration_s=30) | FAIL | - |
| hover | gps_stale(duration_s=5) | FAIL | - |
| hover | gps_stale(duration_s=15) | FAIL | - |
| hover | gps_stale(duration_s=30) | FAIL | - |
| hover | gps_noise | FAIL | - |
| hover | gps_outliers(frac=0.01) | FAIL | - |
| hover | gps_rate(hz=1.0) | FAIL | - |
| hover | gyro_bias(dps=0.2) | FAIL | - |
| hover | gyro_bias(dps=1.0) | FAIL | - |
| hover | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| hover | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.05) | PASS | - |
| hover | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| hover | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| hover | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| hover | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| hover | mag_bias(frac=0.05) | WARN | - |
| hover | mag_bias(frac=0.15) | FAIL | - |
| hover | mag_interference(amp=0.3) | FAIL | - |
| hover | mag_interference(amp=1.0) | FAIL | - |
| hover | imu_noise | FAIL | - |
| hover | imu_spikes | WARN | - |
| hover | imu_dropouts | FAIL | - |
| hover | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00536->0.00539; tilt_max_deg 0.0198->0.02; yaw_rms_deg 0.0184->0.0183; pos_h_rms_m 0.00156->0.00156; vel_h_rms_ms 0.00112->0.00112 |
| hover | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00547->0.00571; tilt_max_deg 0.0268->0.0257; yaw_rms_deg 0.018->0.0181; pos_h_rms_m 0.00171->0.00168; vel_h_rms_ms 0.00112->0.00112; nees_vel 0.00126->0.00125; nees_pos 0.000138->0.000145 |
| hover | combined_realistic | WARN | - |
| hover | cal_ideal | FAIL | - |
| box | none | FAIL | tilt_rms_deg 0.00889->0.00877; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0201->0.0198; pos_h_rms_m 0.00995->0.00994; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000502->0.000504; nees_vel 0.156->0.156; nees_pos 0.00102->0.00102 |
| box | gps_dropout(duration_s=5) | FAIL | tilt_rms_deg 0.0135->0.0134; tilt_max_deg 0.0619->0.0619; yaw_rms_deg 0.0269->0.0266; pos_h_rms_m 0.0131->0.0131; vel_h_rms_ms 0.0304->0.0304; nees_att 0.000523->0.000527; nees_vel 0.152->0.151; nees_pos 0.00113->0.00113; tilt_drift_deg_s 0.00717->0.00716 |
| box | gps_dropout(duration_s=15) | FAIL | tilt_rms_deg 0.0357->0.0357; tilt_max_deg 0.148->0.148; yaw_rms_deg 0.0672->0.0666; pos_h_rms_m 0.173->0.173; vel_h_rms_ms 0.0474->0.0475; nees_att 0.000525->0.00053; nees_vel 0.149->0.148; nees_pos 0.00167->0.00167; tilt_drift_deg_s 0.00579->0.00579 |
| box | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.0724->0.0724; tilt_max_deg 0.315->0.315; yaw_rms_deg 0.145->0.145; pos_h_rms_m 0.96->0.96; vel_h_rms_ms 0.125->0.125; nees_att 0.000531->0.000538; nees_vel 0.0889->0.0887; nees_pos 0.00357->0.00357; tilt_drift_deg_s 0.00353->0.00353 |
| box | gps_stale(duration_s=5) | FAIL | tilt_rms_deg 0.187->0.187; tilt_max_deg 1.19->1.19; yaw_rms_deg 1.29->1.29; pos_h_rms_m 0.814->0.814; vel_h_rms_ms 0.104->0.104; nees_att 0.616->0.606; nees_vel 2.28->2.29; nees_pos 2.85->2.85; tilt_drift_deg_s -0.0018->-0.00167 |
| box | gps_stale(duration_s=15) | WARN | tilt_rms_deg 0.411->0.411; tilt_max_deg 2.02->2.02; yaw_rms_deg 1.43->1.41; pos_h_rms_m 2.11->2.11; vel_h_rms_ms 0.25->0.251; nees_att 1.03->0.996; nees_vel 11.2->11.2; nees_pos 19->19; tilt_drift_deg_s 0.0545->0.0545 |
| box | gps_stale(duration_s=30) | WARN | tilt_rms_deg 0.494->0.495; tilt_max_deg 2.14->2.14; yaw_rms_deg 2.39->2.37; pos_h_rms_m 4.3->4.3; vel_h_rms_ms 0.356->0.356; nees_att 3.18->3.13; nees_vel 16.7->16.7; nees_pos 79.1->79.1; tilt_drift_deg_s 0.00899->0.009; recovery_s 1.99->2.39 |
| box | gps_noise | FAIL | tilt_rms_deg 0.312->0.335; tilt_max_deg 0.922->1.04; yaw_rms_deg 0.624->0.669; pos_h_rms_m 0.496->0.498; vel_h_rms_ms 0.0682->0.0686; nees_att 0.0657->0.0716; nees_vel 0.432->0.432; nees_pos 1.19->1.19 |
| box | gps_outliers(frac=0.01) | FAIL | tilt_rms_deg 0.00898->0.00886; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0202->0.0199; pos_h_rms_m 0.00991->0.0099; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000503->0.000505; nees_vel 0.156->0.156; nees_pos 0.00102->0.00102 |
| box | gps_rate(hz=1.0) | FAIL | tilt_rms_deg 0.0104->0.0111; tilt_max_deg 0.0481->0.0481; yaw_rms_deg 0.0267->0.0272; pos_h_rms_m 0.0101->0.0101; vel_h_rms_ms 0.0302->0.0303; nees_vel 0.0723->0.0721; nees_pos 0.000938->0.00094 |
| box | gyro_bias(dps=0.2) | FAIL | tilt_rms_deg 0.00889->0.00877; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0201->0.0198; pos_h_rms_m 0.00995->0.00994; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000502->0.000504; nees_vel 0.156->0.156; nees_pos 0.00102->0.00102 |
| box | gyro_bias(dps=1.0) | FAIL | tilt_rms_deg 0.00889->0.00877; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0201->0.0198; pos_h_rms_m 0.00995->0.00994; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000502->0.000504; nees_vel 0.156->0.156; nees_pos 0.00102->0.00102 |
| box | gyro_bias(dps=1.0,from_cal=False) | FAIL | tilt_rms_deg 0.0568->0.0568; yaw_rms_deg 0.174->0.174; pos_h_rms_m 0.0101->0.0101; vel_h_rms_ms 0.0313->0.0313; nees_att 0.004->0.00398; nees_vel 0.158->0.158; nees_pos 0.00102->0.00102 |
| box | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.00913->0.00914; yaw_rms_deg 0.0169->0.0163; pos_h_rms_m 0.00995->0.00994; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000501->0.000503; nees_vel 0.156->0.156; nees_pos 0.00102->0.00102 |
| box | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.292->0.292; tilt_max_deg 0.316->0.316; yaw_rms_deg 0.0863->0.0867; pos_h_rms_m 0.00965->0.00964; vel_h_rms_ms 0.0302->0.0303; nees_att 2.02->2.02; nees_vel 0.156->0.156; nees_pos 0.001->0.00101 |
| box | accel_bias(axis=x,ms2=0.2) | FAIL | tilt_rms_deg 1.17->1.17; yaw_rms_deg 0.405->0.406; pos_h_rms_m 0.00908->0.00903; vel_h_rms_ms 0.0304->0.0304; nees_att 32.4->32.4; nees_vel 0.161->0.161 |
| box | accel_bias(axis=x,ms2=0.5) | FAIL | tilt_rms_deg 2.93->2.93; yaw_rms_deg 2.12->2.12; pos_h_rms_m 0.0125->0.0122; vel_h_rms_ms 0.0331->0.033; nees_att 203->203; nees_vel 0.214->0.213; nees_pos 0.00188->0.00185 |
| box | accel_bias(axis=z,ms2=0.05) | FAIL | tilt_rms_deg 0.00886->0.00873; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0201->0.0198; pos_h_rms_m 0.00994->0.00993; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000502->0.000503; nees_vel 0.153->0.153; nees_pos 0.000576->0.000575 |
| box | accel_bias(axis=z,ms2=0.2) | FAIL | tilt_rms_deg 0.00876->0.00863; tilt_max_deg 0.0376->0.0376; yaw_rms_deg 0.0199->0.0196; pos_h_rms_m 0.00992->0.00991; vel_h_rms_ms 0.0302->0.0302; nees_att 0.0005->0.000501; nees_vel 0.161->0.16; nees_pos 0.00148->0.00148 |
| box | accel_bias(axis=z,ms2=0.5) | FAIL | tilt_rms_deg 0.00857->0.00844; tilt_max_deg 0.0373->0.0373; yaw_rms_deg 0.0197->0.0194; pos_h_rms_m 0.00989->0.00988; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000497->0.000498; nees_vel 0.248->0.248; nees_pos 0.0133->0.0134 |
| box | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | tilt_rms_deg 0.837->0.831; tilt_max_deg 1.13->1.12; yaw_rms_deg 6.54->6.51; pos_h_rms_m 0.425->0.429; vel_h_rms_ms 0.243->0.245; nees_att 92->90.7; nees_vel 13.8->14; nees_pos 0.769->0.784 |
| box | mag_bias(frac=0.05) | WARN | tilt_rms_deg 0.0292->0.029; yaw_rms_deg 2.23->2.23; pos_h_rms_m 0.0163->0.016; vel_h_rms_ms 0.0309->0.0309; nees_att 0.998->0.997; nees_vel 0.164->0.163; nees_pos 0.00174->0.0017 |
| box | mag_bias(frac=0.15) | FAIL | tilt_rms_deg 0.0968->0.0961; yaw_rms_deg 7.38->7.39; pos_h_rms_m 0.0351->0.0339; vel_h_rms_ms 0.0393->0.039; nees_att 34.6->34.5; nees_vel 0.313->0.307; nees_pos 0.00588->0.00553 |
| box | mag_interference(amp=0.3) | FAIL | tilt_rms_deg 0.0101->0.00996; tilt_max_deg 0.0376->0.0376; yaw_rms_deg 0.0218->0.0218; pos_h_rms_m 0.00996->0.00995; vel_h_rms_ms 0.0303->0.0303; nees_att 0.000463->0.000466; nees_vel 0.15->0.15; nees_pos 0.00102->0.00102; tilt_drift_deg_s -0.00408->-0.00407 |
| box | mag_interference(amp=1.0) | FAIL | tilt_rms_deg 0.0101->0.00996; tilt_max_deg 0.0376->0.0376; yaw_rms_deg 0.0218->0.0218; pos_h_rms_m 0.00996->0.00995; vel_h_rms_ms 0.0303->0.0303; nees_att 0.000463->0.000466; nees_vel 0.15->0.15; nees_pos 0.00102->0.00102; tilt_drift_deg_s -0.00408->-0.00407 |
| box | imu_noise | FAIL | tilt_rms_deg 0.0257->0.0264; tilt_max_deg 0.0799->0.0804; yaw_rms_deg 0.0581->0.0555; pos_h_rms_m 0.0147->0.0148; vel_h_rms_ms 0.0308->0.0309; nees_att 0.00212->0.00207; nees_vel 0.179->0.179; nees_pos 0.00312->0.00314 |
| box | imu_spikes | WARN | tilt_rms_deg 0.217->0.219; tilt_max_deg 1.28->1.28; yaw_rms_deg 0.47->0.476; pos_h_rms_m 0.0182->0.0182; vel_h_rms_ms 0.0485->0.0484; nees_att 0.208->0.208; nees_vel 0.829->0.829; nees_pos 0.0114->0.0115 |
| box | imu_dropouts | FAIL | tilt_rms_deg 0.00923->0.00912; tilt_max_deg 0.0354->0.0355; yaw_rms_deg 0.0211->0.0207; pos_h_rms_m 0.0098->0.00979; vel_h_rms_ms 0.0304->0.0304; nees_att 0.000504->0.000505; nees_vel 0.159->0.158; nees_pos 0.00103->0.00103 |
| box | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0089->0.00909; tilt_max_deg 0.0846->0.0439; yaw_rms_deg 0.0222->0.0228; vel_h_rms_ms 0.0303->0.0303; nees_att 0.000512->0.000522; nees_vel 0.154->0.154; nees_pos 0.000807->0.000808 |
| box | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.012->0.0166; tilt_max_deg 0.108->0.121; yaw_rms_deg 0.027->0.0333; pos_h_rms_m 0.0186->0.0174; vel_h_rms_ms 0.0305->0.0305; nees_att 0.000502->0.000503; nees_vel 0.155->0.154; nees_pos 0.00119->0.00116 |
| box | combined_realistic | WARN | tilt_rms_deg 0.437->0.454; tilt_max_deg 0.959->1.07; yaw_rms_deg 2.52->2.54; pos_h_rms_m 0.506->0.508; vel_h_rms_ms 0.0695->0.07; nees_att 3.55->3.58; nees_vel 0.463->0.468; nees_pos 1.19->1.19 |
| box | cal_ideal | FAIL | tilt_rms_deg 0.00889->0.00877; tilt_max_deg 0.0377->0.0377; yaw_rms_deg 0.0201->0.0198; pos_h_rms_m 0.00995->0.00993; vel_h_rms_ms 0.0302->0.0302; nees_att 0.000502->0.000504; nees_vel 0.155->0.155 |
| circle_slow | none | FAIL | - |
| circle_slow | gps_dropout(duration_s=5) | FAIL | - |
| circle_slow | gps_dropout(duration_s=15) | FAIL | - |
| circle_slow | gps_dropout(duration_s=30) | FAIL | - |
| circle_slow | gps_stale(duration_s=5) | WARN | - |
| circle_slow | gps_stale(duration_s=15) | WARN | - |
| circle_slow | gps_stale(duration_s=30) | WARN | - |
| circle_slow | gps_noise | FAIL | - |
| circle_slow | gps_outliers(frac=0.01) | FAIL | - |
| circle_slow | gps_rate(hz=1.0) | FAIL | - |
| circle_slow | gyro_bias(dps=0.2) | FAIL | - |
| circle_slow | gyro_bias(dps=1.0) | FAIL | - |
| circle_slow | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| circle_slow | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.05) | PASS | - |
| circle_slow | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| circle_slow | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| circle_slow | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| circle_slow | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| circle_slow | mag_bias(frac=0.05) | PASS | - |
| circle_slow | mag_bias(frac=0.15) | FAIL | - |
| circle_slow | mag_interference(amp=0.3) | FAIL | - |
| circle_slow | mag_interference(amp=1.0) | FAIL | - |
| circle_slow | imu_noise | FAIL | - |
| circle_slow | imu_spikes | WARN | - |
| circle_slow | imu_dropouts | FAIL | - |
| circle_slow | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0137->0.0143; tilt_max_deg 0.0516->0.0516; yaw_rms_deg 0.0474->0.0478; pos_h_rms_m 0.0149->0.0149; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00259->0.00264; nees_vel 0.13->0.13; nees_pos 0.00109->0.00108 |
| circle_slow | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0138->0.0191; tilt_max_deg 0.0516->0.116; yaw_rms_deg 0.0484->0.0522; pos_h_rms_m 0.0182->0.0334; vel_h_rms_ms 0.0301->0.0301; nees_att 0.00264->0.00266; nees_vel 0.128->0.128; nees_pos 0.00129->0.00282 |
| circle_slow | combined_realistic | WARN | - |
| circle_slow | cal_ideal | FAIL | - |
| circle_fast | none | FAIL | - |
| circle_fast | gps_dropout(duration_s=5) | FAIL | - |
| circle_fast | gps_dropout(duration_s=15) | FAIL | - |
| circle_fast | gps_dropout(duration_s=30) | FAIL | - |
| circle_fast | gps_stale(duration_s=5) | WARN | - |
| circle_fast | gps_stale(duration_s=15) | FAIL | - |
| circle_fast | gps_stale(duration_s=30) | FAIL | - |
| circle_fast | gps_noise | WARN | - |
| circle_fast | gps_outliers(frac=0.01) | FAIL | - |
| circle_fast | gps_rate(hz=1.0) | FAIL | - |
| circle_fast | gyro_bias(dps=0.2) | FAIL | - |
| circle_fast | gyro_bias(dps=1.0) | FAIL | - |
| circle_fast | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| circle_fast | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| circle_fast | accel_bias(axis=x,ms2=0.05) | PASS | - |
| circle_fast | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| circle_fast | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| circle_fast | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| circle_fast | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| circle_fast | mag_bias(frac=0.05) | WARN | - |
| circle_fast | mag_bias(frac=0.15) | WARN | - |
| circle_fast | mag_interference(amp=0.3) | FAIL | - |
| circle_fast | mag_interference(amp=1.0) | FAIL | - |
| circle_fast | imu_noise | FAIL | - |
| circle_fast | imu_spikes | PASS | - |
| circle_fast | imu_dropouts | FAIL | - |
| circle_fast | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.164->0.163; tilt_max_deg 0.465->0.465; yaw_rms_deg 0.291->0.29; pos_h_rms_m 0.0576->0.0579; vel_h_rms_ms 0.111->0.111; nees_att 0.0851->0.0847; nees_vel 1.4->1.41; nees_pos 0.0144->0.0146 |
| circle_fast | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.237->0.195; tilt_max_deg 2.52->0.875 **BETTER**; yaw_rms_deg 0.371->0.331; pos_h_rms_m 0.0715->0.179 **WORSE**; vel_h_rms_ms 0.111->0.111; nees_att 0.0859->0.0863; nees_vel 1.39->1.39; nees_pos 0.0176->0.0741 **BETTER**; recovery_s 0.808->1.41 **WORSE** |
| circle_fast | combined_realistic | PASS | - |
| circle_fast | cal_ideal | FAIL | - |
| stops | none | FAIL | - |
| stops | gps_dropout(duration_s=5) | FAIL | - |
| stops | gps_dropout(duration_s=15) | FAIL | - |
| stops | gps_dropout(duration_s=30) | FAIL | - |
| stops | gps_stale(duration_s=5) | FAIL | - |
| stops | gps_stale(duration_s=15) | FAIL | - |
| stops | gps_stale(duration_s=30) | WARN | - |
| stops | gps_noise | WARN | - |
| stops | gps_outliers(frac=0.01) | FAIL | - |
| stops | gps_rate(hz=1.0) | FAIL | - |
| stops | gyro_bias(dps=0.2) | FAIL | - |
| stops | gyro_bias(dps=1.0) | FAIL | - |
| stops | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| stops | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.05) | PASS | - |
| stops | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| stops | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| stops | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| stops | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| stops | mag_bias(frac=0.05) | PASS | - |
| stops | mag_bias(frac=0.15) | FAIL | - |
| stops | mag_interference(amp=0.3) | FAIL | - |
| stops | mag_interference(amp=1.0) | FAIL | - |
| stops | imu_noise | FAIL | - |
| stops | imu_spikes | WARN | - |
| stops | imu_dropouts | FAIL | - |
| stops | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0572->0.0576; tilt_max_deg 0.37->0.394; yaw_rms_deg 0.113->0.117; pos_h_rms_m 0.0397->0.0401; vel_h_rms_ms 0.0806->0.081; nees_att 0.0735->0.0737; nees_vel 1.65->1.66; nees_pos 0.00676->0.0069 |
| stops | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0578->0.0583; tilt_max_deg 0.27->0.27; yaw_rms_deg 0.12->0.12; pos_h_rms_m 0.0593->0.207 **WORSE**; vel_h_rms_ms 0.0805->0.0805; nees_att 0.0742->0.0743; nees_vel 1.62->1.63; nees_pos 0.00979->0.0901 **BETTER** |
| stops | combined_realistic | WARN | - |
| stops | cal_ideal | FAIL | - |
| yaw_steps | none | FAIL | - |
| yaw_steps | gps_dropout(duration_s=5) | FAIL | - |
| yaw_steps | gps_dropout(duration_s=15) | FAIL | - |
| yaw_steps | gps_dropout(duration_s=30) | FAIL | - |
| yaw_steps | gps_stale(duration_s=5) | FAIL | - |
| yaw_steps | gps_stale(duration_s=15) | FAIL | - |
| yaw_steps | gps_stale(duration_s=30) | FAIL | - |
| yaw_steps | gps_noise | WARN | - |
| yaw_steps | gps_outliers(frac=0.01) | FAIL | - |
| yaw_steps | gps_rate(hz=1.0) | FAIL | - |
| yaw_steps | gyro_bias(dps=0.2) | FAIL | - |
| yaw_steps | gyro_bias(dps=1.0) | FAIL | - |
| yaw_steps | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| yaw_steps | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.05) | PASS | - |
| yaw_steps | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| yaw_steps | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| yaw_steps | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| yaw_steps | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| yaw_steps | mag_bias(frac=0.05) | WARN | - |
| yaw_steps | mag_bias(frac=0.15) | FAIL | - |
| yaw_steps | mag_interference(amp=0.3) | FAIL | - |
| yaw_steps | mag_interference(amp=1.0) | FAIL | - |
| yaw_steps | imu_noise | FAIL | - |
| yaw_steps | imu_spikes | WARN | - |
| yaw_steps | imu_dropouts | FAIL | - |
| yaw_steps | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0427->0.0423; tilt_max_deg 0.148->0.148; yaw_rms_deg 0.0899->0.09; pos_h_rms_m 0.0236->0.0236; vel_h_rms_ms 0.00938->0.00937; nees_att 0.0356->0.0356; nees_vel 0.0236->0.0236; nees_pos 0.0025->0.00249 |
| yaw_steps | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0417->0.0421; tilt_max_deg 0.148->0.148; yaw_rms_deg 0.0899->0.0896; pos_h_rms_m 0.0243->0.0242; vel_h_rms_ms 0.00936->0.00936; nees_att 0.0358->0.0358; nees_vel 0.0236->0.0236; nees_pos 0.00258->0.00256 |
| yaw_steps | combined_realistic | PASS | - |
| yaw_steps | cal_ideal | FAIL | - |
| yaw_spin | none | FAIL | - |
| yaw_spin | gps_dropout(duration_s=5) | FAIL | - |
| yaw_spin | gps_dropout(duration_s=15) | FAIL | - |
| yaw_spin | gps_dropout(duration_s=30) | FAIL | - |
| yaw_spin | gps_stale(duration_s=5) | FAIL | - |
| yaw_spin | gps_stale(duration_s=15) | FAIL | - |
| yaw_spin | gps_stale(duration_s=30) | FAIL | - |
| yaw_spin | gps_noise | WARN | - |
| yaw_spin | gps_outliers(frac=0.01) | FAIL | - |
| yaw_spin | gps_rate(hz=1.0) | FAIL | - |
| yaw_spin | gyro_bias(dps=0.2) | FAIL | - |
| yaw_spin | gyro_bias(dps=1.0) | FAIL | - |
| yaw_spin | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| yaw_spin | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| yaw_spin | accel_bias(axis=x,ms2=0.05) | PASS | - |
| yaw_spin | accel_bias(axis=x,ms2=0.2) | WARN | - |
| yaw_spin | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| yaw_spin | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| yaw_spin | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | - |
| yaw_spin | mag_bias(frac=0.05) | WARN | - |
| yaw_spin | mag_bias(frac=0.15) | WARN | - |
| yaw_spin | mag_interference(amp=0.3) | FAIL | - |
| yaw_spin | mag_interference(amp=1.0) | FAIL | - |
| yaw_spin | imu_noise | FAIL | - |
| yaw_spin | imu_spikes | PASS | - |
| yaw_spin | imu_dropouts | FAIL | - |
| yaw_spin | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0214->0.0215; tilt_max_deg 0.0518->0.0534; yaw_rms_deg 0.116->0.117; pos_h_rms_m 0.0274->0.0274; vel_h_rms_ms 0.0104->0.0104; nees_att 0.071->0.071; nees_vel 0.0312->0.0312; nees_pos 0.0033->0.0033 |
| yaw_spin | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0217->0.022; tilt_max_deg 0.0947->0.0958; yaw_rms_deg 0.117->0.118; pos_h_rms_m 0.0281->0.0277; vel_h_rms_ms 0.0103->0.0103; nees_att 0.0708->0.0708; nees_vel 0.0304->0.0304; nees_pos 0.00312->0.00305 |
| yaw_spin | combined_realistic | PASS | - |
| yaw_spin | cal_ideal | FAIL | - |
| patrol_long | none | FAIL | tilt_rms_deg 0.0149->0.016; tilt_max_deg 0.338->0.39; yaw_rms_deg 0.0294->0.0319; pos_h_rms_m 0.0131->0.0132; vel_h_rms_ms 0.0405->0.0405; nees_att 0.000801->0.000769; nees_vel 0.238->0.238; nees_pos 0.000809->0.000813 |
| patrol_long | gps_dropout(duration_s=30) | FAIL | tilt_rms_deg 0.0483->0.0487; tilt_max_deg 0.582->0.582; yaw_rms_deg 0.0927->0.0938; pos_h_rms_m 0.711->0.711; vel_h_rms_ms 0.0966->0.0966; nees_att 0.000804->0.000771; nees_vel 0.232->0.232; nees_pos 0.000865->0.00087; tilt_drift_deg_s 0.0149->0.0149 |
| patrol_long | gps_noise | FAIL | tilt_rms_deg 0.35->0.351; tilt_max_deg 2.95->2.38; yaw_rms_deg 0.7->0.703; pos_h_rms_m 1.12->1.12; vel_h_rms_ms 0.0746->0.0746; nees_att 0.0759->0.0752; nees_vel 0.479->0.478; nees_pos 5.71->5.71 |
| patrol_long | gyro_drift(dps=0.5,over_s=60) | FAIL | tilt_rms_deg 0.0149->0.015; tilt_max_deg 0.341->0.359; yaw_rms_deg 0.029->0.0296; pos_h_rms_m 0.013->0.0131; vel_h_rms_ms 0.0405->0.0405; nees_att 0.000803->0.00077; nees_vel 0.238->0.238; nees_pos 0.000808->0.000812 |
| patrol_long | accel_bias(axis=x,ms2=0.05) | PASS | tilt_rms_deg 0.292->0.292; tilt_max_deg 0.521->0.525; yaw_rms_deg 0.0666->0.0679; pos_h_rms_m 0.0125->0.0126; vel_h_rms_ms 0.0405->0.0405; nees_att 2.05->2.05; nees_vel 0.238->0.238; nees_pos 0.000763->0.000767 |
| patrol_long | mag_bias(frac=0.05) | PASS | tilt_rms_deg 0.0194->0.0236; tilt_max_deg 0.324->0.49; yaw_rms_deg 0.968->0.969; pos_h_rms_m 0.0164->0.0165; vel_h_rms_ms 0.0405->0.0405; nees_att 0.659->0.657; nees_vel 0.238->0.238; nees_pos 0.00121->0.00121 |
| patrol_long | imu_noise | FAIL | tilt_rms_deg 0.0271->0.0258; tilt_max_deg 0.32->0.316; yaw_rms_deg 0.0535->0.051; pos_h_rms_m 0.0174->0.0168; vel_h_rms_ms 0.0409->0.0409; nees_att 0.00269->0.00269; nees_vel 0.288->0.288; nees_pos 0.0095->0.00944 |
| patrol_long | imu_spikes | WARN | tilt_rms_deg 0.277->0.294; tilt_max_deg 3.25->3.61; yaw_rms_deg 0.613->0.625; pos_h_rms_m 0.0552->0.0512; vel_h_rms_ms 0.0673->0.0674; nees_att 0.334->0.33; nees_vel 2.34->2.35; nees_pos 0.0975->0.0967 |
| patrol_long | combined_realistic | WARN | tilt_rms_deg 0.458->0.459; tilt_max_deg 3->2.31 **BETTER**; yaw_rms_deg 1.24->1.24; pos_h_rms_m 1.12->1.12; vel_h_rms_ms 0.0749->0.0749; nees_att 2.94->2.93; nees_vel 0.516->0.515; nees_pos 5.81->5.81 |
| patrol_long | cal_ideal | FAIL | tilt_rms_deg 0.0149->0.016; tilt_max_deg 0.338->0.39; yaw_rms_deg 0.0294->0.0319; pos_h_rms_m 0.0131->0.0132; vel_h_rms_ms 0.0405->0.0405; nees_att 0.000801->0.000769; nees_vel 0.238->0.237; nees_pos 0.000781->0.000785 |
| takeoff_land | none | FAIL | - |
| takeoff_land | gps_dropout(duration_s=5) | FAIL | - |
| takeoff_land | gps_dropout(duration_s=15) | FAIL | - |
| takeoff_land | gps_dropout(duration_s=30) | FAIL | - |
| takeoff_land | gps_stale(duration_s=5) | FAIL | - |
| takeoff_land | gps_stale(duration_s=15) | FAIL | - |
| takeoff_land | gps_stale(duration_s=30) | FAIL | - |
| takeoff_land | gps_noise | FAIL | - |
| takeoff_land | gps_outliers(frac=0.01) | FAIL | - |
| takeoff_land | gps_rate(hz=1.0) | FAIL | - |
| takeoff_land | gyro_bias(dps=0.2) | FAIL | - |
| takeoff_land | gyro_bias(dps=1.0) | FAIL | - |
| takeoff_land | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| takeoff_land | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| takeoff_land | accel_bias(axis=x,ms2=0.05) | PASS | - |
| takeoff_land | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| takeoff_land | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| takeoff_land | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| takeoff_land | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| takeoff_land | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| takeoff_land | accel_bias(axis=x,from_cal=False,ms2=0.2) | FAIL | - |
| takeoff_land | mag_bias(frac=0.05) | WARN | - |
| takeoff_land | mag_bias(frac=0.15) | FAIL | - |
| takeoff_land | mag_interference(amp=0.3) | FAIL | - |
| takeoff_land | mag_interference(amp=1.0) | FAIL | - |
| takeoff_land | imu_noise | FAIL | - |
| takeoff_land | imu_spikes | FAIL | - |
| takeoff_land | imu_dropouts | FAIL | - |
| takeoff_land | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.00517->0.00516; tilt_max_deg 0.0139->0.0139; yaw_rms_deg 0.0206->0.0206; pos_h_rms_m 0.000392->0.000387; vel_h_rms_ms 0.000659->0.000656; nees_att 0.00026->0.000259; nees_vel 1.03->1.03; nees_pos 0.0444->0.0444 |
| takeoff_land | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00706->0.00805; tilt_max_deg 0.0542->0.0475; yaw_rms_deg 0.0233->0.0249; pos_h_rms_m 0.00176->0.000735; vel_h_rms_ms 0.000679->0.000698; nees_vel 0.961->0.961; nees_pos 0.0451->0.0451 |
| takeoff_land | combined_realistic | WARN | - |
| takeoff_land | cal_ideal | FAIL | - |
| hover_ct | none | FAIL | - |
| hover_ct | gps_dropout(duration_s=5) | FAIL | - |
| hover_ct | gps_dropout(duration_s=15) | FAIL | - |
| hover_ct | gps_dropout(duration_s=30) | FAIL | - |
| hover_ct | gps_stale(duration_s=5) | FAIL | - |
| hover_ct | gps_stale(duration_s=15) | FAIL | - |
| hover_ct | gps_stale(duration_s=30) | FAIL | - |
| hover_ct | gps_noise | FAIL | - |
| hover_ct | gps_outliers(frac=0.01) | FAIL | - |
| hover_ct | gps_rate(hz=1.0) | FAIL | - |
| hover_ct | gyro_bias(dps=0.2) | FAIL | - |
| hover_ct | gyro_bias(dps=1.0) | FAIL | - |
| hover_ct | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| hover_ct | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| hover_ct | accel_bias(axis=x,ms2=0.05) | PASS | - |
| hover_ct | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| hover_ct | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| hover_ct | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| hover_ct | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| hover_ct | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| hover_ct | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | - |
| hover_ct | mag_bias(frac=0.05) | WARN | - |
| hover_ct | mag_bias(frac=0.15) | WARN | - |
| hover_ct | mag_interference(amp=0.3) | FAIL | - |
| hover_ct | mag_interference(amp=1.0) | FAIL | - |
| hover_ct | imu_noise | FAIL | - |
| hover_ct | imu_spikes | WARN | - |
| hover_ct | imu_dropouts | FAIL | - |
| hover_ct | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0091->0.00911; yaw_rms_deg 0.058->0.058; pos_h_rms_m 0.00426->0.00426; vel_h_rms_ms 0.00192->0.00192; nees_vel 0.00267->0.00267; nees_pos 0.000349->0.00035 |
| hover_ct | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.00914->0.00915; yaw_rms_deg 0.0582->0.0582; pos_h_rms_m 0.00467->0.0043; vel_h_rms_ms 0.00192->0.00192; nees_vel 0.00266->0.00266 |
| hover_ct | combined_realistic | PASS | - |
| hover_ct | cal_ideal | FAIL | - |
| box_ct | none | FAIL | - |
| box_ct | gps_dropout(duration_s=5) | FAIL | - |
| box_ct | gps_dropout(duration_s=15) | FAIL | - |
| box_ct | gps_dropout(duration_s=30) | FAIL | - |
| box_ct | gps_stale(duration_s=5) | FAIL | - |
| box_ct | gps_stale(duration_s=15) | FAIL | - |
| box_ct | gps_stale(duration_s=30) | WARN | - |
| box_ct | gps_noise | FAIL | - |
| box_ct | gps_outliers(frac=0.01) | FAIL | - |
| box_ct | gps_rate(hz=1.0) | FAIL | - |
| box_ct | gyro_bias(dps=0.2) | FAIL | - |
| box_ct | gyro_bias(dps=1.0) | FAIL | - |
| box_ct | gyro_bias(dps=1.0,from_cal=False) | FAIL | - |
| box_ct | gyro_drift(dps=0.5,over_s=60) | FAIL | - |
| box_ct | accel_bias(axis=x,ms2=0.05) | PASS | - |
| box_ct | accel_bias(axis=x,ms2=0.2) | FAIL | - |
| box_ct | accel_bias(axis=x,ms2=0.5) | FAIL | - |
| box_ct | accel_bias(axis=z,ms2=0.05) | FAIL | - |
| box_ct | accel_bias(axis=z,ms2=0.2) | FAIL | - |
| box_ct | accel_bias(axis=z,ms2=0.5) | FAIL | - |
| box_ct | accel_bias(axis=x,from_cal=False,ms2=0.2) | WARN | - |
| box_ct | mag_bias(frac=0.05) | WARN | - |
| box_ct | mag_bias(frac=0.15) | WARN | - |
| box_ct | mag_interference(amp=0.3) | FAIL | - |
| box_ct | mag_interference(amp=1.0) | FAIL | - |
| box_ct | imu_noise | FAIL | - |
| box_ct | imu_spikes | PASS | - |
| box_ct | imu_dropouts | FAIL | - |
| box_ct | imu_gap(gap_s=0.2) | FAIL | tilt_rms_deg 0.0113->0.0113; tilt_max_deg 0.116->0.0686; yaw_rms_deg 0.0601->0.0601; pos_h_rms_m 0.0066->0.00662; vel_h_rms_ms 0.0152->0.0152; nees_att 0.0141->0.0141; nees_vel 0.0418->0.0418 |
| box_ct | imu_gap(gap_s=1.0) | FAIL | tilt_rms_deg 0.0123->0.0143; tilt_max_deg 0.0925->0.089; yaw_rms_deg 0.0607->0.0619; pos_h_rms_m 0.0182->0.0165; vel_h_rms_ms 0.0153->0.0153; nees_att 0.0142->0.0142; nees_vel 0.0414->0.0414; nees_pos 0.00103->0.000918 |
| box_ct | combined_realistic | PASS | - |
| box_ct | cal_ideal | FAIL | - |
| ref_live1 | none | WARN | - |
