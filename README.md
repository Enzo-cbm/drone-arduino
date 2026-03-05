# Drone quadricoptère (Arduino Uno R3) — Contrôle & stabilisation (MPU6050 + Mahony + PID)

Contrôleur de vol “from scratch” sur Arduino Uno R3 pour un quadricoptère en mode stabilisé : estimation d’orientation via quaternions (filtre de Mahony), régulation PID sur vitesses angulaires, lecture radio par interruptions, génération PWM ESC optimisée via accès direct aux ports.

-------------------------------------------------------------------------------

## Démo
- Vidéo : (à ajouter)
- Photos : (à ajouter)

-------------------------------------------------------------------------------

## Fonctionnalités clés
- IMU MPU6050 (I²C, lecture accel + gyro)
- Fusion capteurs : filtre de Mahony (quaternions + renormalisation rapide fastInvSqrt)
- Angles : conversion quaternion → Euler (roll/pitch/yaw en degrés)
- Contrôle : PID sur axes (roll/pitch/yaw) en vitesse angulaire (°/s) + mode stabilisé (retour à l’horizontale)
- Radio : acquisition des 4 voies RC par Pin Change Interrupt (PCINT0, pins 8–11)
- ESC / moteurs : génération PWM en µs via PORTD (pins 4–7) + temporisation micros()
- Temps réel : boucle stabilisation ~200 Hz (période ~5 ms)
- Fail-safe : coupure moteurs si signal RC hors [900, 2100] µs

-------------------------------------------------------------------------------

## Vue d’ensemble système

### Dataflow (contrôle)
Radio (PWM) → interruptions → duree_impulsion[] → calcul_consigne()
IMU → lecture_MPU() → calcul_angle_fusion() → Mahony (quaternion q) → angle[]
Consignes + gyro → calcul_commande_pid() → correctifs correct_PID[]
Mixage moteurs → calculer_impulsions_ESC() → PWM ESC → generer_impulsion_ESC()

-------------------------------------------------------------------------------

## Schéma — top view (configuration X)

Numérotation moteurs & sens (top view)
- 1 = avant droit, CCW
- 2 = arrière droit, CW
- 3 = arrière gauche, CCW
- 4 = avant gauche, CW

                 AVANT
                   ^
                   |
        (4) CW              (1) CCW
             \               /
              \             /
               \           /
                \         /
                 \       /
                  \     /
                   \   /
                    \ /
                    / \
                   /   \
                  /     \
                 /       \
                /         \
               /           \
        (3) CCW             (2) CW
                   |
                   v
                 ARRIÈRE

### Mixage utilisé dans le code (même ordre que les sorties ESC)
- ESC1 = gaz + pitch − roll + yaw
- ESC2 = gaz − pitch − roll − yaw
- ESC3 = gaz − pitch + roll + yaw
- ESC4 = gaz + pitch + roll − yaw

Si le câblage moteurs (1–4) ne correspond pas à ESC1–ESC4 (pins 4–7), ajuster le mapping (voir section “Câblage ESC / moteurs”).

-------------------------------------------------------------------------------

## Schéma électrique (montage)

                          LiPo 3S 3300 mAh
                         +               -
                         |               |
                         |               |
              +----------+---------------+----------+
              |           Distribution / châssis    |
              +-----+-----------+-----------+-------+
                    |           |           |
                   +|          +|          +|          +|
                 +--+--+     +--+--+     +--+--+     +--+--+
                 | ESC1|     | ESC2|     | ESC3|     | ESC4|
                 |30A  |     |30A  |     |30A  |     |30A  |
                 +--+--+     +--+--+     +--+--+     +--+--+
                    |           |           |           |
                 Motor1       Motor2      Motor3      Motor4
               (avant droit) (arrière d) (arrière g) (avant g)
                  CCW           CW         CCW          CW

                 Filtrage alim (près du BEC / distribution)
                   LiPo + ----+----[ C1 ]----+
                           |                |
                           +----[ C2 ]------+---- LiPo -

                             Alimentation électronique
                   LiPo + --------------------+
                                             |
                                          +--+--+
                                          | BEC |
                                          | 5V  |
                                          +--+--+
                                             |
                                             +-----> Arduino 5V
                   LiPo - -------------------+-----> Arduino GND

Masse commune obligatoire :
Arduino GND ↔ GND ESC ↔ GND récepteur ↔ GND MPU6050

-------------------------------------------------------------------------------

## Matériel

### Électronique principale
- Carte : Arduino Uno R3
- IMU : MPU6050 (adresse I²C 0x68)
- Batterie : LiPo 3S 3300 mAh
- ESC : KAVAN R-30B PLUS BEC 30A (x4)
- Moteurs : KAVAN Brushless motor C2836-1120 (x4)
- Hélices : 10"
- Alimentation Arduino : BEC 5V entre batterie et Arduino
- Filtrage alimentation : 2 condensateurs en parallèle (placés avant le BEC, côté alimentation)

### Distribution d’énergie (principe)
La batterie alimente :
1) les 4 ESC (via le châssis / distribution)
2) l’Arduino via un BEC 5V, avec 2 condensateurs en parallèle pour réduire bruit/creux de tension

-------------------------------------------------------------------------------

## Câblage (recommandé)

### ESC (PWM)
Le code génère le PWM via PORTD :
- ESC1 → D4
- ESC2 → D5
- ESC3 → D6
- ESC4 → D7

### Radio (PWM par voie) — interrupts PCINT0
Le code lit 4 voies via pins 8–11 (PORTB) :
- Canal 1 (roll)  → D8  (PB0)
- Canal 2 (pitch) → D9  (PB1)
- Canal 3 (gaz)   → D10 (PB2)
- Canal 4 (yaw)   → D11 (PB3)

Masse commune obligatoire : GND récepteur ↔ GND Arduino ↔ GND ESC

### MPU6050 (I²C)
- SDA → A4
- SCL → A5
- VCC → 5V (ou 3.3V selon module)
- GND → GND

Bus I²C accéléré via TWBR = 2 (~800 kHz)

-------------------------------------------------------------------------------

## Paramètres & performances

### Temps réel
- Boucle principale verrouillée à ~5200 µs :
  while(micros() - debut_loop < 5200);
- Fréquence ≈ 200 Hz

### Mahony
- Mahony_Kp = 2.0
- Mahony_Ki = 0.05
- intégrale bias : integralFBx/y/z

### PID (valeurs actuelles)
- Roll/Pitch : kp=0.475, ki=0.017, kd=1.95
- Yaw : kp=2.75, ki=0.02, kd=0.00
- Anti-windup : decay + seuils (INTEGRAL_DECAY_*, INTEGRAL_THRESHOLD)

### Sécurité / fail-safe
Dans loop() :
- si une impulsion RC est < 900 µs ou > 2100 µs → etat=ARRET, stopper_moteur(), LED ON, return

-------------------------------------------------------------------------------

## Modes & logique de vol

### États
- ARRET : moteurs coupés
- ARME : armé (sécurité)
- MARCHE : vol actif (PID + mixage)

### Armement / désarmement (selon le code)
- ARRET → ARME : gaz bas + yaw bas
- ARME → MARCHE : gaz bas + yaw au milieu/haut
- MARCHE → ARRET : gaz bas + yaw très haut

-------------------------------------------------------------------------------

## Organisation du code (fonctions principales)

### IMU
- initialisation_MPU() : configure gyro, accel, DLPF
- calibrer_MPU() : 2000 mesures offsets
- lecture_MPU() : lecture accel/gyro, convention d’axes

### Estimation attitude (quaternion)
- pid_mahony_angle(...) : Mahony (PI) + intégration quaternion
- quat_normalize() : renormalisation via fastInvSqrt
- get_euler_angles_from_quaternion() : angles Euler

### Radio (interruptions)
- configuration() : active PCINT0 sur PB0..PB3
- ISR(PCINT0_vect) : mesure largeur impulsions (front montant/descendant)

### Contrôle
- calcul_consigne() : map RC → consignes en °/s + stabilisation (retour horizontal)
- calcul_commande_pid() : PID + anti-windup
- calculer_impulsions_ESC() : mixage quad
- generer_impulsion_ESC() : PWM ESC optimisé (ports)

-------------------------------------------------------------------------------

## Compilation & upload
1. Ouvrir dans Arduino IDE
2. Sélectionner : Arduino Uno
3. Téléverser

### Debug temps de boucle (option)
- (à ajouter)

-------------------------------------------------------------------------------

## Calibration & tuning

### Calibration IMU
- Au boot : laisser le drone immobile pendant calibrer_MPU() (2000 mesures)
- Le code ajuste acceleration_offset[Z] -= 4096 (compensation gravité)

### PID (méthode simple)
1) P uniquement : augmenter jusqu’à réaction rapide sans oscillations fortes
2) Ajouter D : réduire oscillations / overshoot
3) Ajouter I : corriger dérive lente (très petit)

### Mahony
- Si vibrations/bruit : réduire Mahony_Kp ou filtrer accel davantage
- Si dérive : ajuster Mahony_Ki

-------------------------------------------------------------------------------

## Câblage ESC / moteurs — point critique
Numérotation moteurs (mécanique) et numérotation ESC (logicielle) doivent correspondre.
- faire correspondre ESC1..ESC4 (pins D4..D7) aux moteurs physiques (1..4)
- sinon, adapter le mixage dans calculer_impulsions_ESC()

### Test au banc (hélices retirées)
- appliquer une petite consigne roll/pitch et vérifier que les bons moteurs accélèrent

-------------------------------------------------------------------------------

## Limitations connues
- Sans magnétomètre, le yaw peut dériver (gyro/accel uniquement)
- Arduino Uno : budget CPU limité

-------------------------------------------------------------------------------

## Roadmap / TODO
- Optimisation : réduire temps loop (<5000 µs), ajuster deadband, affiner coefficients
- Exécuter Mahony une itération sur deux (option) pour gagner du temps CPU
- Migration STM32 : objectif boucle ~1.5 ms (≈ 650 Hz) + architecture modulaire

-------------------------------------------------------------------------------

## Arborescence du dépôt

```
drone-arduino/
│
├── arduino/
│   └── flight_controller.ino
│
├── docs/
│   └── wiring.md
│
├── assets/
│   └── drone_photo.jpg
│
└── README.md
```

-------------------------------------------------------------------------------

## Sécurité
- Retirer les hélices pour les premiers tests
- Tester armement/désarmement au sol
- Vérifier fail-safe (coupure si signal RC absent)
- Vérifier masses communes (Arduino/ESC/récepteur)

-------------------------------------------------------------------------------

## Licence
À définir

## Contact
Enzo Coranson-Beaudu — Enzomarcuscb@gmail.com
