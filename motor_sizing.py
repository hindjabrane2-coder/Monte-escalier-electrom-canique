"""
Dimensionnement complet du groupe motopropulseur pour monte-escalier.
Charge : fauteuil roulant + utilisateur sur rail incline.
Inclut le calcul dynamique, la selection moteur/reducteur, le bilan
thermique, le dimensionnement batterie et les protections electriques.

Stage EMPEGEC -- Monte-escalier electromecanique
"""
import numpy as np
import matplotlib.pyplot as plt

# Cahier des charges
MASSE_UTILISATEUR = 100       # kg (charge maximale normative)
MASSE_FAUTEUIL = 25           # kg (fauteuil roulant standard)
MASSE_CHARIOT = 15            # kg (chassis + galets + fixations)
MASSE_TOTALE = MASSE_UTILISATEUR + MASSE_FAUTEUIL + MASSE_CHARIOT

ANGLE_ESCALIER = 35           # degres
VITESSE_LINEAIRE = 0.08       # m/s
LONGUEUR_RAIL = 5.0           # m
ACCELERATION = 0.10           # m/s^2 (phase de demarrage)
DECELERATION = 0.12           # m/s^2 (freinage normal)
DECEL_URGENCE = 0.5           # m/s^2 (freinage d'urgence)

# Transmission pignon-cremaillere
D_PIGNON = 40                 # mm
R_PIGNON = D_PIGNON / 2000   # m
MODULE_CREMAILLERE = 2        # mm (module de denture)
Z_PIGNON = int(D_PIGNON / MODULE_CREMAILLERE)  # nombre de dents

# Rendements
ETA_REDUCTEUR = 0.75          # rendement reducteur (vis sans fin)
ETA_CREMAILLERE = 0.90        # rendement pignon-cremaillere
ETA_TOTAL = ETA_REDUCTEUR * ETA_CREMAILLERE
ETA_MOTEUR_ELEC = 0.85        # rendement electrique du moteur DC

# Inerties des pieces tournantes
J_MOTEUR = 1.5e-5             # kg.m^2 (rotor du moteur)
J_PIGNON = 0.5 * 0.3 * R_PIGNON**2   # kg.m^2 (approx disque plein acier)
J_ACCOUPLEMENT = 2e-6         # kg.m^2 (accouplement flexible)

# Constantes
G = 9.81
MU_FROTTEMENT = 0.05
U_BATTERIE = 24               # V (tension nominale du pack)

# Catalogue de moteurs disponibles
# Chaque entree : (reference, puissance_W, couple_Nm, vitesse_rpm, courant_A, prix_eur, masse_kg)
CATALOGUE_MOTEURS = [
    ("MY1016Z  80W",   80,  0.32, 2400,  4.5,  35, 1.2),
    ("MY1016Z 150W",  150,  0.50, 2900,  7.5,  48, 1.5),
    ("ZY6812  200W",  200,  0.64, 3000,  9.8,  62, 1.8),
    ("MY1020  250W",  250,  0.80, 3000, 12.0,  75, 2.0),
    ("MY1025  300W",  300,  0.95, 3000, 14.5,  88, 2.3),
    ("MY1020  350W",  350,  1.10, 3000, 16.5,  95, 2.5),
    ("MY1020  500W",  500,  1.60, 3000, 22.0, 125, 3.1),
]

# Catalogue de reducteurs
# (type, rapport, rendement, prix_eur, masse_kg, bruit_dB)
CATALOGUE_REDUCTEURS = [
    ("Vis sans fin 30:1",      30, 0.65,  80, 1.5, 45),
    ("Vis sans fin 50:1",      50, 0.60, 100, 1.8, 48),
    ("Vis sans fin 80:1",      80, 0.55, 115, 2.2, 50),
    ("Planetaire 2 etages",    36, 0.90, 180, 1.0, 40),
    ("Planetaire 3 etages",    80, 0.85, 250, 1.3, 42),
    ("Engrenages droits 4:1",   4, 0.95,  50, 0.8, 55),
    ("Engrenages droits 10:1", 10, 0.92,  70, 1.2, 58),
]
def efforts_statiques(masse, angle_deg, mu=MU_FROTTEMENT):
    """Forces en regime permanent sur le chariot en montee."""
    angle = np.radians(angle_deg)
    f_gravite = masse * G * np.sin(angle)
    f_normal = masse * G * np.cos(angle)
    f_frottement = mu * f_normal
    return {
        'gravite': f_gravite,
        'frottement': f_frottement,
        'normal': f_normal,
        'total_montee': f_gravite + f_frottement,
        'total_descente': f_gravite - f_frottement,
    }
def force_acceleration(masse, accel, angle_deg):
    """Force supplementaire necessaire pendant l'acceleration."""
    return masse * accel
def inertie_ramenee_moteur(j_moteur, j_pignon, j_accoupl, rapport):
    """Inertie totale ramenee a l'arbre moteur (kg.m^2).
    On ramene l'inertie du pignon a travers le rapport de reduction."""
    j_ramenee_pignon = j_pignon / (rapport ** 2)
    return j_moteur + j_accoupl + j_ramenee_pignon
def couple_acceleration_inertie(j_total, accel_angulaire):
    """Couple necessaire pour accelerer les masses tournantes."""
    return j_total * accel_angulaire
def vitesse_rotation_pignon(v_lineaire, r_pignon):
    """Vitesse de rotation du pignon en tr/min."""
    omega = v_lineaire / r_pignon
    return omega * 60 / (2 * np.pi)
def profil_trapeze(longueur, v_max, accel, decel):
    """Genere un profil de vitesse trapezoidale complet.
    Retourne les tableaux temps, vitesse, position, acceleration."""
    t_acc = v_max / accel
    t_dec = v_max / decel
    d_acc = 0.5 * accel * t_acc ** 2
    d_dec = 0.5 * decel * t_dec ** 2

    if d_acc + d_dec >= longueur:
        # profil triangulaire, la vitesse max n'est pas atteinte
        # on resout pour trouver la vitesse de crete
        v_crete = np.sqrt(2 * longueur * accel * decel / (accel + decel))
        t_acc = v_crete / accel
        t_dec = v_crete / decel
        t_total = t_acc + t_dec
        d_acc = 0.5 * accel * t_acc ** 2
        d_dec = longueur - d_acc
    else:
        d_cruise = longueur - d_acc - d_dec
        t_cruise = d_cruise / v_max
        t_total = t_acc + t_cruise + t_dec

    n_pts = 1000
    t = np.linspace(0, t_total, n_pts)
    v = np.zeros(n_pts)
    pos = np.zeros(n_pts)
    acc = np.zeros(n_pts)

    t_fin_acc = t_acc
    if d_acc + d_dec >= longueur:
        t_fin_cruise = t_acc
    else:
        t_fin_cruise = t_acc + t_cruise
    t_fin_dec = t_total

    for i in range(n_pts):
        ti = t[i]
        if ti <= t_fin_acc:
            v[i] = accel * ti
            pos[i] = 0.5 * accel * ti ** 2
            acc[i] = accel
        elif ti <= t_fin_cruise:
            dt = ti - t_fin_acc
            v_at_end_acc = accel * t_fin_acc
            v[i] = v_at_end_acc
            pos[i] = d_acc + v_at_end_acc * dt
            acc[i] = 0.0
        else:
            dt = ti - t_fin_cruise
            v_at_end_cruise = v[int((t_fin_cruise / t_total) * (n_pts - 1))]
            v[i] = max(v_at_end_cruise - decel * dt, 0)
            pos[i] = d_acc + (d_cruise if d_acc + d_dec < longueur else 0) + \
                     v_at_end_cruise * dt - 0.5 * decel * dt ** 2
            acc[i] = -decel

    return t, v, pos, acc, t_total
def selectionner_moteur(puissance_requise, couple_requis, vitesse_rpm):
    """Parcourt le catalogue et retourne le moteur le plus adapte.
    Le moteur doit fournir au moins la puissance requise avec une marge de 20%."""
    candidats = []
    for ref, p, c, n, i, prix, masse in CATALOGUE_MOTEURS:
        if p >= puissance_requise * 1.2 and c >= couple_requis:
            score = prix + masse * 20  # critere : prix + penalite de masse
            candidats.append((score, ref, p, c, n, i, prix, masse))

    if not candidats:
        # aucun moteur suffisant, prendre le plus gros
        ref, p, c, n, i, prix, masse = CATALOGUE_MOTEURS[-1]
        return ref, p, c, n, i, prix, masse

    candidats.sort(key=lambda x: x[0])
    _, ref, p, c, n, i, prix, masse = candidats[0]
    return ref, p, c, n, i, prix, masse
def comparer_reducteurs(couple_sortie_requis, vitesse_sortie_rpm, vitesse_moteur_rpm):
    """Compare les reducteurs du catalogue et attribue un score."""
    rapport_ideal = vitesse_moteur_rpm / vitesse_sortie_rpm
    resultats = []
    for nom, rapport, eta, prix, masse, bruit in CATALOGUE_REDUCTEURS:
        # le rapport doit etre suffisant (tolerance de 30%)
        if rapport < rapport_ideal * 0.7 or rapport > rapport_ideal * 1.5:
            continue
        couple_entree = couple_sortie_requis / (rapport * eta)
        # score pondere : 40% rendement, 30% prix, 20% bruit, 10% masse
        score = (1 - eta) * 400 + prix * 0.3 + bruit * 0.2 + masse * 10
        resultats.append({
            'nom': nom,
            'rapport': rapport,
            'rendement': eta,
            'couple_entree': couple_entree,
            'prix': prix,
            'masse': masse,
            'bruit': bruit,
            'score': score,
        })
    resultats.sort(key=lambda x: x['score'])
    return resultats
def analyse_thermique_cycle(puissance_pertes, duree_marche, duree_repos,
                             resistance_th=2.5, capacite_th=150.0, t_ambiante=25.0):
    """Modele thermique simplifie du moteur (RC premier ordre).
    resistance_th en K/W, capacite_th en J/K.
    Retourne l'evolution de la temperature sur un cycle complet."""
    tau = resistance_th * capacite_th
    dt = 0.5  # pas de temps en secondes
    duree_totale = duree_marche + duree_repos

    n_pts = int(duree_totale / dt)
    temps = np.linspace(0, duree_totale, n_pts)
    temperature = np.zeros(n_pts)
    temperature[0] = t_ambiante

    for i in range(1, n_pts):
        if temps[i] <= duree_marche:
            q = puissance_pertes
        else:
            q = 0
        d_temp = (q * resistance_th - (temperature[i - 1] - t_ambiante)) / tau * dt
        temperature[i] = temperature[i - 1] + d_temp

    return temps, temperature
def analyse_multi_cycles(puissance_pertes, duree_marche, duree_repos,
                          n_cycles, resistance_th=2.5, capacite_th=150.0,
                          t_ambiante=25.0, t_max=80.0):
    """Simule l'echauffement sur plusieurs allers-retours successifs.
    Retourne la temperature max atteinte et si la limite est depassee."""
    tau = resistance_th * capacite_th
    dt = 0.5
    duree_cycle = duree_marche + duree_repos
    duree_totale = n_cycles * duree_cycle

    n_pts = int(duree_totale / dt)
    temps = np.linspace(0, duree_totale, n_pts)
    temperature = np.zeros(n_pts)
    temperature[0] = t_ambiante

    for i in range(1, n_pts):
        t_dans_cycle = temps[i] % duree_cycle
        q = puissance_pertes if t_dans_cycle <= duree_marche else 0
        d_temp = (q * resistance_th - (temperature[i - 1] - t_ambiante)) / tau * dt
        temperature[i] = temperature[i - 1] + d_temp

    t_max_atteinte = np.max(temperature)
    return temps, temperature, t_max_atteinte, t_max_atteinte > t_max
def dimensionner_batterie(energie_par_trajet_wh, n_trajets, tension_v,
                           dod=0.8, marge=1.2):
    """Calcule la capacite de batterie necessaire.
    dod = profondeur de decharge admissible (80% par defaut).
    marge = facteur de securite."""
    energie_totale = energie_par_trajet_wh * n_trajets * marge
    capacite_ah = energie_totale / (tension_v * dod)
    return {
        'energie_totale_wh': energie_totale,
        'capacite_ah': capacite_ah,
        'tension_v': tension_v,
    }
def energie_regenerative(masse, angle_deg, longueur, eta_regen=0.30):
    """Estime l'energie recuperable en descente par freinage regeneratif.
    Le rendement de recuperation est faible (30%) car le chemin energetique
    passe par le moteur, le controleur, et le chargeur."""
    angle = np.radians(angle_deg)
    f_frein = masse * G * np.sin(angle) - MU_FROTTEMENT * masse * G * np.cos(angle)
    if f_frein <= 0:
        return 0
    e_brute = f_frein * longueur
    return e_brute * eta_regen
def section_conducteur(courant_a, longueur_m, chute_tension_max_pct=3.0,
                        tension_v=24.0, rho_cuivre=1.72e-8):
    """Calcule la section minimale de conducteur en cuivre (mm^2).
    On impose une chute de tension max en pourcentage."""
    chute_v = tension_v * chute_tension_max_pct / 100
    # R = rho * L / S, et U = R * I (aller-retour donc 2*L)
    section_m2 = rho_cuivre * 2 * longueur_m * courant_a / chute_v
    section_mm2 = section_m2 * 1e6
    # arrondir a la section normalisee superieure
    sections_normalisees = [0.5, 0.75, 1.0, 1.5, 2.5, 4.0, 6.0, 10.0, 16.0, 25.0]
    for s in sections_normalisees:
        if s >= section_mm2:
            return s, section_mm2
    return sections_normalisees[-1], section_mm2
def choisir_fusible(courant_nominal, facteur=1.6):
    """Choix du calibre de fusible standard.
    On prend un fusible > courant_nominal * facteur pour eviter
    le declenchement en service normal, tout en protegeant le cablage."""
    calibres = [5, 6, 8, 10, 15, 20, 25, 30, 35, 40, 50]
    seuil = courant_nominal * facteur
    for c in calibres:
        if c >= seuil:
            return c
    return calibres[-1]
def carte_rendement(force_range, vitesse_range, eta_meca, eta_elec):
    """Genere une matrice de rendement global en fonction de la force et de la vitesse.
    En realite le rendement varie avec le point de fonctionnement mais on simplifie
    avec une degradation aux faibles charges et faibles vitesses."""
    f_grid, v_grid = np.meshgrid(force_range, vitesse_range)
    f_norm = f_grid / np.max(force_range)
    v_norm = v_grid / np.max(vitesse_range)
    # le rendement chute aux extremes (faible charge, faible vitesse)
    eta = eta_meca * eta_elec * (0.6 + 0.4 * f_norm) * (0.5 + 0.5 * v_norm)
    eta = np.clip(eta, 0.1, eta_meca * eta_elec)
    return f_grid, v_grid, eta
def freinage_urgence(masse, vitesse, angle_deg, decel_urgence):
    """Calcule la distance et le couple de frein necessaire pour un arret d'urgence."""
    distance = vitesse ** 2 / (2 * decel_urgence)
    angle = np.radians(angle_deg)
    # la gravite aide au freinage en montee
    f_frein_necessaire = masse * decel_urgence + masse * G * np.sin(angle) * 0
    # en fait en montee la gravite aide deja, c'est en descente qu'il faut plus
    f_frein_montee = masse * decel_urgence - masse * G * np.sin(angle)
    f_frein_descente = masse * decel_urgence + masse * G * np.sin(angle)
    # le cas dimensionnant est la descente
    f_frein = max(f_frein_montee, f_frein_descente)
    couple_frein = f_frein * R_PIGNON
    return {
        'distance_m': distance,
        'distance_mm': distance * 1000,
        'force_frein_montee': f_frein_montee,
        'force_frein_descente': f_frein_descente,
        'couple_frein_pignon': couple_frein,
        'temps_arret': vitesse / decel_urgence,
    }
def analyse_sensibilite(masses, angle_deg, vitesse, eta):
    """Etudie l'impact de la masse sur la puissance et le courant moteur."""
    resultats = []
    for m in masses:
        eff = efforts_statiques(m, angle_deg)
        p = eff['total_montee'] * vitesse / eta
        i = p / (U_BATTERIE * ETA_MOTEUR_ELEC)
        resultats.append({
            'masse': m,
            'force': eff['total_montee'],
            'puissance': p,
            'courant': i,
        })
    return resultats
def bilan_energetique_complet(efforts, longueur, eta_red, eta_crem):
    """Bilan detaille de l'energie pour un trajet (montee).
    Fournit les donnees pour un diagramme de Sankey."""
    e_gravite = efforts['gravite'] * longueur
    e_frottement = efforts['frottement'] * longueur
    e_utile = efforts['total_montee'] * longueur

    e_sortie_reducteur = e_utile / eta_crem
    e_entree_reducteur = e_sortie_reducteur / eta_red
    e_pertes_crem = e_sortie_reducteur - e_utile
    e_pertes_red = e_entree_reducteur - e_sortie_reducteur

    e_electrique = e_entree_reducteur / ETA_MOTEUR_ELEC
    e_pertes_elec = e_electrique - e_entree_reducteur

    return {
        'gravite_J': e_gravite,
        'frottement_J': e_frottement,
        'utile_J': e_utile,
        'pertes_cremaillere_J': e_pertes_crem,
        'pertes_reducteur_J': e_pertes_red,
        'pertes_electriques_J': e_pertes_elec,
        'total_electrique_J': e_electrique,
        'total_electrique_Wh': e_electrique / 3600,
    }
def scenarios_utilisation():
    """Definit plusieurs profils d'utilisation pour le dimensionnement."""
    return {
        'leger': {
            'masse_utilisateur': 60,
            'trajets_par_jour': 4,
            'description': 'Utilisateur leger, usage modere',
        },
        'nominal': {
            'masse_utilisateur': 80,
            'trajets_par_jour': 8,
            'description': 'Utilisation standard quotidienne',
        },
        'intensif': {
            'masse_utilisateur': 100,
            'trajets_par_jour': 14,
            'description': 'Charge maximale, usage frequent',
        },
        'maintenance': {
            'masse_utilisateur': 0,
            'trajets_par_jour': 2,
            'description': 'Chariot a vide pour essais et reglages',
        },
    }
def estimation_cout(moteur_prix, reducteur_prix, batterie_ah, prix_ah=8.0):
    """Estimation du cout du groupe motopropulseur."""
    prix_batterie = batterie_ah * prix_ah
    prix_cablage = 25      # estimation forfaitaire
    prix_fusible = 5
    prix_controleur = 40   # carte de puissance
    total = moteur_prix + reducteur_prix + prix_batterie + prix_cablage + \
            prix_fusible + prix_controleur
    return {
        'moteur': moteur_prix,
        'reducteur': reducteur_prix,
        'batterie': prix_batterie,
        'cablage': prix_cablage,
        'fusible': prix_fusible,
        'controleur': prix_controleur,
        'total': total,
    }
def afficher_tableau_specs(specs):
    """Affiche un tableau de specifications formate."""
    largeur = max(len(k) for k in specs) + 4
    print("+" + "-" * (largeur + 22) + "+")
    print("| {:^{}} |".format("FICHE DE SPECIFICATIONS", largeur + 20))
    print("+" + "-" * (largeur + 22) + "+")
    for cle, valeur in specs.items():
        print("| {:<{}} : {:>18} |".format(cle, largeur, str(valeur)))
    print("+" + "-" * (largeur + 22) + "+")
if __name__ == '__main__':
    print("=" * 60)
    print("  DIMENSIONNEMENT MONTE-ESCALIER ELECTROMECANIQUE")
    print("  Stage EMPEGEC")
    print("=" * 60)

    # Efforts statiques
    eff = efforts_statiques(MASSE_TOTALE, ANGLE_ESCALIER)
    print(f"\nMasse totale      : {MASSE_TOTALE} kg")
    print(f"Angle escalier    : {ANGLE_ESCALIER} deg")
    print(f"Force gravitaire  : {eff['gravite']:.1f} N")
    print(f"Force frottement  : {eff['frottement']:.1f} N")
    print(f"Force en montee   : {eff['total_montee']:.1f} N")
    print(f"Force en descente : {eff['total_descente']:.1f} N")

    # Forces dynamiques
    f_accel = force_acceleration(MASSE_TOTALE, ACCELERATION, ANGLE_ESCALIER)
    f_total_dyn = eff['total_montee'] + f_accel
    print(f"\nForce d'acceleration : {f_accel:.1f} N")
    print(f"Force totale dynamique (montee + accel) : {f_total_dyn:.1f} N")

    # Cinematique du pignon
    n_pignon = vitesse_rotation_pignon(VITESSE_LINEAIRE, R_PIGNON)
    c_pignon_statique = eff['total_montee'] * R_PIGNON / ETA_CREMAILLERE
    c_pignon_dynamique = f_total_dyn * R_PIGNON / ETA_CREMAILLERE
    print(f"\nVitesse pignon       : {n_pignon:.1f} tr/min")
    print(f"Couple pignon (stat) : {c_pignon_statique:.2f} N.m")
    print(f"Couple pignon (dyn)  : {c_pignon_dynamique:.2f} N.m")

    # Comparaison des reducteurs
    N_MOTEUR_REF = 3000
    print("\n--- Comparaison des reducteurs ---")
    classement_red = comparer_reducteurs(c_pignon_dynamique, n_pignon, N_MOTEUR_REF)
    if classement_red:
        for i, r in enumerate(classement_red):
            print(f"  {i+1}. {r['nom']:<28} eta={r['rendement']:.2f}  "
                  f"rapport={r['rapport']}:1  {r['prix']} EUR  score={r['score']:.0f}")
        meilleur_red = classement_red[0]
    else:
        print("  Aucun reducteur convenable trouve, utilisation du vis sans fin 50:1")
        meilleur_red = {'nom': 'Vis sans fin 50:1', 'rapport': 50,
                        'rendement': 0.60, 'prix': 100, 'masse': 1.8}
    rapport = meilleur_red['rapport']
    eta_red_choisi = meilleur_red['rendement']
    eta_total_reel = eta_red_choisi * ETA_CREMAILLERE

    # Inertie et couple d'acceleration
    j_total = inertie_ramenee_moteur(J_MOTEUR, J_PIGNON, J_ACCOUPLEMENT, rapport)
    omega_moteur = N_MOTEUR_REF * 2 * np.pi / 60
    accel_angulaire_moteur = ACCELERATION / R_PIGNON * rapport
    c_inertie = couple_acceleration_inertie(j_total, accel_angulaire_moteur)
    print(f"\nInertie ramenee moteur : {j_total:.2e} kg.m^2")
    print(f"Couple d'inertie       : {c_inertie:.4f} N.m")

    # Couple et puissance moteur
    c_moteur_statique = c_pignon_statique / (rapport * eta_red_choisi)
    c_moteur_dyn = c_pignon_dynamique / (rapport * eta_red_choisi) + c_inertie
    p_moteur = f_total_dyn * VITESSE_LINEAIRE / eta_total_reel
    print(f"\nCouple moteur (stat)   : {c_moteur_statique:.4f} N.m")
    print(f"Couple moteur (dyn)    : {c_moteur_dyn:.4f} N.m")
    print(f"Puissance mecanique    : {p_moteur:.0f} W")

    # Selection du moteur
    ref_m, p_m, c_m, n_m, i_m, prix_m, masse_m = selectionner_moteur(
        p_moteur, c_moteur_dyn, N_MOTEUR_REF)
    print(f"\nMoteur selectionne : {ref_m}")
    print(f"  Puissance nominale : {p_m} W")
    print(f"  Couple nominal     : {c_m} N.m")
    print(f"  Vitesse nominale   : {n_m} tr/min")
    print(f"  Courant nominal    : {i_m} A")
    print(f"  Prix               : {prix_m} EUR")

    # Profil de vitesse
    t_arr, v_arr, pos_arr, acc_arr, t_total = profil_trapeze(
        LONGUEUR_RAIL, VITESSE_LINEAIRE, ACCELERATION, DECELERATION)
    print(f"\nDuree du trajet : {t_total:.1f} s ({t_total/60:.1f} min)")

    # Courant et protections
    i_nominal = p_moteur / (U_BATTERIE * ETA_MOTEUR_ELEC)
    i_demarrage = i_nominal * 2.5  # appel de courant au demarrage
    section, section_calc = section_conducteur(i_nominal, 8.0)
    fusible = choisir_fusible(i_nominal)
    print(f"\nCourant nominal    : {i_nominal:.1f} A")
    print(f"Courant demarrage  : {i_demarrage:.1f} A (estime)")
    print(f"Section conducteur : {section} mm^2 (calcul: {section_calc:.2f} mm^2)")
    print(f"Fusible            : {fusible} A")

    # Freinage d'urgence
    frein = freinage_urgence(MASSE_TOTALE, VITESSE_LINEAIRE,
                              ANGLE_ESCALIER, DECEL_URGENCE)
    print(f"\nFreinage d'urgence :")
    print(f"  Distance d'arret      : {frein['distance_mm']:.1f} mm")
    print(f"  Temps d'arret         : {frein['temps_arret']:.3f} s")
    print(f"  Couple frein (pignon) : {frein['couple_frein_pignon']:.2f} N.m")

    # Bilan energetique
    bilan = bilan_energetique_complet(eff, LONGUEUR_RAIL, eta_red_choisi, ETA_CREMAILLERE)
    print(f"\nBilan energetique (1 montee) :")
    print(f"  Energie gravite       : {bilan['gravite_J']:.0f} J")
    print(f"  Energie frottement    : {bilan['frottement_J']:.0f} J")
    print(f"  Pertes cremaillere    : {bilan['pertes_cremaillere_J']:.0f} J")
    print(f"  Pertes reducteur      : {bilan['pertes_reducteur_J']:.0f} J")
    print(f"  Pertes electriques    : {bilan['pertes_electriques_J']:.0f} J")
    print(f"  Total electrique      : {bilan['total_electrique_J']:.0f} J ({bilan['total_electrique_Wh']:.2f} Wh)")

    # Energie regenerative
    e_regen = energie_regenerative(MASSE_TOTALE, ANGLE_ESCALIER, LONGUEUR_RAIL)
    print(f"  Energie recuperable (descente) : {e_regen:.0f} J ({e_regen/3600:.2f} Wh)")

    # Batterie
    e_par_trajet_wh = bilan['total_electrique_Wh']
    n_trajets_cible = 20  # nombre de trajets sur une charge complete
    bat = dimensionner_batterie(e_par_trajet_wh, n_trajets_cible, U_BATTERIE)
    print(f"\nBatterie ({n_trajets_cible} trajets) :")
    print(f"  Energie necessaire : {bat['energie_totale_wh']:.1f} Wh")
    print(f"  Capacite           : {bat['capacite_ah']:.1f} Ah")

    # Analyse thermique
    pertes_moteur = p_moteur * (1 - ETA_MOTEUR_ELEC)
    duree_marche = t_total
    duree_repos = 120  # 2 minutes de pause entre deux trajets
    t_therm, temp_therm = analyse_thermique_cycle(pertes_moteur, duree_marche, duree_repos)
    t_max_1cycle = np.max(temp_therm)
    print(f"\nAnalyse thermique (1 cycle) :")
    print(f"  Pertes moteur    : {pertes_moteur:.1f} W")
    print(f"  Temperature max  : {t_max_1cycle:.1f} C")

    # multi-cycles
    t_multi, temp_multi, t_max_multi, depasse = analyse_multi_cycles(
        pertes_moteur, duree_marche, duree_repos, n_cycles=10)
    print(f"  Temperature apres 10 cycles : {t_max_multi:.1f} C {'(DEPASSEMENT)' if depasse else '(OK)'}")

    # Scenarios
    print("\n--- Scenarios d'utilisation ---")
    scenarios = scenarios_utilisation()
    for nom, params in scenarios.items():
        m_tot = params['masse_utilisateur'] + MASSE_FAUTEUIL + MASSE_CHARIOT
        eff_s = efforts_statiques(m_tot, ANGLE_ESCALIER)
        p_s = eff_s['total_montee'] * VITESSE_LINEAIRE / eta_total_reel
        i_s = p_s / (U_BATTERIE * ETA_MOTEUR_ELEC)
        print(f"  {nom:<12} : {m_tot:>5} kg, {p_s:>6.0f} W, {i_s:>5.1f} A  ({params['description']})")

    # Sensibilite
    masses_sensib = np.arange(80, 200, 10)
    sensib = analyse_sensibilite(masses_sensib, ANGLE_ESCALIER,
                                  VITESSE_LINEAIRE, eta_total_reel)

    # Cout
    bat_ah_arrondi = int(np.ceil(bat['capacite_ah']))
    cout = estimation_cout(prix_m, meilleur_red['prix'], bat_ah_arrondi)
    print(f"\n--- Estimation des couts ---")
    for poste, montant in cout.items():
        if poste != 'total':
            print(f"  {poste:<14} : {montant:>6.0f} EUR")
    print(f"  {'TOTAL':<14} : {cout['total']:>6.0f} EUR")

    # Tableau recapitulatif
    specs = {
        'Moteur': ref_m,
        'Puissance moteur': f"{p_m} W",
        'Tension': f"{U_BATTERIE} V DC",
        'Courant nominal': f"{i_nominal:.1f} A",
        'Reducteur': meilleur_red['nom'],
        'Rapport reduction': f"{rapport}:1",
        'Pignon': f"D{D_PIGNON} mm, module {MODULE_CREMAILLERE}, {Z_PIGNON} dents",
        'Vitesse lineaire': f"{VITESSE_LINEAIRE*100:.0f} cm/s",
        'Charge max': f"{MASSE_TOTALE} kg",
        'Batterie': f"{U_BATTERIE}V {bat_ah_arrondi}Ah",
        'Fusible': f"{fusible} A",
        'Section cable': f"{section} mm^2",
        'Duree trajet': f"{t_total:.0f} s",
        'Arret urgence': f"{frein['distance_mm']:.0f} mm",
        'Cout estime': f"{cout['total']:.0f} EUR",
    }
    print()
    afficher_tableau_specs(specs)

    # Graphiques
    fig, axes = plt.subplots(3, 3, figsize=(16, 14))

    # 1. Force vs angle
    angles = np.linspace(20, 50, 60)
    forces_montee = [efforts_statiques(MASSE_TOTALE, a)['total_montee'] for a in angles]
    forces_descente = [efforts_statiques(MASSE_TOTALE, a)['total_descente'] for a in angles]
    axes[0, 0].plot(angles, forces_montee, 'b-', linewidth=2, label='Montee')
    axes[0, 0].plot(angles, forces_descente, 'g--', linewidth=2, label='Descente')
    axes[0, 0].axvline(x=ANGLE_ESCALIER, color='r', linestyle=':', label=f'{ANGLE_ESCALIER} deg')
    axes[0, 0].set_xlabel('Angle (deg)')
    axes[0, 0].set_ylabel('Force (N)')
    axes[0, 0].set_title('Force vs pente')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 2. Profil de vitesse
    axes[0, 1].plot(t_arr, v_arr * 100, 'g-', linewidth=2)
    axes[0, 1].set_xlabel('Temps (s)')
    axes[0, 1].set_ylabel('Vitesse (cm/s)')
    axes[0, 1].set_title('Profil de vitesse')
    axes[0, 1].grid(True, alpha=0.3)

    # 3. Position
    axes[0, 2].plot(t_arr, pos_arr, 'b-', linewidth=2)
    axes[0, 2].set_xlabel('Temps (s)')
    axes[0, 2].set_ylabel('Position (m)')
    axes[0, 2].set_title('Position du chariot')
    axes[0, 2].grid(True, alpha=0.3)

    # 4. Puissance vs masse (sensibilite)
    masses_plot = [r['masse'] for r in sensib]
    puiss_plot = [r['puissance'] for r in sensib]
    axes[1, 0].plot(masses_plot, puiss_plot, 'r-o', linewidth=2, markersize=4)
    axes[1, 0].axvline(x=MASSE_TOTALE, color='b', linestyle='--', label=f'{MASSE_TOTALE} kg')
    axes[1, 0].set_xlabel('Masse totale (kg)')
    axes[1, 0].set_ylabel('Puissance (W)')
    axes[1, 0].set_title('Sensibilite a la masse')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 5. Bilan energetique
    labels_bilan = ['Gravite', 'Frottement', 'Pertes\ncremaillere',
                    'Pertes\nreducteur', 'Pertes\nelectriques']
    vals_bilan = [bilan['gravite_J'], bilan['frottement_J'],
                  bilan['pertes_cremaillere_J'], bilan['pertes_reducteur_J'],
                  bilan['pertes_electriques_J']]
    couleurs = ['#4e79a7', '#f28e2b', '#e15759', '#76b7b2', '#59a14f']
    axes[1, 1].bar(labels_bilan, [v / 1000 for v in vals_bilan], color=couleurs)
    axes[1, 1].set_ylabel('Energie (kJ)')
    axes[1, 1].set_title('Repartition des pertes (1 montee)')
    axes[1, 1].grid(True, axis='y', alpha=0.3)

    # 6. Thermique multi-cycles
    axes[1, 2].plot(t_multi / 60, temp_multi, 'r-', linewidth=1.5)
    axes[1, 2].axhline(y=80, color='k', linestyle='--', label='Limite 80 C')
    axes[1, 2].set_xlabel('Temps (min)')
    axes[1, 2].set_ylabel('Temperature (C)')
    axes[1, 2].set_title('Echauffement moteur (10 cycles)')
    axes[1, 2].legend()
    axes[1, 2].grid(True, alpha=0.3)

    # 7. Carte de rendement
    forces_r = np.linspace(100, 900, 30)
    vitesses_r = np.linspace(0.01, 0.12, 30)
    f_g, v_g, eta_map = carte_rendement(forces_r, vitesses_r,
                                         eta_total_reel, ETA_MOTEUR_ELEC)
    cs = axes[2, 0].contourf(f_g, v_g * 100, eta_map * 100, levels=15, cmap='viridis')
    fig.colorbar(cs, ax=axes[2, 0], label='Rendement (%)')
    axes[2, 0].set_xlabel('Force (N)')
    axes[2, 0].set_ylabel('Vitesse (cm/s)')
    axes[2, 0].set_title('Carte de rendement global')
    axes[2, 0].plot(eff['total_montee'], VITESSE_LINEAIRE * 100, 'r*',
                    markersize=15, label='Point nominal')
    axes[2, 0].legend()

    # 8. Comparaison moteurs du catalogue
    refs = [m[0] for m in CATALOGUE_MOTEURS]
    puissances_cat = [m[1] for m in CATALOGUE_MOTEURS]
    ax8 = axes[2, 1]
    bars = ax8.barh(refs, puissances_cat, color='#4e79a7')
    ax8.axvline(x=p_moteur * 1.2, color='r', linestyle='--', label='Besoin x1.2')
    for i, b in enumerate(bars):
        if CATALOGUE_MOTEURS[i][0] == ref_m:
            b.set_color('#e15759')
    ax8.set_xlabel('Puissance (W)')
    ax8.set_title('Catalogue moteurs')
    ax8.legend()
    ax8.grid(True, axis='x', alpha=0.3)

    # 9. Repartition des couts
    postes = [k for k in cout if k != 'total']
    valeurs_cout = [cout[k] for k in postes]
    axes[2, 2].pie(valeurs_cout, labels=postes, autopct='%1.0f%%',
                   colors=couleurs + ['#edc948'])
    axes[2, 2].set_title(f'Repartition des couts ({cout["total"]:.0f} EUR)')

    plt.suptitle('Dimensionnement moteur -- Monte-escalier electromecanique', fontsize=14)
    plt.tight_layout()
    plt.savefig('motor_sizing.png', dpi=150)
    print("\nFigure sauvegardee : motor_sizing.png")
