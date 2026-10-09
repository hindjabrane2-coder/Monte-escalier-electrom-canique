"""
Interface de supervision du monte-escalier electromecanique.
Communication serie avec l'Arduino Mega, affichage temps reel
de l'etat, des capteurs et de l'historique des trajets.

Stage EMPEGEC
"""
import serial
import serial.tools.list_ports
import time
import csv
import os
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from threading import Thread, Lock
from datetime import datetime
from collections import deque

try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_OK = True
except ImportError:
    MATPLOTLIB_OK = False
# Taille de l'historique des mesures pour les graphiques
TAILLE_HISTORIQUE = 300

# Descriptions des codes d'erreur (meme table que dans le firmware)
CODES_ERREUR = {
    'E00': 'Aucune erreur',
    'E01': 'Surintensite moteur',
    'E02': 'Court-circuit detecte',
    'E03': 'Fins de course incoherents',
    'E04': 'Surchauffe moteur',
    'E05': 'Batterie vide',
    'E06': 'Encodeur bloque',
    'E07': 'Timeout communication',
    'E08': 'Reset watchdog',
    'E09': 'Defaut frein',
    'E10': 'Echec autotest',
}
class MonteEscalierGUI:

    def __init__(self, port=None, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.lock_serie = Lock()

        # donnees temps reel
        self.etat_systeme = 'INCONNU'
        self.fc_haut = False
        self.fc_bas = False
        self.pwm = 0
        self.courant_adc = 0
        self.temperature = 0
        self.tension_batt = 0
        self.position_mm = 0
        self.derniere_erreur = 'E00'

        # historiques pour les graphiques
        self.hist_temps = deque(maxlen=TAILLE_HISTORIQUE)
        self.hist_courant = deque(maxlen=TAILLE_HISTORIQUE)
        self.hist_temperature = deque(maxlen=TAILLE_HISTORIQUE)
        self.hist_tension = deque(maxlen=TAILLE_HISTORIQUE)
        self.t0 = time.time()

        # journal des trajets
        self.trajets = []

        # journal des alarmes
        self.alarmes = []

        # tentatives de reconnexion
        self.reconnexion_active = False
        self.tentatives_reconnexion = 0

        self.construire_fenetre()

    def construire_fenetre(self):
        self.root = tk.Tk()
        self.root.title("Monte-escalier -- Supervision EMPEGEC")
        self.root.geometry("950x700")
        self.root.minsize(800, 600)
        self.root.protocol("WM_DELETE_WINDOW", self.quitter)

        # appliquer un theme ttk
        style = ttk.Style()
        themes_dispo = style.theme_names()
        if 'clam' in themes_dispo:
            style.theme_use('clam')
        elif 'alt' in themes_dispo:
            style.theme_use('alt')

        # style personnalise pour les boutons d'arret
        style.configure('Arret.TButton', foreground='white', background='#cc3333')
        style.configure('Maintenance.TButton', foreground='white', background='#336699')

        self.construire_barre_connexion()
        self.construire_onglets()
        self.construire_barre_etat()

        # raccourcis clavier
        self.root.bind('<F5>', lambda e: self.demander_statut())
        self.root.bind('<Escape>', lambda e: self.cmd_stop())
        self.root.bind('<F1>', lambda e: self.afficher_a_propos())
        self.root.bind('<Control-e>', lambda e: self.exporter_csv())
        self.root.bind('<Control-q>', lambda e: self.quitter())

    # Barre de connexion

    def construire_barre_connexion(self):
        frm = ttk.Frame(self.root, padding=4)
        frm.pack(fill='x')

        ttk.Label(frm, text="Port :").pack(side='left', padx=(5, 2))

        self.var_port = tk.StringVar()
        self.combo_port = ttk.Combobox(frm, textvariable=self.var_port,
                                        width=18, state='readonly')
        self.combo_port.pack(side='left', padx=2)
        self.rafraichir_ports()

        ttk.Button(frm, text="Rafraichir", command=self.rafraichir_ports,
                   width=9).pack(side='left', padx=2)

        self.btn_conn = ttk.Button(frm, text="Connecter", command=self.connecter)
        self.btn_conn.pack(side='left', padx=5)

        self.var_reconnect = tk.BooleanVar(value=True)
        ttk.Checkbutton(frm, text="Reconnexion auto",
                        variable=self.var_reconnect).pack(side='left', padx=5)

        self.lbl_statut_conn = ttk.Label(frm, text="Deconnecte", foreground='red')
        self.lbl_statut_conn.pack(side='right', padx=10)

    def rafraichir_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port['values'] = ports
        if ports:
            if self.port and self.port in ports:
                self.var_port.set(self.port)
            else:
                self.var_port.set(ports[0])

    # Onglets

    def construire_onglets(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Onglet 1 : Controle
        self.tab_controle = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_controle, text="  Controle  ")
        self.construire_tab_controle()

        # Onglet 2 : Supervision (graphiques)
        self.tab_supervision = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_supervision, text="  Supervision  ")
        self.construire_tab_supervision()

        # Onglet 3 : Configuration
        self.tab_config = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_config, text="  Configuration  ")
        self.construire_tab_configuration()

        # Onglet 4 : Historique
        self.tab_historique = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_historique, text="  Historique  ")
        self.construire_tab_historique()

    # Onglet Controle

    def construire_tab_controle(self):
        frm_gauche = ttk.Frame(self.tab_controle)
        frm_gauche.pack(side='left', fill='both', expand=True, padx=5, pady=5)

        frm_droite = ttk.Frame(self.tab_controle)
        frm_droite.pack(side='right', fill='both', expand=True, padx=5, pady=5)

        # tableau de bord
        frm_etat = ttk.LabelFrame(frm_gauche, text="Etat du systeme", padding=8)
        frm_etat.pack(fill='x', pady=3)

        self.var_etat = tk.StringVar(value="--")
        self.var_fc_haut = tk.StringVar(value="--")
        self.var_fc_bas = tk.StringVar(value="--")
        self.var_pwm = tk.StringVar(value="--")
        self.var_courant = tk.StringVar(value="--")
        self.var_temp = tk.StringVar(value="--")
        self.var_batt = tk.StringVar(value="--")
        self.var_position = tk.StringVar(value="--")
        self.var_erreur = tk.StringVar(value="--")

        indicateurs = [
            ("Etat", self.var_etat),
            ("FC haut", self.var_fc_haut),
            ("FC bas", self.var_fc_bas),
            ("PWM", self.var_pwm),
            ("Courant (ADC)", self.var_courant),
            ("Temperature", self.var_temp),
            ("Batterie", self.var_batt),
            ("Position", self.var_position),
            ("Erreur", self.var_erreur),
        ]
        for i, (label, var) in enumerate(indicateurs):
            ttk.Label(frm_etat, text=label + " :").grid(
                row=i, column=0, sticky='w', padx=4, pady=1)
            lbl = ttk.Label(frm_etat, textvariable=var, font=('Consolas', 10, 'bold'))
            lbl.grid(row=i, column=1, sticky='w', padx=8, pady=1)

        # indicateur visuel (gros cercle de couleur)
        self.canvas_etat = tk.Canvas(frm_etat, width=50, height=50,
                                      highlightthickness=0)
        self.canvas_etat.grid(row=0, column=2, rowspan=3, padx=10)
        self.cercle_etat = self.canvas_etat.create_oval(5, 5, 45, 45, fill='gray',
                                                          outline='#333', width=2)

        # commandes
        frm_cmd = ttk.LabelFrame(frm_gauche, text="Commandes", padding=8)
        frm_cmd.pack(fill='x', pady=3)

        ligne1 = ttk.Frame(frm_cmd)
        ligne1.pack(fill='x', pady=3)
        ttk.Button(ligne1, text="MONTEE", command=self.cmd_montee,
                   width=12).pack(side='left', padx=3)
        ttk.Button(ligne1, text="DESCENTE", command=self.cmd_descente,
                   width=12).pack(side='left', padx=3)
        ttk.Button(ligne1, text="ARRET", command=self.cmd_stop,
                   style='Arret.TButton', width=12).pack(side='left', padx=3)

        ligne2 = ttk.Frame(frm_cmd)
        ligne2.pack(fill='x', pady=3)
        ttk.Button(ligne2, text="RESET", command=self.cmd_reset,
                   width=12).pack(side='left', padx=3)
        ttk.Button(ligne2, text="Maintenance", command=self.cmd_maintenance,
                   style='Maintenance.TButton', width=12).pack(side='left', padx=3)
        ttk.Button(ligne2, text="Statut", command=self.demander_statut,
                   width=12).pack(side='left', padx=3)

        # visualisation du rail (canvas)
        frm_visu = ttk.LabelFrame(frm_droite, text="Position sur le rail", padding=5)
        frm_visu.pack(fill='both', expand=True, pady=3)
        self.construire_visu_rail(frm_visu)

        # alarmes actives
        frm_alarmes = ttk.LabelFrame(frm_droite, text="Alarmes actives", padding=5)
        frm_alarmes.pack(fill='x', pady=3)
        self.lst_alarmes = tk.Listbox(frm_alarmes, height=4,
                                       font=('Consolas', 9), fg='red')
        self.lst_alarmes.pack(fill='x')

        # journal texte
        frm_log = ttk.LabelFrame(frm_gauche, text="Journal", padding=5)
        frm_log.pack(fill='both', expand=True, pady=3)
        self.txt_log = tk.Text(frm_log, height=6, font=('Consolas', 9),
                                state='disabled', wrap='word')
        scroll_log = ttk.Scrollbar(frm_log, orient='vertical',
                                    command=self.txt_log.yview)
        self.txt_log.configure(yscrollcommand=scroll_log.set)
        self.txt_log.pack(side='left', fill='both', expand=True)
        scroll_log.pack(side='right', fill='y')

    def construire_visu_rail(self, parent):
        """Canvas montrant le rail incline et la position du chariot."""
        self.canvas_rail = tk.Canvas(parent, width=200, height=280,
                                      bg='#f0f0f0', highlightthickness=0)
        self.canvas_rail.pack(fill='both', expand=True)

        # dessiner le rail (ligne inclinee)
        # coordonnees : bas-gauche vers haut-droite
        self.rail_x1, self.rail_y1 = 40, 260   # bas
        self.rail_x2, self.rail_y2 = 160, 30   # haut

        self.canvas_rail.create_line(self.rail_x1, self.rail_y1,
                                      self.rail_x2, self.rail_y2,
                                      width=4, fill='#666')

        # marqueurs
        self.canvas_rail.create_text(self.rail_x1 - 10, self.rail_y1 + 10,
                                      text="BAS", font=('Arial', 8))
        self.canvas_rail.create_text(self.rail_x2 + 10, self.rail_y2 - 10,
                                      text="HAUT", font=('Arial', 8))

        # le chariot (rectangle sur le rail)
        self.chariot_id = self.canvas_rail.create_rectangle(
            30, 250, 50, 265, fill='#336699', outline='#222', width=2)

        # texte de position
        self.txt_pos_rail = self.canvas_rail.create_text(
            100, 285, text="0 mm", font=('Consolas', 9))

    def maj_position_rail(self, position_mm):
        """Deplace le chariot sur le dessin du rail."""
        longueur_rail_mm = 5000
        ratio = max(0, min(1, position_mm / longueur_rail_mm))

        x = self.rail_x1 + ratio * (self.rail_x2 - self.rail_x1)
        y = self.rail_y1 + ratio * (self.rail_y2 - self.rail_y1)

        self.canvas_rail.coords(self.chariot_id,
                                 x - 10, y - 7, x + 10, y + 7)
        self.canvas_rail.itemconfig(self.txt_pos_rail,
                                     text=f"{position_mm} mm")

    # Onglet Supervision

    def construire_tab_supervision(self):
        if not MATPLOTLIB_OK:
            ttk.Label(self.tab_supervision,
                      text="matplotlib non disponible -- graphiques desactives",
                      font=('Arial', 12)).pack(pady=40)
            return

        self.fig = Figure(figsize=(9, 5), dpi=90)
        self.fig.set_facecolor('#f5f5f5')

        self.ax_courant = self.fig.add_subplot(2, 2, 1)
        self.ax_courant.set_title('Courant moteur', fontsize=9)
        self.ax_courant.set_ylabel('ADC')
        self.ax_courant.grid(True, alpha=0.3)

        self.ax_temp = self.fig.add_subplot(2, 2, 2)
        self.ax_temp.set_title('Temperature moteur', fontsize=9)
        self.ax_temp.set_ylabel('C')
        self.ax_temp.grid(True, alpha=0.3)

        self.ax_batt = self.fig.add_subplot(2, 2, 3)
        self.ax_batt.set_title('Tension batterie', fontsize=9)
        self.ax_batt.set_ylabel('mV')
        self.ax_batt.set_xlabel('Temps (s)')
        self.ax_batt.grid(True, alpha=0.3)

        self.ax_position = self.fig.add_subplot(2, 2, 4)
        self.ax_position.set_title('Position chariot', fontsize=9)
        self.ax_position.set_ylabel('mm')
        self.ax_position.set_xlabel('Temps (s)')
        self.ax_position.grid(True, alpha=0.3)

        self.fig.tight_layout(pad=2)

        self.canvas_fig = FigureCanvasTkAgg(self.fig, master=self.tab_supervision)
        self.canvas_fig.get_tk_widget().pack(fill='both', expand=True)

    def maj_graphiques(self):
        if not MATPLOTLIB_OK or not self.hist_temps:
            return
        t = list(self.hist_temps)

        self.ax_courant.clear()
        self.ax_courant.plot(t, list(self.hist_courant), color='#cc3333', linewidth=1)
        self.ax_courant.set_title('Courant moteur', fontsize=9)
        self.ax_courant.set_ylabel('ADC')
        self.ax_courant.grid(True, alpha=0.3)

        self.ax_temp.clear()
        self.ax_temp.plot(t, list(self.hist_temperature), color='#e67300', linewidth=1)
        self.ax_temp.axhline(80, color='red', linestyle='--', linewidth=0.8)
        self.ax_temp.set_title('Temperature moteur', fontsize=9)
        self.ax_temp.set_ylabel('C')
        self.ax_temp.grid(True, alpha=0.3)

        self.ax_batt.clear()
        self.ax_batt.plot(t, list(self.hist_tension), color='#336699', linewidth=1)
        self.ax_batt.set_title('Tension batterie', fontsize=9)
        self.ax_batt.set_ylabel('mV')
        self.ax_batt.set_xlabel('Temps (s)')
        self.ax_batt.grid(True, alpha=0.3)

        self.ax_position.clear()
        # la position n'est pas dans l'historique courant/tension, on la trace
        # seulement si on a assez de points
        self.ax_position.set_title('Position chariot', fontsize=9)
        self.ax_position.set_ylabel('mm')
        self.ax_position.set_xlabel('Temps (s)')
        self.ax_position.grid(True, alpha=0.3)

        self.fig.tight_layout(pad=2)
        self.canvas_fig.draw_idle()

    # Onglet Configuration

    def construire_tab_configuration(self):
        frm = ttk.Frame(self.tab_config, padding=15)
        frm.pack(fill='both')

        ttk.Label(frm, text="Parametres envoyes a l'Arduino",
                  font=('Arial', 11, 'bold')).grid(row=0, column=0,
                                                      columnspan=3, pady=(0, 10))

        # PWM max
        ttk.Label(frm, text="PWM max :").grid(row=1, column=0, sticky='w', pady=5)
        self.var_cfg_pwm = tk.IntVar(value=200)
        ttk.Spinbox(frm, from_=60, to=255, textvariable=self.var_cfg_pwm,
                     width=8).grid(row=1, column=1, padx=5)
        ttk.Label(frm, text="(60-255)").grid(row=1, column=2, sticky='w')

        # seuil de courant
        ttk.Label(frm, text="Seuil courant :").grid(row=2, column=0,
                                                       sticky='w', pady=5)
        self.var_cfg_seuil_i = tk.IntVar(value=720)
        ttk.Spinbox(frm, from_=550, to=900, textvariable=self.var_cfg_seuil_i,
                     width=8).grid(row=2, column=1, padx=5)
        ttk.Label(frm, text="(valeur ADC)").grid(row=2, column=2, sticky='w')

        # intervalle de polling
        ttk.Label(frm, text="Intervalle polling :").grid(row=3, column=0,
                                                            sticky='w', pady=5)
        self.var_cfg_poll = tk.IntVar(value=500)
        ttk.Spinbox(frm, from_=100, to=5000, increment=100,
                     textvariable=self.var_cfg_poll,
                     width=8).grid(row=3, column=1, padx=5)
        ttk.Label(frm, text="ms").grid(row=3, column=2, sticky='w')

        # boutons
        frm_btn = ttk.Frame(frm)
        frm_btn.grid(row=5, column=0, columnspan=3, pady=15)

        ttk.Button(frm_btn, text="Envoyer PWM max",
                   command=self.envoyer_config_pwm).pack(side='left', padx=5)
        ttk.Button(frm_btn, text="Envoyer seuil courant",
                   command=self.envoyer_config_seuil).pack(side='left', padx=5)
        ttk.Button(frm_btn, text="Lire version",
                   command=self.cmd_version).pack(side='left', padx=5)
        ttk.Button(frm_btn, text="Lire log erreurs",
                   command=self.cmd_log_erreurs).pack(side='left', padx=5)
        ttk.Button(frm_btn, text="Statistiques",
                   command=self.cmd_stats).pack(side='left', padx=5)

        # zone de reponse
        ttk.Label(frm, text="Reponse :").grid(row=6, column=0, sticky='nw', pady=5)
        self.txt_reponse_config = tk.Text(frm, height=8, width=60,
                                           font=('Consolas', 9), state='disabled')
        self.txt_reponse_config.grid(row=6, column=1, columnspan=2,
                                      sticky='nsew', pady=5)

    def envoyer_config_pwm(self):
        val = self.var_cfg_pwm.get()
        self.envoyer(f"CONFIG:PWM_MAX={val}")
        self.log(f"Config envoyee : PWM_MAX={val}")

    def envoyer_config_seuil(self):
        val = self.var_cfg_seuil_i.get()
        self.envoyer(f"CONFIG:SEUIL_I={val}")
        self.log(f"Config envoyee : SEUIL_I={val}")

    def cmd_version(self):
        self.envoyer("VERSION")

    def cmd_log_erreurs(self):
        self.envoyer("LOG")

    def cmd_stats(self):
        self.envoyer("STATS")

    # Onglet Historique

    def construire_tab_historique(self):
        frm_haut = ttk.Frame(self.tab_historique)
        frm_haut.pack(fill='x', padx=5, pady=5)

        ttk.Label(frm_haut, text="Journal des trajets",
                  font=('Arial', 11, 'bold')).pack(side='left')
        ttk.Button(frm_haut, text="Exporter CSV",
                   command=self.exporter_csv).pack(side='right', padx=5)
        ttk.Button(frm_haut, text="Effacer",
                   command=self.effacer_historique).pack(side='right', padx=5)

        colonnes = ('date', 'direction', 'duree_ms', 'courant_max')
        self.tree_trajets = ttk.Treeview(self.tab_historique,
                                          columns=colonnes, show='headings',
                                          height=15)
        self.tree_trajets.heading('date', text='Date/Heure')
        self.tree_trajets.heading('direction', text='Direction')
        self.tree_trajets.heading('duree_ms', text='Duree (ms)')
        self.tree_trajets.heading('courant_max', text='Courant max (ADC)')

        self.tree_trajets.column('date', width=160)
        self.tree_trajets.column('direction', width=100)
        self.tree_trajets.column('duree_ms', width=100)
        self.tree_trajets.column('courant_max', width=130)

        scroll_tree = ttk.Scrollbar(self.tab_historique, orient='vertical',
                                     command=self.tree_trajets.yview)
        self.tree_trajets.configure(yscrollcommand=scroll_tree.set)

        self.tree_trajets.pack(side='left', fill='both', expand=True, padx=(5, 0))
        scroll_tree.pack(side='right', fill='y', padx=(0, 5))

    def ajouter_trajet(self, direction, duree_ms, courant_max):
        """Enregistre un trajet dans l'historique."""
        date = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.trajets.append({
            'date': date,
            'direction': direction,
            'duree_ms': duree_ms,
            'courant_max': courant_max,
        })
        self.tree_trajets.insert('', 'end', values=(
            date, direction, duree_ms, courant_max))

    def effacer_historique(self):
        if messagebox.askyesno("Confirmation", "Effacer tout l'historique ?"):
            self.trajets.clear()
            for item in self.tree_trajets.get_children():
                self.tree_trajets.delete(item)

    def exporter_csv(self):
        if not self.trajets:
            messagebox.showinfo("Export", "Aucun trajet a exporter.")
            return
        chemin = filedialog.asksaveasfilename(
            defaultextension='.csv',
            filetypes=[("CSV", "*.csv"), ("Tous", "*.*")],
            initialfile=f"trajets_{datetime.now().strftime('%Y%m%d')}.csv")
        if not chemin:
            return
        with open(chemin, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=['date', 'direction',
                                                     'duree_ms', 'courant_max'])
            writer.writeheader()
            writer.writerows(self.trajets)
        self.log(f"Export vers {chemin}")

    # Barre d'etat

    def construire_barre_etat(self):
        frm = ttk.Frame(self.root, relief='sunken', padding=2)
        frm.pack(fill='x', side='bottom')
        self.lbl_barre_etat = ttk.Label(
            frm, text="Pret. F5=Statut, Esc=Arret, Ctrl+E=Export, Ctrl+Q=Quitter",
            font=('Arial', 8))
        self.lbl_barre_etat.pack(side='left')

        self.lbl_nb_trajets = ttk.Label(frm, text="Trajets: 0", font=('Arial', 8))
        self.lbl_nb_trajets.pack(side='right', padx=10)

    # Connexion serie

    def connecter(self):
        if self.ser and self.ser.is_open:
            self.deconnecter()
            return

        port = self.var_port.get()
        if not port:
            messagebox.showwarning("Port", "Selectionner un port serie.")
            return

        try:
            self.ser = serial.Serial(port, self.baud, timeout=1)
            time.sleep(2)  # attente du reset Arduino
            self.running = True
            self.tentatives_reconnexion = 0
            self.lbl_statut_conn.config(text=f"Connecte ({port})", foreground='green')
            self.btn_conn.config(text="Deconnecter")
            self.log(f"Connexion ouverte sur {port}")

            # lancer le thread de lecture serie
            t = Thread(target=self.boucle_lecture, daemon=True)
            t.start()

            # demarrer l'interrogation periodique
            self.interroger()

        except serial.SerialException as e:
            self.log(f"Erreur de connexion : {e}")
            if self.var_reconnect.get():
                self.planifier_reconnexion()

    def deconnecter(self):
        self.running = False
        self.reconnexion_active = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.lbl_statut_conn.config(text="Deconnecte", foreground='red')
        self.btn_conn.config(text="Connecter")
        self.log("Deconnecte")

    def planifier_reconnexion(self):
        if self.reconnexion_active:
            return
        self.reconnexion_active = True
        self.tentatives_reconnexion += 1
        delai = min(5000, 1000 * self.tentatives_reconnexion)
        self.lbl_statut_conn.config(
            text=f"Reconnexion dans {delai/1000:.0f}s...", foreground='orange')
        self.root.after(delai, self.tenter_reconnexion)

    def tenter_reconnexion(self):
        self.reconnexion_active = False
        if self.running:
            return  # deja connecte entre-temps
        self.log(f"Tentative de reconnexion ({self.tentatives_reconnexion})")
        self.connecter()

    # Lecture serie (thread separee)

    def boucle_lecture(self):
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    with self.lock_serie:
                        ligne = self.ser.readline().decode('utf-8',
                                                            errors='ignore').strip()
                    if ligne:
                        # planifier le traitement dans le thread principal
                        self.root.after(0, self.traiter_message, ligne)
            except serial.SerialException:
                self.root.after(0, self.perte_connexion)
                break
            except Exception:
                break
            time.sleep(0.02)

    def perte_connexion(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.lbl_statut_conn.config(text="Connexion perdue", foreground='red')
        self.btn_conn.config(text="Connecter")
        self.log("Connexion serie perdue")
        if self.var_reconnect.get():
            self.planifier_reconnexion()

    # Traitement des messages Arduino

    def traiter_message(self, msg):
        # message de statut complet
        if msg.startswith("ETAT:"):
            self.parser_statut(msg)
            return

        # fin de trajet
        if msg.startswith("<<ARRET:"):
            self.parser_fin_trajet(msg)
            return

        # erreur
        if msg.startswith("!ERR:"):
            code = msg[5:]
            desc = CODES_ERREUR.get(code, 'Erreur inconnue')
            self.ajouter_alarme(f"{code} : {desc}")
            self.log(f"ERREUR {code} : {desc}")
            return

        if msg.startswith("!ARU"):
            self.ajouter_alarme("Arret d'urgence active")
            self.log("ARRET D'URGENCE")
            return

        # reponse de configuration ou autres
        if msg.startswith("V:") or msg.startswith("TRAJETS:") or \
           msg.startswith("ERREURS:") or msg.startswith("OK:") or \
           msg.startswith("  E"):
            self.afficher_reponse_config(msg)
            return

        if msg.startswith("BATT:") or msg.startswith("TEMP:") or \
           msg.startswith("POS:"):
            self.afficher_reponse_config(msg)
            return

        if msg.startswith("W:"):
            self.log(f"Avertissement : {msg[2:]}")
            return

        # message generique
        self.log(msg)

    def parser_statut(self, msg):
        """Parse le message de statut : ETAT:xxx,FC_H:x,FC_B:x,..."""
        parties = msg.split(',')
        for p in parties:
            if ':' not in p:
                continue
            cle, val = p.split(':', 1)
            if cle == 'ETAT':
                self.etat_systeme = val
                self.var_etat.set(val)
                couleurs = {
                    'ARRET': 'gray', 'MONTEE': '#33aa33',
                    'DESCENTE': '#3366cc', 'URGENCE': '#cc3333',
                    'MAINT': '#336699', 'RALENTI': '#cc9900',
                }
                self.canvas_etat.itemconfig(
                    self.cercle_etat, fill=couleurs.get(val, 'gray'))
            elif cle == 'FC_H':
                self.fc_haut = (val == '1')
                self.var_fc_haut.set('Actif' if val == '1' else 'Libre')
            elif cle == 'FC_B':
                self.fc_bas = (val == '1')
                self.var_fc_bas.set('Actif' if val == '1' else 'Libre')
            elif cle == 'PWM':
                self.pwm = int(val)
                self.var_pwm.set(val)
            elif cle == 'I':
                self.courant_adc = int(val)
                self.var_courant.set(val)
            elif cle == 'T':
                self.temperature = int(val)
                self.var_temp.set(f"{val} C")
            elif cle == 'V':
                self.tension_batt = int(val)
                self.var_batt.set(f"{val} mV")
            elif cle == 'POS':
                self.position_mm = int(val)
                self.var_position.set(f"{val} mm")
            elif cle == 'ERR':
                self.derniere_erreur = val
                desc = CODES_ERREUR.get(val, '')
                self.var_erreur.set(f"{val} {desc}")

        # mettre a jour la visu
        self.maj_position_rail(self.position_mm)

        # enregistrer dans l'historique des capteurs
        t_rel = time.time() - self.t0
        self.hist_temps.append(t_rel)
        self.hist_courant.append(self.courant_adc)
        self.hist_temperature.append(self.temperature)
        self.hist_tension.append(self.tension_batt)

    def parser_fin_trajet(self, msg):
        """Parse <<ARRET:raison,T=xxxms,Imax=yyy"""
        direction = "?"
        duree = "?"
        i_max = "?"
        parties = msg[8:].split(',')
        for p in parties:
            if p.startswith('T='):
                duree = p[2:].replace('ms', '')
            elif p.startswith('Imax='):
                i_max = p[5:]
            elif p in ('FC_HAUT', 'FC_BAS', 'BTN_RELACHE'):
                if 'HAUT' in p:
                    direction = 'MONTEE'
                elif 'BAS' in p:
                    direction = 'DESCENTE'
                else:
                    direction = self.etat_systeme
        self.ajouter_trajet(direction, duree, i_max)
        self.lbl_nb_trajets.config(text=f"Trajets: {len(self.trajets)}")

    # Alarmes

    def ajouter_alarme(self, texte):
        horodatage = datetime.now().strftime("%H:%M:%S")
        entree = f"[{horodatage}] {texte}"
        self.alarmes.append(entree)
        self.lst_alarmes.insert('end', entree)
        self.lst_alarmes.see('end')
        # garder les 50 dernieres
        if self.lst_alarmes.size() > 50:
            self.lst_alarmes.delete(0)

    # Envoi de commandes

    def envoyer(self, cmd):
        if not self.ser or not self.ser.is_open:
            self.log("Non connecte")
            return
        try:
            with self.lock_serie:
                self.ser.write(f"{cmd}\n".encode())
        except serial.SerialException as e:
            self.log(f"Erreur d'envoi : {e}")

    def cmd_montee(self):
        self.envoyer("MOVE_UP")
        self.log("Commande : MOVE_UP")

    def cmd_descente(self):
        self.envoyer("MOVE_DOWN")
        self.log("Commande : MOVE_DOWN")

    def cmd_stop(self):
        self.envoyer("STOP")
        self.log("Commande : STOP")

    def cmd_reset(self):
        self.envoyer("RESET")
        self.log("Commande : RESET")

    def cmd_maintenance(self):
        self.envoyer("MAINTENANCE")
        self.log("Commande : MAINTENANCE")

    def demander_statut(self):
        self.envoyer("STATUS")

    # Interrogation periodique

    def interroger(self):
        if self.running:
            self.demander_statut()
            # mettre a jour les graphiques toutes les 5 interrogations
            if len(self.hist_temps) % 5 == 0 and MATPLOTLIB_OK:
                self.maj_graphiques()
            intervalle = self.var_cfg_poll.get()
            self.root.after(intervalle, self.interroger)

    # Journal

    def log(self, msg):
        horodatage = datetime.now().strftime("%H:%M:%S")
        self.txt_log.configure(state='normal')
        self.txt_log.insert('end', f"[{horodatage}] {msg}\n")
        self.txt_log.see('end')
        # garder max 500 lignes
        nb_lignes = int(self.txt_log.index('end-1c').split('.')[0])
        if nb_lignes > 500:
            self.txt_log.delete('1.0', '100.0')
        self.txt_log.configure(state='disabled')

    def afficher_reponse_config(self, msg):
        self.txt_reponse_config.configure(state='normal')
        self.txt_reponse_config.insert('end', msg + '\n')
        self.txt_reponse_config.see('end')
        self.txt_reponse_config.configure(state='disabled')

    # Dialogs

    def afficher_a_propos(self):
        messagebox.showinfo("A propos",
            "Monte-escalier -- Interface de supervision\n"
            "Stage EMPEGEC\n\n"
            "Communication serie avec Arduino Mega\n"
            "Protocole : 115200 baud, 8N1\n\n"
            "Raccourcis :\n"
            "  F5 : Statut\n"
            "  Esc : Arret\n"
            "  Ctrl+E : Export CSV\n"
            "  Ctrl+Q : Quitter\n"
            "  F1 : A propos")

    # Fermeture

    def quitter(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()
if __name__ == '__main__':
    app = MonteEscalierGUI()
    app.run()
