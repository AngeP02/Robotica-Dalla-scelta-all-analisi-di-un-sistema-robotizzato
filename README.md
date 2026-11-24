# Robotica - Dalla scelta all'analisi di un sistema robotizzato

Progetto universitario per il corso di **Elementi di Robotica**

**Autori:** Elisabetta Surace, Angelica Porco, Desirè Virginia Fiore, Hanna Andrieieva

### Descrizione

L'obiettivo è trovare il robot industriale più adatto all'automazione di operazioni di **pick and place in ambito farmaceutico** — nello specifico, l'assemblaggio e il confezionamento di scatole contenenti medicinali (blister, flaconi, dispositivi medici). Le masse da manipolare sono contenute (max ~300 g), ma il processo richiede precisione, ripetibilità e affidabilità elevate.

Il lavoro si articola in tre fasi principali:
1. Confronto e selezione del robot più idoneo
2. Analisi cinematica (spazio di lavoro e traiettoria)
3. Analisi statica e dinamica del robot scelto

### Robot confrontati

| Robot | Tipo | Payload | Ripetibilità |
|---|---|---|---|
| **ABB IRB 120** | Industriale (6 DOF) | 3 kg | ±0.01 mm |
| **Kawasaki RS010N** | Industriale (6 DOF) | 10 kg | ±0.04 mm |
| **Universal Robots UR10e** | Collaborativo (6 DOF) | 12.5 kg | ±0.05 mm |

Il robot selezionato è l'**ABB IRB 120**, scelto per il miglior tempo di ammortamento reale (~171 giorni) e per le specifiche di precisione più adatte all'applicazione farmaceutica.

### Fasi

**Selezione del robot**

Confronto tecnico ed economico tra i tre robot. Vengono calcolati:
- **Tempo di ammortamento semplice** (formula base P / kHL)
- **Tempo di ammortamento reale** (considera defiscalizzazione, deprezzamento, costi operativi)
- **ROI** (Return On Investment)

Tutti gli indici sono calcolati assumendo 1 turno/giorno, 8 ore/turno, costo manodopera 22 €/h, 200 giorni lavorativi/anno.

**Analisi dello spazio di lavoro**

Modellazione con la **notazione di Denavit-Hartenberg (D-H)** e calcolo del workspace tramite script MATLAB che itera su tutte le configurazioni ammissibili dei giunti. I parametri D-H dell'ABB IRB 120 sono estratti dalla documentazione ufficiale.

**Analisi statica**

Studio delle forze e coppie agenti sui link in condizione di equilibrio, con il metodo ricorsivo di **Newton-Euler** (versione statica). Carico analizzato: 300 g applicati all'end-effector nella configurazione di massimo sbraccio. Implementato in MATLAB.

**Path Planning**

Pianificazione della traiettoria tra due punti (A = [0.3, 0, 0.4] e B = [0, 0.3, 0.4]):
- Traiettorie **cubiche** nello spazio dei giunti (cinematica diretta)
- Traiettoria tramite **cinematica inversa** con interpolazione spline cubica
- Grafici di posizione, velocità, accelerazione e jerk per ogni giunto

**Analisi dinamica**

Algoritmo completo di Newton-Euler (**Outward + Inward Iteration**) per calcolare forze e coppie durante il movimento. I parametri inerziali (masse, baricentri, momenti di inerzia) sono estratti dal modello URDF ufficiale del pacchetto ROS-Industrial `abb_irb120_support`.

### Risultati principali

- Il robot **ABB IRB 120** risulta il più adatto per l'applicazione farmaceutica considerata, con un tempo di ammortamento reale di ~171 giorni contro i ~202 del Kawasaki e i ~205 del UR10e.
- Lo spazio di lavoro calcolato via D-H è coerente con i dati di targa del costruttore (raggio ~0.58 m).
- L'analisi statica conferma che le coppie richieste agli attuatori rientrano nei limiti operativi dichiarati da ABB.
- La traiettoria cubica garantisce continuità di posizione e velocità; la cinematica inversa migliora l'accuratezza rispetto alla cinematica diretta, tenendo conto dei limiti fisici dei giunti.
- L'analisi dinamica evidenzia i picchi di coppia nei giunti 1 e 2 durante le fasi di accelerazione.
