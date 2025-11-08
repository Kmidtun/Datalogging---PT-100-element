Auto generert README av KI - enklere README i prosjekt mappe:

# Datalogging – PT-100-element
Logger temperaturmålinger fra et PT-100-sensor-system via en Nucleo-mikrokontroller, og plott­er dataene i Python.

---

## 📋 Oversikt
Prosjektet består av to programmer som samarbeider: (3 med mikrokontrolleren)

1. **SerialRead.py**  
   Leser serie-data fra mikrokontrolleren og lagrer dem i en CSV-fil.
2. **plotData.py**  
   Leser CSV-filen og plott­er temperatur mot tid.

Formålet er å lære grunnleggende:
- Seriell kommunikasjon
- Filskriving i CSV-format
- Datalesing fra fil
- Plotting i Python (`matplotlib`)

---

## 🛠 Innhold i repoet
| Filnavn | Beskrivelse |
|--------|-------------|
| `SerialRead.py` | Leser `DATA,...` linjer fra seriellport og lagrer til `data.csv` |
| `plotData.py`   | Leser CSV og plott­er temperatur som funksjon av tid |
| `README.md` | Ligger i prosjekt mappen |

---

## 🚀 Hvordan bruke prosjektet

### 1) Oppsett
Installer nødvendige Python-pakker:
```bash
pip install pyserial matplotlib

