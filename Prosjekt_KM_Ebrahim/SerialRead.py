import argparse             #Brukes til å kunne gi info via console
import serial               #Brukes til å åpne og lese fra COM-port
import time                 #Brukes til å gi en liten pause før lesing ved start
from pathlib import Path    #Brukes til å enkelt navigere/bruke filer
import datetime             #Brukes til å hente tiden fra PC-en

## = fungerende kode som er "kommentert" bort, ikke nødvendig
#  = vanlig kommentar

#Programmet leser rett fra COM-port som Nucleo skriver til. 
#Data punktene lagres i definert fil i samme mappe

#Kjører programmet fra console: "python serialRead.py --port" + ("ønsket port")
#Husk å bruke "cd" for å gå til mappen programmet ligger i

def main():
    p = argparse.ArgumentParser(description="Les DATA-linjer fra Nucleo og lagre til CSV.")

    #Start argumenter ved oppstart av program: 

    #Definer hvilken port som skal leses av, krever input for å starte.
    p.add_argument("--port", required=True)

    #Definer baudrate, ikke krevd, default 115200.
    p.add_argument("--baud", type=int, default=115200)

    #Definer hvilken fil det skal lagres i, ikke krevd, default til data.csv.
    p.add_argument("--outfile", default="data.csv")
    
    #Leser verdiene fra bruker.
    args = p.parse_args()

    #Åpner definert port for lesing
    cmd = serial.Serial(args.port, args.baud, timeout=1)    #Åpner med inputen fra bruker i starten
    time.sleep(2)                                           #Kort vent rett etter åpning av port

    #Forberede / sjekker filen for lagring av data
    out = Path(args.outfile)                                #Setter destinasjon for data som logges
    first_write = not out.exists()                          #Sjekker om filen eksisterer

    #Hvis destinasjonen ikke eksisterer -- Lager header til filen
    with out.open("a", newline="") as f:
        if first_write:                                     #Hvis det er sant at filen ikke eksisterer
            f.write("time_s,temp_c,R_cal_ohm,Vpt_V,ADC\n")  #Skriver inn headeren

        #Skriver til konsol hvor dataen lagres + hvordan stoppe logging
        print(f"Logger DATA-linjer til {out.resolve()}  (Ctrl+C for stopp)\n") 
        try:
            while True:
                cmdLogg = cmd.readline()        #Leser input/linjer i console fra mikrokontrolleren
                if not cmdLogg:                 #Hvis linjen er tom gå videre
                    continue
                try:
                    line = cmdLogg.decode(errors="ignore").strip()  #Gjør om bytes til string, ignorerer rare tegn, fjerner whitespace
                except Exception:
                    continue

                #Siden mikrokontrolleren skriver ut "realtime" data hvert 2 sekund
                #er det ikke alt vi vil ha med oss, bare datapunkter som kommer hvert
                #5 sekund. Og disse linjene starter med "DATA".
                #Hele utskriftlinjene er på formatet: "DATA, tid, T, R_cal, Vpt, ADC"
                if line.startswith("DATA,"):                            #Velger rett linjer å ta med
                    try:
                        _, t_s, t_c, r_cal, vpt, adc = line.split(",")  #splitter linjene på komma
                        #Definerer/sjekker datatypene på inputen
                        float(t_s); 
                        float(t_c); 
                        float(r_cal); 
                        float(vpt); 
                        int(adc)
                        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                        #Skriver til filen:
                        f.write(f"{timestamp},{t_s},{t_c},{r_cal},{vpt},{adc}\n")
                        f.flush()       #Skriver/lagrer linjen med en gang
                        print(line)     #Skriver en kopi til console, slik at vi ser den også

                    #Hvis noe uforventet skjer så dropper vi linjen og fortsetter.
                    #Isteden for at hele programmet stopper
                    except Exception:
                        pass
                else:
                    #Skriver all annen input til console. Siden mikrokontrolleren logger "sanntid" også.
                    print(line)
        #Sjekker etter crlt+C for å stoppe logging
        except KeyboardInterrupt:
            print("\nStoppet.")         #Bekrefter stopp
        finally:
            cmd.close()                 #Lukker for lesing


if __name__ == "__main__":
    main()
