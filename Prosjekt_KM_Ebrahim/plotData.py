import argparse                  #Brukes til å kunne gi info via console
import csv                       #Brukes til å jobbe med csv filer
import matplotlib.pyplot as plt  #Brukes til å plotte og visualisere graft
import datetime as dt            #Brukes til å sette tider

def main():
    p = argparse.ArgumentParser(description="Plott temperatur fra data.csv (med timestamp).")

    #Definer hvilken fil som skal leses av, kreves ikke, default "data.csv".
    p.add_argument("--file", default="data.csv")
    
    #Leser verdiene fra bruker.
    args = p.parse_args()

    #Lager listene som tid og temperatur skal lagres i
    tid = []
    temps = []

    with open(args.file, "r", newline="") as f:     #Åpner filen for lesing
        rdr = csv.reader(f)                         #leser rad for rad
        header = next(rdr, None)                    #Inkluderer ikke header

        for row in rdr:     #For hver rad
            if not row:     #Hvis rad er tom, fortsett (feilhåndtering)
                continue
            try:
                timestamp_str = row[0].strip()                                          #Tar ut tiden, rad 1
                temp_c = float(row[2])                                                  #Tar ut temperaturen, rad 3 + konvertering til float
                timestamp = dt.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")    #Tar ut tid fra PCen

                tid.append(timestamp)   #legger tid inn i listen
                temps.append(temp_c)    #legger temp inn i listen

            #Hvis noe rart dukker opp, hopp over linejn og fortsett (feilhandtering)
            except Exception:
                continue

    #Hvis tid er tom, print feilmelding
    if not tid:
        print("Ingen gyldige linjer i filen (sjekk formatet).")
        return

    print(f"Leste {len(tid)} gyldige målinger.")        #Skriv ut antall målinger

    #Visualisere plot av data, en graf:
    plt.figure(figsize=(9,4))
    plt.plot(tid, temps, marker="o", linewidth=1)
    plt.title("Pt100 temperaturmåling")
    plt.xlabel("Tid (dato + klokkeslett)")
    plt.ylabel("Temperatur (°C)")
    plt.grid(True)
    plt.gcf().autofmt_xdate()   # gjør datoene lesbare
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
