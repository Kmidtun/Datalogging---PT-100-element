Programmet skal måle temperatur og starte en vifte når en bestemt temp. er nådd.

Temperatur måles ved hjelp av et PT100 element, hvor vi bruker en spenningsdeler for å ta ut ADC verdien og gjør omregninger med denne.

Viften styres enkelt med en av og på funksjon, og en LED indikerer drift/på.

Dataen fra PT100 elementet skal logges og brukes til å generere en graf over temperaturen. Dataen overføres via Usart3.

Datalogging og plotting i grad blir kjørt av to separate Python program.

serialRead.py
plotData.py

Disse to filene kjøres fra cmd. sett direktivet til mappe lokasjonen.

bruke #cd i console.
For å kjøre serialRead: #python serialRead.py --port "COM3" (COM3 endrer du ved bruk av annen port)
For å kjøre plotData.py #python plotData.py