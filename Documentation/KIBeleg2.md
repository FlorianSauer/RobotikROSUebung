<h1> Einleitung: Was macht das Programm & wie ist es zu nutzen?</h1> 


<h3>Was macht das Program </h3>
Unser Programm ist eine Erweiterung von Exercise 1, dessen Dokumentation man hier finden kann: <https://github.com/FlorianSauer/RobotikROSUebung/blob/master/Solution.md> 

Es ist in 3 Hinsichten erweitert: </br> 
1. Bilddaten werden von einer Camera gelesen </br> 
2. Eingelesene Daten werden über ein Peer to peer Netzwerk an ROS geschickt  </br> 
3. Vorhersagen werden jetzt per text to speech ausgegeben </br> 
</br>

<mark> Bevor man loslegt sollte man sicherstellen dass man sich auf dem Sound Branch befindet </mark>
<h3>Wie startet man das Program? </h3>

1. catkin_make aufrufen
2. roslaunch beleg beleg.launch //startet beleg //bundelt des Startens von camera_pseudo, prediction und sound Nodes
3. Cameradaten müssen vom Hostsystem ans Gastsystem geschickt werden. Zu empfehlen ist die VM in bridgemodus zu starten oder eine Tunneling software zu verwenden (ssh Tunnel). In unseren Fall wurde dies mit eine selbstgeschriebene P2P Programs gemacht. 
4. WebcamTransmiter in Hostsystem starten (per python WebcamTransmiter)

<h3>Wie ist es zu nutzen? </h3>
das Program erweitert die Funktionalität nämlich so dass man beliebige Bilder von Zahlen ohne weiteres einsetzen kann, was dadurch ermöglicht wird, dass die zu analysierende Daten direkt von der Camera abgefragt werden. 

Weiterhin erlaubt die P2P infrastruktur eine Entkopplung vom Daten auslesen und Daten asuwerten. Und zwar so, dass man die Camera nicht mehr direkt auf der auswertende Maschine haben müsste. Es wäre jetzt denkbar Daten aus einer beliebigen Location zu empfangen und lokal bzw. in einer andere beliebige Location auszuwerten. Dies setzt natäürlich voraus dass die Nodes über das P2P Netz miteinander kommunizieren.

Lediglich haben wir den Sound Node eingefügt als alternative  ausgabe möglichkeit für die Vorhersagen. Dies bietet keine radikal neue Funktionalität an, aber ist ein hinweis auf die Usability möglichkeiten des Systems. Und zwar so, dass ein Mensch nicht unbedignt vor einem Bildschirm sitzen muss um das system einzusetzen, er könnte genauso die Auswertungen des Systems gesagt bekommen. 



 


