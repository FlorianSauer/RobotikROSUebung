<h1> Einleitung: Was macht das Programm & wie ist es zu nutzen?</h1> 

Unser Programm ist eine Erweiterung von Exercise 1, dessen Dokumentation man hier finden kann: <https://github.com/FlorianSauer/RobotikROSUebung/blob/master/Solution.md> 

<h3>Wie startet man das Program? </h3>
1. catkin_make aufrufen
2. roslaunch beleg beleg.launch //startet beleg //bundelt des Startens von camera_pseudo, prediction und sound Nodes
3. Cameradaten müssen vom Hostsystem ans Gastsystem geschickt werden.zu empfehlen ist die VM in bridgemodus zu starten oder eine Tunneling software zu verwenden (ssh Tunnel). In unseren Fall wurde dies mit eine selbstgeschriebene P2P Programs gemacht. 
4. WebcamTransmiter in Hostsystem starten (per python WebcamTransmiter)



Es ist in 3 Hinsichten erweitert: </br> 
1. Bilddaten werden von einer Camera gelesen </br> 
2. Eingelesene Daten werden über ein Peer to peer Netzwerk an ROS geschickt  </br> 
3. Vorhersagen werden jetzt per text to speech ausgegeben




<h2> Schriftliche Ausarbeitung</h2>

<h2> Beschreibung verwendeter Konzepte und Architekturen (Theorie)</h2>

<h2> Beschreibung der Implementierung </h2>

#Project extensions
	# Callback module
	# removed publisher subscriber nodes
	# camera class
	# removed cv bridge dependency


<h2> Graphische Darstellungen </h2>

<h2>Quellenangabe </h2>




