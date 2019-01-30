<h1> Einleitung: Was macht das Programm & wie ist es zu nutzen?</h1> 


<h3>Was macht das Program </h3>
Unser Programm ist eine Erweiterung von Exercise 1, dessen Dokumentation man hier finden kann: <https://github.com/FlorianSauer/RobotikROSUebung/blob/master/Solution.md> 

Es ist in 3 Hinsichten erweitert: </br> 
1. Bilddaten werden von einer Camera gelesen </br> 
2. Eingelesene Daten werden über ein Peer to peer Netzwerk an ROS geschickt  </br> 
3. Vorhersagen werden jetzt per text to speech ausgegeben


<h3>Wie startet man das Program? </h3>
1. catkin_make aufrufen
2. roslaunch beleg beleg.launch //startet beleg //bundelt des Startens von camera_pseudo, prediction und sound Nodes
3. Cameradaten müssen vom Hostsystem ans Gastsystem geschickt werden. Zu empfehlen ist die VM in bridgemodus zu starten oder eine Tunneling software zu verwenden (ssh Tunnel). In unseren Fall wurde dies mit eine selbstgeschriebene P2P Programs gemacht. 
4. WebcamTransmiter in Hostsystem starten (per python WebcamTransmiter)

<h3>Wie ist es zu nutzen? </h3>
das Program erweitert die Funktionalität nämlich so dass man beliebige Bilder von Zahlen ohne weiteres einsetzen kann, was dadurch ermöglicht wird, dass die zu analysierende Daten direkt von der Camera abgefragt werden. 

Weiterhin erlaubt die P2P infrastruktur eine Entkopplung vom Daten auslesen und Daten asuwerten. Und zwar so, dass man die Camera nicht mehr direkt auf der auswertende Maschine haben müsste. Es wäre jetzt denkbar Daten aus einer beliebigen Location zu empfangen und lokal bzw. in einer andere beliebige Location auszuwerten. Dies setzt natäürlich voraus dass die Nodes über das P2P Netz miteinander kommunizieren.

Lediglich haben wir den Sound Node eingefügt als alternative  ausgabe möglichkeit für die Vorhersagen. Dies bietet keine radikal neue Funktionalität an, aber ist ein hinweis auf die Usability möglichkeiten des Systems. Und zwar so, dass ein Mensch nicht unbedignt vor einem Bildschirm sitzen muss um das system einzusetzen, er könnte genauso die Auswertungen des Systems gesagt bekommen. 


<h2> Schriftliche Ausarbeitung</h2>

<h2> Beschreibung verwendeter Konzepte und Architekturen (Theorie)</h2>

- autonomes p2p system (lookup diplomarbeit)
- VM mit NAT oder Bridge network
- NAT und warum wir p2p brauchen + rtunnel
- bridge??!?!?!!!!ausrufezeichen
- Publisher Subscriber (copy paste)
- Callbacks

<h2> Beschreibung der Implementierung </h2>

- vm p2p node macht connection zu public p2p node, asks for routing
- public node creates connection to host p2p node, notifies vm p2p node about success. From now on the Public p2p node routes all traffic between host and vm p2p node.
- vm p2p node informs host p2p node to create a RTunnel.
- WebcamTransmitter connects on host system to tunnelentry on host p2p node
- host p2p node forwards tunnel data to the recipient (vm p2p node)
- vm p2p node forwards tunnled data to the destination (ros-node camera_pseudo)
- ros does ros things


#Project extensions
    # camera_pseudo accepts image data via TCP sockets (socket server)
    # 
	# removed not needed publisher subscriber topics in camera_pseudo and prediction nodes
	# camera Program to read images from webcam and send via TCP socket to the camera_pseudo-TCP-server
    # added sound node for sound output
 

<h2> Graphische Darstellungen </h2>

<h2>Quellenangabe </h2>

- P2P Diplomarbeit (https://domino.mpi-inf.mpg.de/intranet/ag5/ag5publ.nsf/0/FC279B9D6374B04CC1256FDC00351C82/$file/Diplom_Schade.pdf)
- ROS-Docu
- PyGame for sound
- typing module for python2 typing support
- 

