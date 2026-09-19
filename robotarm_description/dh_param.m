%Darstellung Roboter mit Denavit Hartenberg Parametern

%SNOOPY
K(1)=Link([0 350  25  -pi/2]) %Denavit Hartenberg Parameter (theta (0=variabel), d,a, alpha) der einzelnen Körper eingeben
K(2)=Link([0 18.5 300 0])
K(3)=Link([0 0    15  -pi/2])
K(4)=Link([0 252  0   pi/2])
K(5)=Link([0 0    0   -pi/2])
K(6)=Link([0 148  0   0])
SNOOPY = SerialLink(K,'name','SNOOPY') %Objekt Roboter erzeugen
Winkel= ([0 0 -pi/2 0 0 0]) %Stellung theta der Gelenke eingeben
axis auto %Abschalten der automatischen Achseinstellung
SNOOPY.plot(Winkel) 
SNOOPY.fkine(Winkel) %Berechnung der direkten Kinematik als homogene Matrix