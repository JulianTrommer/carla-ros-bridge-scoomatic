# Odometry Publisher

Dieses ROS Paket erstellt den für verschiedene Navigationspakete benötigten TF-Baum.

# Features

- [x] Setzen des Odometrie-Nullpunkts *odom\_origin* (Hilfsframe)
- [x] Lookup von Odometrie-Nullpunkt zu Ego Vehicle (*odom\_origin -> ego\_vehicle*)
- [x] Setzen der Transformation von *odom\_base\_link* zu *base\_link*
- [x] Setzen der Transformation von *base\_link* zu *lidar\_base\_link*
- [x] Setzen der Transformation von *map* zu *odom\_base\_link*

# Setup

Der Odometry Publisher ist ein relatives simples Paket und foredert keine zusätzlichen Installationen.

# Start des Odometry Publishers

Beim Starten des GMappings wird der Odometry Publisher mitgestartet.

Um den Odometry Publisher separat zu starten kann folgender Befehl eingegeben werden:
```
roslaunch odom_publisher odom_publisher.launch
```

# ROS Topics

## Subscriber

|Topic                                 | Typ | Beschreibung |
|--------------------------------------|------|-------------|
| `/ego_vehicle/odom_data` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) | Odometrie Nachricht des Fahrzeugs |
| `/ego_vehicle/laserscan` | [sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) | LaserScan-Daten des Fahrzeugs |
| `/ego_vehicle/lidar` | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | Lidar Daten des Fahrzeugs |

## Publisher

|Topic                                 | Type | Description |
|--------------------------------------|------|-------------|
| `/odom` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) | Odometrie Nachricht des Fahrzeugs |
| `/base_scan_laser` | [sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) | LaserScan-Daten des Fahrzeugs |
| `/base_scan_lidar` | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | Lidar Daten des Fahrzeugs |

# Weitere Informationen

## Erstellung des TF-Baums

Das Paket erstellt den Teilbaum *odom\_base\_link -> base\_link -> lidar\_base\_link* (siehe [REP105](https://www.ros.org/reps/rep-0105.html)). Nach Bedarf wird dann noch der Teilbaum an den *map*-Baum angehängt, um diese zu verbinden. Soll der Teilbaum verbunden werden, kann dies mithilfe der erweiterten Abstraction Bridge mit Lokalisierung erfolgen. Sollen die Teilbäume nicht verbunden werden, wird dies durch Ausführen der erweiterten Abstraction Bridge ohne Lokalisierung erreicht.

Um den Teilbaum zu erstellen wird folgendermaßen vorgegangen:
1. Setzen des Odometrie-Nullpunktes: Als erstes wird der Frame *odom\_origin* entweder in den Frame *map* oder den Frame *ego\_vehicle* gelegt.
2. Transformation *odom\_origin -> ego\_vehicle* nachschauen und das Ergebnis als *odom\_base\_link -> base\_link* publishen.
3. Transformation *base\_link -> lidar\_base\_link* publishen. Dies ist eine statische Transformation und immer gleich (mit der benutzten Konfiguration befand sich der Lidar lediglich 2.4 Meter in z-Richtung über dem Fahrzeug-Nullpunkt. Deshalb beinhaltet die Transformation in diesem Fall keine Rotation und nur eine Translation in z-Richtung um 2.4).
4. Wiederhole ab Schritt 2.

## Anpassen der Header

Dieser Schritt ist zwar nicht wirklich nötig, wird allerdings aus Konsistenzgründen empfohlen. Da in den Navigationspaketen mit den neu erstellten Frames gearbeitet wird, sollten die Header der Sensor-Nachrichten angepasst werden. Manche Pakete benutzen den im Header der Sensor-Nachrichten hinterlegten Frame um im TF-Baum den Frame des Sensors zu identifizieren. Da der Frame *ego\_vehicle* jedoch gleich *base\_link* ist, ist dies nicht weiter schlimm.

Aus Konsistenzgründen ist es allerdings viel schöner, wenn einfach neue Sensornachrichten aus den vorhandenen erstellt werden, bei welchen in dem Header die angepassten Frames des neuen Teilbaums stehen.
