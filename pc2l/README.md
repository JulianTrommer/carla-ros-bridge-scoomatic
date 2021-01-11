# PointCloud to Laserscan

Dieses ROS Paket realisiert die Umwandlung zwischen PointCloud- zu LaserScan-Nachrichten.

# Features

- [x] Umwandlung PointCloud2- zu LaserScan-Nachrichten

# Setup

Um das Paket zu nutzen muss das Paket *pointcloud\_to\_laserscan* installiert werden:

```
sudo apt install ros-melodic-pointcloud-to-laserscan
```

# Start des pc2l

pc2l wird beim Starten des GMappings simultan gestartet.

Soll der pc2l separat gestartet werden, kann dies über diesen Befehl erfolgen:

```
roslaunch pc2l pointcloud_to_laserscan.launch
```

# pc2l

Das pc2l Paket basiert komplett auf dem Paket (*pointcloud\_to\_laserscan*)[http://wiki.ros.org/pointcloud_to_laserscan] und implementiert im Grunde nur ein Launch-File, das auf den benötigten Anwendungsfall angepasst werden kann.

Durch Modifizieren des Launch Files kann die Node auf z.B. die Frequenz der PointCloud-Nachrichten, das Topic der Nachrichten, die maximale Höhe von Objekten, die minimale Höhe von Objekten und vieles mehr angepasst werden.