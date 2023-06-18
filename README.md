# camera_view
Topside package. Displays the cameras adds the overlay.
Listens for a ping from cameras on 192.168.1.100:12345, and once it detects the ping, it streams video from 192.168.1:5000.
Can be launched before or after cameras have begun streaming, and more cameras can be added automatically mid session. This code was originally called ROVMIND, but it was moved to this current repository during a restructure in 2022-2023 season.

## Contributors

* Current maintainers: Alex Bertran, James Randall

* Contributors:
  * Andrew Grindstaff '21 - Initial work on RPICamera
  * Alex Bertran '24 - Camera Feed Overlay
  * James Randall - Rewrite in 2022-2023 season
