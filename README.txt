# Laboratorio

Progetto Collision Avoidance LUIGI ANTONELLI 1851425:

Il sistema prende in input un comando di velocità e uno scan e produce un comando di velocità aggiornato in modo tale che il robot non vada a sbattere.

Ho utilizzato due subscriber, uno per il comando di velocità dell'utente e uno per il laser scan, e un publisher che invia un geometry message Twist al robot.

Dopo aver attivato i primi 5 comandi del web controlle, per avviare il programma bisogna eseguire il comando "rosrun progetto progetto" nel catkin workspace e in un altro terminale il comando "rostopic pub /cmd_vel_user geometry.msgs/Twist..." per inviare un comando di velocità.
